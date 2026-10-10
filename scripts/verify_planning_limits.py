"""同一规划限值文件的正/负向验收；不运行尚未实现的轨迹规划器。"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import platform
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from planning.limits import load_planning_limits, DEFAULT_CONFIG, DEFAULT_MODEL
from adapters.c_kinematics import ContractError


def main():
    """读取统一配置并验证缺项闸门/缩放/模长规则；返回0通过、1失败，保存逐例报告。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--output', type=Path, default=ROOT/'results/planning-limits')
    args = parser.parse_args()
    records = []
    effective = None

    def check(name, function, expected=0):
        try:
            value = function()
            code = 0
            data = value.as_dict() if hasattr(value, 'as_dict') else value
        except ContractError as exc:
            code = exc.code; data = None
        except Exception as exc:
            code = 9000; data = None
            records.append(dict(name=name, expected_code=expected, actual_code=code,
                                passed=False, data=None, exception=str(exc)))
            return None
        records.append(dict(name=name, expected_code=expected, actual_code=code,
                            passed=code == expected, data=data))
        return value if code == 0 else None

    effective = check('complete_shared_config', lambda: load_planning_limits(args.config, args.model))
    if effective is not None:
        check('joint_equal_actual_limits', lambda: effective.check_joint_rates(
            effective.joint_velocity_rad_s, effective.joint_acceleration_rad_s2))
        check('joint_speed_over', lambda: effective.check_joint_rates(
            [effective.joint_velocity_rad_s[0]+1e-8]+[0]*5, [0]*6), 3002)
        check('joint_acceleration_over', lambda: effective.check_joint_rates(
            [0]*6, [effective.joint_acceleration_rad_s2[0]+1e-8]+[0]*5), 3002)
        check('zero_tcp', lambda: effective.check_tcp_rates(*([[0,0,0]]*4)))
        rates = [[effective.linear_velocity_m_s,0,0], [effective.angular_velocity_rad_s,0,0],
                 [effective.linear_acceleration_m_s2,0,0], [effective.angular_acceleration_rad_s2,0,0]]
        check('tcp_equal_actual_limits', lambda: effective.check_tcp_rates(*rates))
        for index, label in enumerate(('linear_velocity','angular_velocity','linear_acceleration','angular_acceleration')):
            values = [[0,0,0] for _ in range(4)]
            limit = rates[index][0]; values[index] = [limit*.8, limit*.8, 0]
            check('tcp_norm_over_not_component_'+label, lambda v=values: effective.check_tcp_rates(*v), 3002)
        check('request_scaling_product', lambda: (
            load_planning_limits(args.config, args.model, velocity_scale=.5, acceleration_scale=.5).linear_velocity_m_s
            == effective.linear_velocity_m_s*.5))
        if records[-1]['data'] is not True: records[-1]['passed'] = False
        check('hardware_use_rejected', lambda: load_planning_limits(args.config,args.model,usage='hardware_validated'), 1008)
        check('request_scale_over_one', lambda: load_planning_limits(args.config,args.model,velocity_scale=1.1), 1001)
        check('request_scale_zero', lambda: load_planning_limits(args.config,args.model,acceleration_scale=0), 1001)
        check('wrong_rate_shape', lambda: effective.check_joint_rates([0]*5,[0]*6), 1001)
        check('nonfinite_rate', lambda: effective.check_tcp_rates([float('nan'),0,0],*[ [0,0,0] ]*3), 1001)
    try:
        original = json.loads(args.config.read_text())
        with tempfile.TemporaryDirectory() as folder:
            test_file = Path(folder)/'limits.json'
            missing_paths = [('joint_limits','velocity_rad_s'), ('joint_limits','acceleration_rad_s2')]
            missing_paths += [('tcp_limits',key) for key in ('linear_velocity_m_s','angular_velocity_rad_s','linear_acceleration_m_s2','angular_acceleration_rad_s2')]
            missing_paths += [('tcp',key) for key in ('frame','point','expression_frame','limit_metric','T_tool0_tcp')]
            missing_paths += [('scales','velocity'),('scales','acceleration')]
            for section, key in missing_paths:
                for kind in ('missing','null'):
                    config = copy.deepcopy(original)
                    if kind=='missing': del config[section][key]
                    else: config[section][key] = None
                    test_file.write_text(json.dumps(config))
                    check(kind+'_'+section+'_'+key, lambda: load_planning_limits(test_file,args.model),1004)
            bad = [
                ('negative_acceleration',('joint_limits','acceleration_rad_s2'),[-1]*6,1001),
                ('infinite_tcp',('tcp_limits','linear_velocity_m_s'),float('inf'),1001),
                ('bool_tcp',('tcp_limits','linear_acceleration_m_s2'),True,1001),
                ('wrong_units',('units','linear_velocity'),'mm/s',1002),
                ('component_policy',('tcp','limit_metric'),'per_component',1007),
                ('different_tool',('tcp','frame'),'flange',1007),
                ('speed_above_model',('joint_limits','velocity_rad_s'),[4]*6,1001)]
            for name,(section,key),value,expected in bad:
                config=copy.deepcopy(original);config[section][key]=value;test_file.write_text(json.dumps(config))
                check(name,lambda:load_planning_limits(test_file,args.model),expected)
            config=copy.deepcopy(original);config['model_sha256']='0'*64;test_file.write_text(json.dumps(config))
            check('model_hash_drift',lambda:load_planning_limits(test_file,args.model),1004)
            check('missing_config_file',lambda:load_planning_limits(Path(folder)/'absent.json',args.model),1004)
    except (OSError, ValueError) as exc:
        records.append(dict(name='negative_fixture_preparation',passed=False,exception=str(exc)))
    report = dict(status='passed' if records and all(r['passed'] for r in records) else 'failed',
                  expected_count=len(records), passed_count=sum(r['passed'] for r in records),
                  cases=records,effective_limits=effective.as_dict() if effective else None,
                  python=platform.python_version(),scope='simulation parameter gate only; planner not implemented',
                  loader_sha256=hashlib.sha256((ROOT/'src/planning/limits.py').read_bytes()).hexdigest(),
                  verifier_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('status','expected_count','passed_count')}))
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
