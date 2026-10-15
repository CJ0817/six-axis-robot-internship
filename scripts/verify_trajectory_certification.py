"""End-to-end joint certification and execution rejection regressions."""
import argparse
import copy
import json
from pathlib import Path
import sys
import tempfile
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import numpy as np
import pybullet as bullet
from adapters.c_kinematics import ContractError
from planning.cartesian import CartesianPlanner
from planning.collision import StaticScene
from planning.execution import execute_trajectory
from planning.joint import JointPlanner


def main():
    """Run real C/FK/URDF checks; persist every success and failure Result."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so')
    parser.add_argument('--planning-library',type=Path,default=ROOT/'build/librobot_planning.so')
    parser.add_argument('--output',type=Path,default=ROOT/'results/trajectory-certification')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cases=[]
    profile=json.loads((ROOT/'models/ur5/kinematics.json').read_text())
    definition=json.loads((ROOT/'scenes/static-apf.v1.json').read_text())
    start=[.3,-1.2,1.1,-1.1,1.,.2];goal=[.36,-1.17,1.06,-1.08,1.01,.2]
    options=dict(method='quintic',sample_period_s=.02,duration_s=8.,duration_policy='strict',max_duration_s=30.)
    scene=StaticScene(definition,profile)
    planner=JointPlanner(args.planning_library)

    def record(name,result,expected):
        """Persist raw Result and verify fail-closed data and execution behavior."""
        (args.output/(name+'.json')).write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
        passed=result['code']==expected
        if expected:
            passed &= result['data'] is None and result.get('next_state') is None
            passed &= bool(result['details'].get('failure_log'))
            calls=[];rejected=execute_trajectory(result,scene,calls.append)
            passed &= rejected['code']!=0 and not calls
        cases.append(dict(id=name,passed=bool(passed),expected_code=expected,actual_code=result['code']))
        print(name,result['code'],passed,flush=True)
        return result

    def plan(opts=None,instance=None,active_scene=scene,end=goal,**kwargs):
        """Invoke the complete public joint planning entry, with real scene/FK."""
        return (instance or planner).plan_joint(start,end,opts or options,scene=active_scene,
                                               kinematics_library=args.library,**kwargs)

    try:
        candidate=planner.plan_candidate(start,goal,options)
        calls=[]
        cases.append(dict(id='candidate-explicitly-nonexecutable',passed=candidate['code']==0 and
            candidate['data']['trajectory_status']=='candidate' and not candidate['data']['executable']))
        record('candidate-execution-rejected',execute_trajectory(candidate,scene,calls.append),3001)
        for method in ('cubic','quintic','trapezoidal'):
            valid=record(method+'-fully-certified',plan(dict(options,method=method)),0)
            if valid['data']:
                d=valid['data'];called=[]
                dispatched=execute_trajectory(valid,scene,called.append)
                cases.append(dict(id=method+'-execution-accepted',passed=dispatched['code']==0 and len(called)==1 and
                    d['collision_checked'] and d['continuous_collision_proof'] and d['continuous_tcp_proof']))
                original=planner.plan_candidate(start,goal,dict(options,method=method))['data']
                cases.append(dict(id=method+'-original-polynomial-preserved',passed=
                    d['coefficients_rad']==original['coefficients_rad'] and d['segment_times_s']==original['segment_times_s']))
        valid=plan()
        for field in ('q_rad','qd_rad_s','qdd_rad_s2','coefficients_rad','segment_times_s'):
            changed=copy.deepcopy(valid)
            if field=='coefficients_rad':changed['data'][field][0][0][0]+=.01
            elif field=='segment_times_s':changed['data'][field][-1]+=.1
            else:changed['data'][field][0][0]+=.01
            record('tampered-'+field,execute_trajectory(changed,scene,calls.append),3001)
        forged=copy.deepcopy(candidate);forged['data'].update(trajectory_status='certified',executable=True)
        record('forged-flags-rejected',execute_trajectory(forged,scene,calls.append),3001)
        record('joint-rate-strict',plan(dict(options,duration_s=.1)),3002)
        record('joint-position',plan(end=[profile['q_max_rad'][0]+.01]+start[1:]),3002)
        record('missing-scene',plan(active_scene=None),1004)
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)
            record('missing-limits',plan(instance=JointPlanner(args.planning_library,directory/'missing.json')),1004)
            record('missing-certification-settings',plan(certification_config=directory/'missing.json'),1004)
            source=json.loads((ROOT/'config/planning-limits.v1.json').read_text())
            for key in source['tcp_limits']:
                changed=copy.deepcopy(source);changed['tcp_limits'][key]=1e-5
                path=directory/(key+'.json');path.write_text(json.dumps(changed))
                record('tcp-'+key,plan(instance=JointPlanner(args.planning_library,path)),3002)
            changed=copy.deepcopy(source);changed['tcp_limits']['linear_velocity_m_s']=.04
            path=directory/'stretch.json';path.write_text(json.dumps(changed))
            adjusted=JointPlanner(args.planning_library,path)
            record('tcp-strict-before-stretch',plan(instance=adjusted),3002)
            stretched=record('tcp-stretch-recertified',plan(dict(options,duration_policy='stretch'),instance=adjusted),0)
            if stretched['data']:
                attempts=stretched['details']['attempts']
                cases.append(dict(id='stretch-has-failure-and-new-certification',passed=
                    any(a['stage']=='certification' and a['code']==3002 for a in attempts) and
                    attempts[-1]['code']==0 and stretched['data']['actual_duration_s']>options['duration_s'] and
                    execute_trajectory(stretched,scene,calls.append)['code']==0))
                path.write_text(json.dumps(source))
                record('configuration-changed-after-certification',execute_trajectory(stretched,scene,calls.append),3001)
                path.unlink()
                record('configuration-deleted-after-certification',execute_trajectory(stretched,scene,calls.append),1004)
            settings=json.loads((ROOT/'config/cartesian-planning.v1.json').read_text())
            settings['collision_subdivisions']=1;settings['collision_max_joint_step_rad']=10.;settings['sample_period_s']=8.
            path=directory/'proof-budget.json';path.write_text(json.dumps(settings))
            record('clearance-proof-fails-closed',plan(dict(options,sample_period_s=8.),certification_config=path),3001)
        blocked=copy.deepcopy(definition)
        # Add a blocker at the upper-arm link's center, away from the TCP.
        scene.check(start)
        bounds=bullet.getAABB(scene.robot,2,physicsClientId=scene.client)
        center=((np.array(bounds[0])+np.array(bounds[1]))/2).tolist()
        blocked['obstacles'].append(dict(id='upperarm-blocker',kind='sphere',position_m=center,radius_m=.04))
        arm=StaticScene(blocked,profile)
        try:record('whole-arm-collision',plan(active_scene=arm),3003)
        finally:arm.close()
        folded=[-.3742565963470028,-2.3843366984624392,-.8970081111253569,1.1288944496614386,-.2917323291194913,-1.869156334633904]
        record('self-collision',planner.plan_joint(folded,folded,options,scene=scene,kinematics_library=args.library),3003)
        # Endpoint-safe but colliding interior: use existing fixed regression poses.
        a=np.array(folded);b=a.copy();a[3]-=2.;b[3]+=2.
        safe=scene.check(a.tolist())['passed'] and scene.check(b.tolist())['passed']
        options_slow=dict(options,duration_s=30.,method='cubic')
        interior=record('safe-endpoints-colliding-interior',planner.plan_joint(a.tolist(),b.tolist(),options_slow,scene=scene,kinematics_library=args.library),3003)
        cases.append(dict(id='interior-fixture-endpoints-safe',passed=safe))
        definition['revision']='changed-after-certification'
        record('scene-changed',execute_trajectory(valid,scene,calls.append),3001)
    except Exception as exc:
        cases.append(dict(id='unexpected-regression-error',passed=False,message=str(exc)))
        (args.output/'unexpected-error.txt').write_text(traceback.format_exc(),encoding='utf-8')
    finally:
        scene.close()
    report=dict(status='passed' if all(c['passed'] for c in cases) else 'failed',
                passed_count=sum(c['passed'] for c in cases),expected_count=len(cases),cases=cases)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':sys.exit(main())
