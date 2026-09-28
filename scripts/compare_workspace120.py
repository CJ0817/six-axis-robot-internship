"""Run frozen workspace120 against C FK/IK and explicit-parameter Robotics Toolbox."""
import argparse
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import sys
import time
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import FKModel,IKOptionsV1,IKResultV1,D6,D16,pack_model,Forward
from verify_c_ik import pose_error,stats
SEED=20260928+1

def error_stats(rows,field):
    return stats([c[field] for r in rows for c in r['candidates'] if c[field] is not None])

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so');parser.add_argument('--output',type=Path,default=ROOT/'results/workspace120-batch');args=parser.parse_args()
    model_path=ROOT/'models/ur5/kinematics.json';main_path=ROOT/'tests/ik/workspace120.json';special_path=ROOT/'tests/ik/workspace120-special.json'
    model_data=json.loads(model_path.read_text());fixture=json.loads(main_path.read_text());special=json.loads(special_path.read_text())
    digest=hashlib.sha256(model_path.read_bytes()).hexdigest()
    assert fixture['model_sha256']==special['model_sha256']==digest
    assert fixture['sample_count']==len(fixture['samples'])==120 and len(fixture['coverage'])==24 and set(fixture['coverage'].values())=={5}
    assert special['boundary_count']==len(special['boundaries'])==16 and special['exception_count']==len(special['exceptions'])==11
    expected=fixture['expected_rules'];assert expected['geometric_reachable'] and expected['fk']['expected_code']==0
    assert expected['ik']['regular_target']['expected_code']==0 and expected['ik']['execution_status']=='not_run'
    pos_tol=expected['ik']['regular_target']['every_retained_candidate_fk_position_error_m_max']
    ori_tol=expected['ik']['regular_target']['every_retained_candidate_fk_orientation_error_rad_max']
    matrix_tol=expected['reference_data_validation']['matrix_max_abs_error_max']
    model=pack_model(model_data);reference=rtb.DHRobot([rtb.RevoluteDH(a=model_data['a_m'][i],d=model_data['d_m'][i],alpha=model_data['alpha_rad'][i],offset=model_data['theta_offset_rad'][i]) for i in range(6)],base=SE3(np.array(model_data['T_base_dh0'])),tool=SE3(np.array(model_data['T_dh6_flange']) @ np.array(model_data['T_flange_tool'])))
    forward=Forward(args.library);inverse=forward.lib.robot_inverse_v1
    inverse.argtypes=[C.POINTER(FKModel),C.POINTER(C.c_double),C.c_size_t,C.POINTER(C.c_double),C.c_size_t,C.POINTER(IKOptionsV1),C.POINTER(IKResultV1)];inverse.restype=C.c_int
    options=IKOptionsV1(1,1,pos_tol,ori_tol,0,.2,0,0,0,0,0)
    rng=random.Random(SEED);samples=[];boundaries=[];exceptions=[]
    def ref(q):return reference.fkine(np.array(q)*model_data['joint_sign']).A
    def fk(q):
        out=D16();code=forward.fn(C.byref(model),D6(*q),6,out)
        return code,list(out) if code==0 else None
    def solve(T,qseed):
        out=IKResultV1();C.memset(C.byref(out),0x5A,C.sizeof(out));before=bytes(out)
        t=D16(*T);q=D6(*qseed);start=time.perf_counter_ns()
        code=inverse(C.byref(model),t,16,q,6,C.byref(options),C.byref(out))
        wall_ns=time.perf_counter_ns()-start
        return code,out,bytes(out)==before,wall_ns
    def evaluate(loaded,target,seed,expected_ik):
        q=loaded['q_rad'];target_array=np.array(target);rref=ref(q).ravel();fcode,forward_values=fk(q)
        row=dict(target_id=loaded['target_id'],region=loaded.get('region'),q_witness_rad=q,q_seed_rad=seed,target_T_base_tool=target,reference_matrix_max_abs_error=float(np.max(np.abs(target_array.ravel()-rref))),fk_code=fcode,ik_code=None,ik_expected_code=expected_ik,candidate_count=None,candidates=[],c_function_s=None,python_to_c_call_ns=None,output_unchanged_on_failure=None,passed=False,reason=None)
        if fcode==0:
            row['fk_position_error_m'],row['fk_orientation_error_rad']=pose_error(forward_values,target_array.ravel())
            row['c_vs_rtb_matrix_max_abs_error']=float(np.max(np.abs(np.array(forward_values)-ref(q).ravel())))
            row['c_vs_rtb_position_error_m'],row['c_vs_rtb_orientation_error_rad']=pose_error(forward_values,ref(q).ravel())
        else:
            row['fk_position_error_m']=row['fk_orientation_error_rad']=None
            row['c_vs_rtb_matrix_max_abs_error']=row['c_vs_rtb_position_error_m']=row['c_vs_rtb_orientation_error_rad']=None
        code,out,untouched,call_ns=solve(target_array.ravel(),seed)
        row.update(ik_code=code,candidate_count=out.solution_count if code==0 else None,c_function_s=out.elapsed_s if code==0 else None,python_to_c_call_ns=call_ns,output_unchanged_on_failure=untouched if code else None)
        try:
            assert row['reference_matrix_max_abs_error']<=matrix_tol
            assert fcode==0 and row['fk_position_error_m']<=expected['fk']['position_error_m_max'] and row['fk_orientation_error_rad']<=expected['fk']['orientation_error_rad_max']
            assert row['c_vs_rtb_matrix_max_abs_error']<=matrix_tol
            assert code==expected_ik,f'IK {code}; expected {expected_ik}'
            if code:
                assert untouched
            else:
                assert 1<=out.solution_count<=8
                for k in range(out.solution_count):
                    cand=list(out.candidates_rad[k]);fc,B=fk(cand)
                    pe,re=pose_error(B,target_array.ravel()) if fc==0 else (None,None)
                    rp,rr=pose_error(ref(cand).ravel(),target_array.ravel())
                    limits=all(lo<=v<=hi for v,lo,hi in zip(cand,model.q_min_rad,model.q_max_rad))
                    valid=fc==0 and limits and pe is not None and all(math.isfinite(v) for v in (pe,re,rp,rr)) and pe<=pos_tol and re<=ori_tol and rp<=pos_tol and rr<=ori_tol
                    row['candidates'].append(dict(target_id=loaded['target_id'],candidate_index=k,q_rad=cand,position_error_m=pe,orientation_error_rad=re,rtb_position_error_m=rp,rtb_orientation_error_rad=rr,within_limits=limits,passed=valid))
                assert len(row['candidates'])==out.solution_count
                assert all(c['passed'] for c in row['candidates'])
                qq=[c['q_rad'] for c in row['candidates']];assert qq==sorted(qq)
                assert list(out.q_rad)==qq[out.selected_index]
                for i,v in enumerate(qq):
                    for w in qq[:i]:assert max(abs(a-b) for a,b in zip(v,w))>1e-9
            row['passed']=True
        except (AssertionError,ValueError,TypeError) as exc:row['reason']=str(exc)
        return row
    for case in fixture['samples']:
        q=case['q_rad'];seed=[min(max(v+rng.uniform(-.05,.05),lo),hi) for v,lo,hi in zip(q,model.q_min_rad,model.q_max_rad)]
        # Frozen main group is not exempt merely because a solver reports a singularity.
        row=evaluate(case,case['target_T_base_tool'],seed,0)
        row['witness_sin_q5_abs']=abs(math.sin(model.joint_sign[4]*q[4]+model.theta_offset_rad[4]))
        samples.append(row)
    for case in special['boundaries']:
        q=case['q_rad'];singular=abs(math.sin(model.joint_sign[4]*q[4]+model.theta_offset_rad[4]))<1e-12
        row=evaluate(case,case['target_T_base_tool'],q,2003 if singular else 0)
        row['wrist_singular_witness']=singular;boundaries.append(row)
    def decode(v):
        if v=='NaN':return float('nan')
        if v=='+Inf':return float('inf')
        return float(v)
    for case in special['exceptions']:
        payload=case['payload'];out=D16();C.memset(C.byref(out),0x5A,C.sizeof(out));before=bytes(out)
        t0=time.perf_counter_ns()
        if case['operation']=='forward':
            q=[decode(v) for v in payload['q_rad']];code=forward.fn(C.byref(model),(C.c_double*len(q))(*q),len(q),out)
        else:
            T=[decode(v) for row in payload['target_T_base_tool'] for v in row];q=[decode(v) for v in payload['q_seed_rad']]
            result=IKResultV1();C.memset(C.byref(result),0x5A,C.sizeof(result));previous=bytes(result)
            code=inverse(C.byref(model),D16(*T),16,D6(*q),6,C.byref(options),C.byref(result))
            before=previous;out=result
        elapsed=time.perf_counter_ns()-t0
        exceptions.append(dict(target_id=case['target_id'],operation=case['operation'],expected_code=case['expected_code'],code=code,output_unchanged=(bytes(out)==before),python_to_c_wrapper_ns=elapsed,passed=code==case['expected_code'] and bytes(out)==before))
    by_region={}
    for region in fixture['coverage']:
        group=[r for r in samples if r['region']==region]
        by_region[region]=dict(expected=len(group),passed=sum(r['passed'] for r in group),singular_exceptions=sum(r['ik_code']==2003 for r in group),candidate_count=sum(len(r['candidates']) for r in group),c_vs_rtb_matrix_max_abs=max(r['c_vs_rtb_matrix_max_abs_error'] for r in group))
    summary=dict(main=dict(expected=len(samples),passed=sum(r['passed'] for r in samples),failed=sum(not r['passed'] for r in samples),solver_success_rate=sum(r['ik_code']==0 and r['passed'] for r in samples)/len(samples),candidate_count=sum(len(r['candidates']) for r in samples),c_vs_rtb_matrix_max_abs=stats([r['c_vs_rtb_matrix_max_abs_error'] for r in samples if r['c_vs_rtb_matrix_max_abs_error'] is not None]),c_vs_rtb_position_m=stats([r['c_vs_rtb_position_error_m'] for r in samples if r['c_vs_rtb_position_error_m'] is not None]),c_vs_rtb_orientation_rad=stats([r['c_vs_rtb_orientation_error_rad'] for r in samples if r['c_vs_rtb_orientation_error_rad'] is not None]),c_fk_position_m=stats([r['fk_position_error_m'] for r in samples if r['fk_position_error_m'] is not None]),c_fk_orientation_rad=stats([r['fk_orientation_error_rad'] for r in samples if r['fk_orientation_error_rad'] is not None]),ik_c_fk_position_m=error_stats(samples,'position_error_m'),ik_c_fk_orientation_rad=error_stats(samples,'orientation_error_rad'),ik_rtb_fk_position_m=error_stats(samples,'rtb_position_error_m'),ik_rtb_fk_orientation_rad=error_stats(samples,'rtb_orientation_error_rad')),boundaries=dict(expected=len(boundaries),passed=sum(r['passed'] for r in boundaries),singular_expected=sum(r['ik_expected_code']==2003 for r in boundaries)),exceptions=dict(expected=len(exceptions),passed=sum(r['passed'] for r in exceptions)))
    report=dict(status='passed' if summary['main']['passed']==120 and summary['boundaries']['passed']==16 and summary['exceptions']['passed']==11 else 'failed',scope='C FK and IK versus explicit DH RTB; main 120, boundary 16, exception 11 executed',model_sha256=digest,fixture_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (main_path,special_path)},library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),generator_seed=fixture['random_seed'],seed_for_ik_initial_values=SEED,seed_rule='Python random.Random(seed); each witness joint plus uniform(-0.05,0.05) rad clamped to model limits; passed as explicit initial guess, not as hidden answer',python=platform.python_version(),numpy=np.__version__,roboticstoolbox=rtb.__version__,quantile='linear h=(n-1)*p',timeout_budget_s=.2,ik_execution_status='run',solver_success_rate=summary['main']['solver_success_rate'],expected_rules_snapshot=expected,summary=summary,by_region=by_region,samples=samples,boundaries=boundaries,exceptions=exceptions)
    args.output.mkdir(parents=True,exist_ok=True);(args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(status=report['status'],summary=summary,failing_main=[r['target_id'] for r in samples if not r['passed']],failing_boundaries=[r['target_id'] for r in boundaries if not r['passed']],failing_exceptions=[r['target_id'] for r in exceptions if not r['passed']]),indent=2))
    return 0 if report['status']=='passed' else 1
if __name__=='__main__':raise SystemExit(main())
