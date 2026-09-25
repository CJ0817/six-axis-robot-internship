"""Reachable/unreachable/near-limit C IK tests with explicit-parameter RTB FK."""
import argparse
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import Forward,FKModel,IKOptionsV1,IKResultV1,D6,D16,pack_model
from verify_c_ik import pose_error,stats

def main():
    p=argparse.ArgumentParser();p.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so');p.add_argument('--output',type=Path,default=ROOT/'results/ik-target-groups');a=p.parse_args()
    profile=json.loads((ROOT/'models/ur5/kinematics.json').read_text());model=pack_model(profile)
    robot=rtb.DHRobot([rtb.RevoluteDH(a=profile['a_m'][i],d=profile['d_m'][i],alpha=profile['alpha_rad'][i],offset=profile['theta_offset_rad'][i]) for i in range(6)],base=SE3(np.array(profile['T_base_dh0'])),tool=SE3(np.array(profile['T_dh6_flange']) @ np.array(profile['T_flange_tool'])))
    fk=Forward(a.library);fn=fk.lib.robot_inverse_v1;fn.restype=C.c_int
    fn.argtypes=[C.POINTER(FKModel),C.POINTER(C.c_double),C.c_size_t,C.POINTER(C.c_double),C.c_size_t,C.POINTER(IKOptionsV1),C.POINTER(IKResultV1)]
    cases=json.loads((ROOT/'tests/ik/reachability_cases.json').read_text())['cases'];rows=[]
    for case in cases:
        m=FKModel.from_buffer_copy(model)
        if 'limit_center_rad' in case:
            for i in range(6):m.q_min_rad[i]=case['limit_center_rad'][i]-case['limit_half_width_rad'];m.q_max_rad[i]=case['limit_center_rad'][i]+case['limit_half_width_rad']
        T=np.array(case['target_T_base_tool']) if 'target_T_base_tool' in case else robot.fkine(np.array(case['q_witness_rad'])*profile['joint_sign']).A
        out=IKResultV1();C.memset(C.byref(out),0x5A,C.sizeof(out));before=bytes(out)
        options=IKOptionsV1(1,1,1e-5,1e-4,0,.2,0,0,0,0,0)
        target_array=D16(*T.ravel());seed_array=D6(*case['q_seed_rad']);start=time.perf_counter()
        code=fn(C.byref(m),target_array,16,seed_array,6,C.byref(options),C.byref(out));wait=time.perf_counter()-start
        row=dict(id=case['id'],group=case['group'],input=case,target_T_base_tool=T.tolist(),expected_code=case['expected_code'],code=code,passed=code==case['expected_code'],solution_count=out.solution_count if code==0 else None,candidates=[],python_to_c_wait_s=wait,c_interface_s=out.elapsed_s if code==0 else None,error=None)
        try:
            assert row['passed'],'Unexpected error code'
            if code:assert bytes(out)==before,'Failure changed output';row['failure_output_unchanged']=True
            else:
                assert 1<=out.solution_count<=8
                for k in range(out.solution_count):
                    q=list(out.candidates_rad[k]);B=D16();fc=fk.fn(C.byref(m),D6(*q),6,B)
                    pe,re=pose_error(list(B),T.ravel()) if fc==0 else (None,None)
                    RT=robot.fkine(np.array(q)*profile['joint_sign']).A
                    rp,rr=pose_error(RT.ravel(),T.ravel())
                    margin=min(min(q[i]-m.q_min_rad[i],m.q_max_rad[i]-q[i]) for i in range(6))
                    valid=fc==0 and margin>=0 and all(math.isfinite(x) for x in [pe,re,rp,rr]) and pe<=1e-5 and re<=1e-4 and rp<=1e-5 and rr<=1e-4
                    row['candidates'].append(dict(target_id=case['id'],candidate_index=k,q_rad=q,position_error_m=pe,orientation_error_rad=re,rtb_position_error_m=rp,rtb_orientation_error_rad=rr,limit_distance_rad=margin,passed=valid))
                assert len(row['candidates'])==out.solution_count
                assert all(c['passed'] for c in row['candidates']),'Candidate FK/RTB/limit failure'
                row['candidate_record_count_matches']=True
                # Check selected q and deterministic candidate set contract too.
                qs=[r['q_rad'] for r in row['candidates']];assert qs==sorted(qs)
                assert list(out.q_rad)==qs[out.selected_index]
                for i,q in enumerate(qs):
                    for r in qs[:i]:assert max(abs(x-y) for x,y in zip(q,r))>1e-9
        except (AssertionError,ValueError) as exc:row['passed']=False;row['error']=str(exc)
        rows.append(row)
    groups={}
    for name in sorted(set(r['group'] for r in rows)):
        subset=[r for r in rows if r['group']==name];errors=[c for r in subset for c in r['candidates']]
        groups[name]=dict(expected=len(subset),passed=sum(r['passed'] for r in subset),failed=sum(not r['passed'] for r in subset),expected_outcome_rate=sum(r['passed'] for r in subset)/len(subset),returned_solution_count=sum(r['solution_count'] or 0 for r in subset),position_error_m=stats([c['position_error_m'] for c in errors if c['position_error_m'] is not None]),orientation_error_rad=stats([c['orientation_error_rad'] for c in errors if c['orientation_error_rad'] is not None]))
    report=dict(generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),inverse_source_sha256=hashlib.sha256((ROOT/'src/kinematics/inverse.c').read_bytes()).hexdigest(),status='passed' if all(r['passed'] for r in rows) else 'failed',groups=groups,samples=rows,python=platform.python_version(),numpy=np.__version__,roboticstoolbox=rtb.__version__,library_sha256=hashlib.sha256(a.library.read_bytes()).hexdigest(),inputs_sha256=hashlib.sha256((ROOT/'tests/ik/reachability_cases.json').read_bytes()).hexdigest(),model_sha256=hashlib.sha256((ROOT/'models/ur5/kinematics.json').read_bytes()).hexdigest(),statistics='linear p95; null for no FK output; expected_outcome_rate includes expected rejections, not IK solve success')
    a.output.mkdir(parents=True,exist_ok=True);(a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(dict(status=report['status'],groups=groups,failures=[r['id'] for r in rows if not r['passed']]),indent=2));return 0 if report['status']=='passed' else 1
if __name__=='__main__':raise SystemExit(main())
