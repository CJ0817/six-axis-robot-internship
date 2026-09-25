"""Reproducible trajectory-sequence tests of state-aware IK selection."""
import argparse
import copy
from dataclasses import replace,asdict
import hashlib
import json
import math
from pathlib import Path
import platform
import numpy as np
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.ik_selection import StatefulIKSelector,SelectionConfig,pack_model

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so');parser.add_argument('--output',type=Path,default=ROOT/'results/ik-selection');args=parser.parse_args()
    profile=json.loads((ROOT/'models/ur5/kinematics.json').read_text());model=pack_model(profile);selector=StatefulIKSelector(args.library);cases=[]
    def pose(q):return selector._fk(model,q).tolist()
    def run(name,sel,T,state,expected=0):
        before=copy.deepcopy(state);start=time.perf_counter();r=sel.select(profile,T,state);wall=time.perf_counter()-start
        ok=r['code']==expected and state==before
        if r['code']==0:
            ok &= r['next_state']['step_index']==state['step_index']+1 and r['next_state']['q_rad']==r['data']['q_rad']
            if r['data']['mode']=='singular_hold':ok &= r['data']['q_rad']==state['q_rad']
            else:
                chosen=next(d for d in r['details']['candidates'] if d['candidate_index']==r['data']['selected_candidate_index'])
                allowed=[d for d in r['details']['candidates'] if d['eligible']]
                ok &= chosen==min(allowed,key=lambda d:(d['total_cost'],tuple(d['q_rad'])))
                ok &= all(abs(v)<=cap+1e-12 for v,cap in zip(chosen['delta_rad'],chosen['step_cap_rad']))
                ok &= chosen['fk_passed'] and chosen['sigma_min']>sel.config.sigma_hard
        else:ok &= r['data'] is None and r['next_state'] is None
        cases.append(dict(name=name,passed=bool(ok),expected_code=expected,actual_code=r['code'],previous_state=before,config=asdict(sel.config),target_T_base_tool=T,actual_wait_s=wall,result=r));return r
    base=[.3,-1,.9,-.7,.8,.2]
    state=dict(q_rad=base.copy(),step_index=0)
    for i in range(1,41):
        q=[base[j]+i*v for j,v in enumerate([.004,-.002,.001,.001,.001,-.002])]
        r=run('regular_%02d'%i,selector,pose(q),state)
        if r['code']==0:state=r['next_state']
    q=base.copy();q[0]=3.12;state=dict(q_rad=q,step_index=0)
    for i in range(1,31):
        q=base.copy();q[0]=3.12+.003*i
        r=run('cross_pi_%02d'%i,selector,pose(q),state)
        if r['code']==0:state=r['next_state']
    q=base.copy();q[4]=.05;state=dict(q_rad=q,step_index=0)
    for i in range(1,16):
        q=base.copy();q[4]=.05-.002*i
        r=run('near_singular_%02d'%i,selector,pose(q),state)
        if r['code']==0:state=r['next_state']
    # An incoming target crossing into the protected wrist region must stop.
    q=base.copy();q[4]=1e-7
    run('protected_region_reject',selector,pose(q),state,2003)
    q=base.copy();q[4]=0
    run('exact_singular_hold',selector,pose(q),dict(q_rad=q,step_index=3))
    before=base.copy();before[4]=1e-4;after=base.copy();after[4]=-1e-4
    run('reject_wrist_crossing_between_valid_endpoints',selector,pose(after),dict(q_rad=before,step_index=0),2003)
    before=base.copy();before[4]=0;after=base.copy();after[4]=.003
    run('small_step_exit_singularity',selector,pose(after),dict(q_rad=before,step_index=0))
    r=run('large_jump_reject',selector,pose([1.3]+base[1:]),dict(q_rad=base,step_index=0),2002)
    q=base.copy();q[0]=2*math.pi-.02;state=dict(q_rad=q,step_index=0);q=q.copy();q[0]+=.002
    run('near_limit_no_jump',selector,pose(q),state)
    T=pose(base);T[0][3]=10;run('unreachable',selector,T,dict(q_rad=base,step_index=0),2001)
    run('invalid_weights',StatefulIKSelector(args.library,replace(selector.config,travel_weight=-1)),pose(base),dict(q_rad=base,step_index=0),1001)
    run('selection_timeout',StatefulIKSelector(args.library,replace(selector.config,timeout_s=1e-15)),pose(base),dict(q_rad=base,step_index=0),2002)
    run('invalid_state_index',selector,pose(base),dict(q_rad=base,step_index=-1),1001)
    q=base.copy();q[0]=7;run('previous_state_outside_limit',selector,pose(base),dict(q_rad=q,step_index=0),1005)
    fixture=json.loads((ROOT/'tests/ik/targets.json').read_text())['groups']['normal'][0]
    selected=[]
    for name,weights in [('travel_only',(1,0,0)),('limit_only',(0,1,0)),('singular_only',(0,0,1)),('combined',(1,.05,.05))]:
        cfg=replace(selector.config,travel_weight=weights[0],limit_weight=weights[1],singular_weight=weights[2],max_step_rad=7,dt_s=3,near_step_scale=1)
        r=run('score_'+name,StatefulIKSelector(args.library,cfg),fixture['target_T_base_tool'],dict(q_rad=fixture['q_seed_rad'],step_index=0))
        if r['code']==0:selected.append(r['data']['selected_candidate_index'])
    checks=[dict(name='weights_change_branch',passed=len(set(selected))>=2,selected_indices=selected)]
    near=[row for row in cases if row['name'].startswith('near_singular_')]
    checks.append(dict(name='near_region_reduces_step_cap',passed=all(r['result']['code']==0 and r['result']['data']['mode']=='near_singular_restricted' for r in near)))
    # Numerical singularity estimate stable against halving the difference step,
    # including a one-sided derivative at a joint limit.
    half=StatefulIKSelector(args.library,replace(selector.config,difference_step_rad=5e-7))
    for name,q in [('regular',base),('near_wrist',[.3,-1,.9,-.7,1e-4,.2]),('joint_endpoint',[2*math.pi,-1,.9,-.7,.8,.2])]:
        s1=selector.sigma_min(model,q);s2=half.sigma_min(model,q)
        checks.append(dict(name='sigma_difference_step_'+name,sigma_h=s1,sigma_half_h=s2,passed=abs(s1-s2)<=1e-6))
    report=dict(status='passed' if all(r['passed'] for r in cases+checks) else 'failed',expected=len(cases),passed=sum(r['passed'] for r in cases),checks=checks,cases=cases,python=platform.python_version(),numpy=np.__version__,model_sha256=hashlib.sha256((ROOT/'models/ur5/kinematics.json').read_bytes()).hexdigest(),library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),selector_sha256=hashlib.sha256((ROOT/'src/adapters/ik_selection.py').read_bytes()).hexdigest(),scope='simulation planning; selection guards are not trajectory collision/acceleration or real-time guarantees')
    args.output.mkdir(parents=True,exist_ok=True);(args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(status=report['status'],cases=report['expected'],failed=[r['name'] for r in cases+checks if not r['passed']],checks=checks),indent=2))
    return 0 if report['status']=='passed' else 1
if __name__=='__main__':raise SystemExit(main())
