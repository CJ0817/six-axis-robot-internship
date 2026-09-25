"""C ABI IK regression: frozen targets, independent target matrices, error boundaries."""
import argparse
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import FKModel,IKOptionsV1,IKResultV1,D6,D16,pack_model,Forward

def stats(v):
    if not v:return dict(valid_count=0,mean=None,rmse=None,p95=None,maximum=None)
    s=sorted(v);h=(len(s)-1)*.95;i=int(h)
    return dict(valid_count=len(v),mean=sum(v)/len(v),rmse=math.sqrt(sum(x*x for x in v)/len(v)),p95=s[i]+(h-i)*(s[min(i+1,len(s)-1)]-s[i]),maximum=max(v))

def pose_error(a,b):
    p=math.sqrt(sum((a[i]-b[i])**2 for i in [3,7,11]))
    r=[[sum(a[4*k+i]*b[4*k+j] for k in range(3)) for j in range(3)] for i in range(3)]
    s=math.sqrt(sum(x*x for x in [r[2][1]-r[1][2],r[0][2]-r[2][0],r[1][0]-r[0][1]]))/2
    return p,math.atan2(s,max(-1,min(1,(sum(r[i][i] for i in range(3))-1)/2)))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so');parser.add_argument('--output',type=Path,default=ROOT/'results/c-ik');args=parser.parse_args()
    start=time.monotonic();profile=json.loads((ROOT/'models/ur5/kinematics.json').read_text());model=pack_model(profile)
    fk=Forward(args.library);fn=fk.lib.robot_inverse_v1;fn.restype=C.c_int
    fn.argtypes=[C.POINTER(FKModel),C.POINTER(C.c_double),C.c_size_t,C.POINTER(C.c_double),C.c_size_t,C.POINTER(IKOptionsV1),C.POINTER(IKResultV1)]
    def options(policy=1):return IKOptionsV1(1,policy,1e-5,1e-4,0,.2,0,0,0,0,0)
    def call(T,seed,m=model,o=None,pc=16,sc=6,null=None):
        opt=options() if o is None else o;out=IKResultV1();C.memset(C.byref(out),0x5A,C.sizeof(out));before=bytes(out)
        target=None if T is None else D16(*T);q=None if seed is None else D6(*seed)
        ptrs=[None if m is None else C.byref(m),target,pc,q,sc,None if null=='options' else C.byref(opt),None if null=='out' else C.byref(out)]
        tick=time.perf_counter();code=fn(*ptrs);wall=time.perf_counter()-tick
        if code:assert bytes(out)==before,'Failure changed output'
        return code,out,wall
    def target(q,m=model):
        T=D16();assert fk.fn(C.byref(m),D6(*q),6,T)==0;return list(T)
    def validate(T,seed,out,m=model,o=None,records=None,target_id=None):
        records=[] if records is None else records
        o=options() if o is None else o;n=out.solution_count;assert 1<=n<=8 and out.selected_index<n
        cs=[list(out.candidates_rad[k]) for k in range(n)];assert cs==sorted(cs)
        assert list(out.q_rad)==cs[out.selected_index]
        dist=[math.dist(c,seed) for c in cs];assert out.selected_index==min(range(n),key=lambda i:(dist[i],cs[i]))
        pe=[];re=[]
        for k,q in enumerate(cs):
            limits_ok=all(math.isfinite(v) and m.q_min_rad[i]+o.joint_margin_rad<=v<=m.q_max_rad[i]-o.joint_margin_rad for i,v in enumerate(q))
            back=D16();fk_code=fk.fn(C.byref(m),D6(*q),6,back)
            p,r=pose_error(list(back),T) if fk_code==0 else (None,None)
            valid=limits_ok and fk_code==0 and all(v is not None and math.isfinite(v) for v in (p,r)) and p<=o.position_tol_m and r<=o.orientation_tol_rad
            records.append(dict(target_id=target_id,branch_policy="all" if o.branch_policy==1 else "nearest_seed",candidate_index=k,q_rad=[v if math.isfinite(v) else None for v in q],position_error_m=p if p is not None and math.isfinite(p) else None,orientation_error_rad=r if r is not None and math.isfinite(r) else None,passed=valid,fk_code=fk_code,within_limits=limits_ok))
            pe.append(p);re.append(r)
        assert len(records)==n,'Candidate error record count differs from solution_count'
        assert all(item['passed'] for item in records),'Candidate FK/limits/residual check failed'
        for k,q in enumerate(cs):
            assert out.branch_ids[k]==k
            for prev in cs[:k]:assert max(abs(a-b) for a,b in zip(prev,q))>1e-9
        for k in range(n,8):assert list(out.candidates_rad[k])==[0]*6 and out.branch_ids[k]==2**32-1
        assert out.iterations==0 and out.reserved==0 and 0<=out.elapsed_s<o.timeout_s
        assert abs(out.position_error_m-pe[out.selected_index])<1e-12 and abs(out.orientation_error_rad-re[out.selected_index])<1e-12
        return max(pe),max(re)
    fixture=json.loads((ROOT/'tests/ik/targets.json').read_text());groups={};rows=[]
    for name,cases in fixture['groups'].items():
        pe=[];re=[];elapsed=[];walls=[];passed=0
        for case in cases:
            T=sum(case['target_T_base_tool'],[]);seed=case['q_seed_rad'];code,out,wall=call(T,seed);ok=False;error=None;records=[];nearest_records=[];nearest_count=None
            try:
                assert code==0,f'code {code}';p,r=validate(T,seed,out,records=records,target_id=case['sample_id'])
                nc,no,_=call(T,seed,o=options(0));assert nc==0
                nearest_count=no.solution_count
                validate(T,seed,no,o=options(0),records=nearest_records,target_id=case['sample_id']);assert list(no.q_rad)==list(out.q_rad) and no.solution_count==1
                pe.append(p);re.append(r);elapsed.append(out.elapsed_s);ok=True
            except AssertionError as exc:error=str(exc)
            passed+=ok;walls.append(wall);rows.append(dict(id=case['sample_id'],group=name,code=code,passed=ok,error=error,count=out.solution_count if code==0 else None,solution_count=out.solution_count if code==0 else None,candidates=records,candidate_record_count_matches=(len(records)==out.solution_count) if code==0 else None,nearest_seed=dict(solution_count=nearest_count,candidates=nearest_records),c_elapsed_s=out.elapsed_s if code==0 else None,python_to_c_s=wall))
        groups[name]=dict(expected=len(cases),passed=passed,failed=len(cases)-passed,success_rate=passed/len(cases),position_m=stats(pe),orientation_rad=stats(re),c_interface_s=stats(elapsed),python_to_c_s=stats(walls))
    special=[]
    def check(name,T,seed,expected=0,**kwargs):
        code,out,_=call(T,seed,**kwargs);ok=code==expected;detail=None;records=[]
        if code==0:
            try:validate(T,seed,out,m=kwargs.get('m',model),o=kwargs.get('o'),records=records,target_id='special:'+name)
            except AssertionError as exc:ok=False;detail=str(exc)
        special.append(dict(name=name,expected=expected,actual=code,passed=ok,detail=detail,solution_count=out.solution_count if code==0 else None,candidates=records,candidate_record_count_matches=(len(records)==out.solution_count) if code==0 else None));return out
    q=[.2,-.6,.8,-.5,.4,-.2];T=target(q)
    for null in ['model','target','seed','options','out']:
        check('null_'+null,None if null=='target' else T,None if null=='seed' else q,1004 if null=='model' else 1001,**({'m':None} if null=='model' else {'null':null}))
    for length in [0,15,17]:check('pose_length_'+str(length),T,q,1001,pc=length)
    for length in [0,5,7]:check('seed_length_'+str(length),T,q,1001,sc=length)
    for bad in [float('nan'),float('inf')]:
        b=T.copy();b[0]=bad;check('target_nonfinite_'+str(bad),b,q,1001)
        b=q.copy();b[0]=bad;check('seed_nonfinite_'+str(bad),T,b,1001)
    for idx,value in [(15,0),(0,2)]:
        b=T.copy();b[idx]=value;check('invalid_rigid_'+str(idx),b,q,1001)
    b=T.copy();b[3]=1e200;check('unreachable_large_finite',b,q,2001)
    o=options();o.method=2;check('invalid_dls_options',T,q,1001,o=o)
    o.max_iterations=100;o.line_search_max_steps=10;o.characteristic_length_m=1;o.damping=.01;o.max_step_rad=.1;check('dls_not_implemented',T,q,1008,o=o)
    b=[0,math.pi/2,0,-math.pi/2,.8,.2];check('shoulder_tangent',target(b),b)
    b=T.copy();b[3]=10;check('unreachable_far',b,q,2001)
    b=[1.,0,0,0,0,1,0,0,0,0,1,.0823,0,0,0,1];check('unreachable_shoulder',b,q,2001)
    for field,value in [('timeout_s',0),('position_tol_m',-1),('orientation_tol_rad',4),('joint_margin_rad',4),('damping',1),('timeout_s',float('nan'))]:
        o=options();setattr(o,field,value);check('invalid_'+field+str(value),T,q,1001,o=o)
    o=options();o.timeout_s=1e-15;check('budget_exhausted',T,q,2002,o=o)
    o=options();o.method=99;check('unsupported_method',T,q,1008,o=o)
    o=options();o.branch_policy=99;check('unsupported_policy',T,q,1008,o=o)
    bad=FKModel.from_buffer_copy(model);bad.d_m[0]=.089459;check('wrong_robot_geometry',T,q,1008,m=bad)
    bad=FKModel.from_buffer_copy(model);bad.joint_sign[0]=0;check('invalid_model_sign',T,q,1001,m=bad)
    b=q.copy();b[0]=7;check('seed_limit',T,b,1005)
    narrow=FKModel.from_buffer_copy(model)
    for i in range(6):narrow.q_min_rad[i]=-.001;narrow.q_max_rad[i]=.001
    check('all_branches_outside_limits',T,[0]*6,1005,m=narrow)
    for t5 in [0,math.pi]:
        b=q.copy();b[4]=t5;BT=target(b)
        check('singular_all_'+str(t5),BT,b,2003)
        check('singular_matching_seed_'+str(t5),BT,b,o=options(0))
        seed=b.copy();seed[0]+=.1;check('singular_unresolved_'+str(t5),BT,seed,2003,o=options(0))
    for theta in [0,math.pi,-math.pi,1e-8]:
        b=q.copy();b[2]=theta;check('elbow_boundary_'+str(theta),target(b),b)
    for theta in [-1e-13,1e-13]:
        b=q.copy();b[4]=theta;check('wrist_numeric_boundary_'+str(theta),target(b),b,2003)
    for theta in [-2*math.pi,2*math.pi]:
        b=q.copy();b[0]=theta;check('joint_endpoint_'+str(theta),target(b),b)
    b=q.copy();b[0]=-.2;seed=b.copy();seed[0]=6.1
    out=check('two_pi_nearest',target(b),seed,o=options(0));special[-1]['passed'] &= abs(out.q_rad[0]-(-.2+2*math.pi))<1e-9
    transformed=FKModel.from_buffer_copy(model)
    transformed.joint_sign[1]=-1;transformed.theta_offset_rad[2]=.15
    transformed.T_base_dh0=D16(0,-1,0,.3,1,0,0,-.2,0,0,1,.1,0,0,0,1)
    transformed.T_flange_tool[3]=.12;transformed.T_dh6_flange[7]=-.08
    check('fixed_frames_sign_offset',target(q,transformed),q,m=transformed)
    report=dict(schema_version=2,candidate_error_source='Independent test-layer C robot_forward per retained candidate; not copied from selected-solution ABI fields',status='passed' if all(g['failed']==0 for g in groups.values()) and all(s['passed'] for s in special) else 'failed',scope='C_analytic_regular_branches_and_explicit_singular_policy',groups=groups,special=special,samples=rows,python=platform.python_version(),platform=platform.platform(),library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),source_sha256=hashlib.sha256((ROOT/'src/kinematics/inverse.c').read_bytes()).hexdigest(),fixture_sha256=hashlib.sha256((ROOT/'tests/ik/targets.json').read_bytes()).hexdigest(),script_work_wall_s=time.monotonic()-start,timeout_policy='C cooperative budget .2s; external runner hard timeout 120s',quantile='linear h=(n-1)*p')
    report['candidate_record_totals']=dict(all=sum(len(row['candidates']) for row in rows),nearest_seed=sum(len(row['nearest_seed']['candidates']) for row in rows),special=sum(len(row['candidates']) for row in special))
    args.output.mkdir(parents=True,exist_ok=True);(args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(status=report['status'],groups=groups,special_count=len(special),special_failures=[s for s in special if not s['passed']]),indent=2))
    return 0 if report['status']=='passed' else 1
if __name__=='__main__':raise SystemExit(main())
