"""Freeze reachable IK targets without running IK or selecting successful solves."""
import argparse
from datetime import datetime,timezone
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import sys
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
os.environ['MPLBACKEND']='Agg'
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import Forward

def require(ok,message):
    if not ok:raise ValueError(message)

def generate(model,forward):
    import numpy as np
    import roboticstoolbox as rtb
    from spatialmath import SE3
    require(version('roboticstoolbox-python')=='1.1.1','Use locked RTB 1.1.1')
    robot=rtb.DHRobot([rtb.RevoluteDH(a=a,d=d,alpha=al,offset=off,flip=sign==-1)
        for a,d,al,off,sign in zip(model['a_m'],model['d_m'],model['alpha_rad'],model['theta_offset_rad'],model['joint_sign'])],name='project_UR5_CB')
    robot.base=SE3(np.asarray(model['T_base_dh0'],float))
    robot.tool=SE3(np.asarray(model['T_dh6_flange'],float)@model['T_flange_tool'])
    low=np.asarray(model['q_min_rad']);high=np.asarray(model['q_max_rad']);margin=np.minimum(.05,.1*(high-low))
    lo,hi=low+margin,high-margin;rng=np.random.default_rng(20260917)
    groups={'normal':[],'wide_initial':[],'near_singular':[]}
    errors=[]
    def sample(name,q,seed):
        q=np.asarray(q);seed=np.asarray(seed)
        require(np.all(q>=lo) and np.all(q<=hi) and np.all(seed>=low) and np.all(seed<=high),'Joint limit violation')
        target=robot.fkine(q).A
        result=forward(model,q.tolist());require(result['code']==0 and result['data'] is not None,'C FK error')
        actual=np.asarray(result['data']);err=float(np.max(np.abs(actual-target)))
        require(np.isfinite(target).all() and err<=1e-9,'C/RTB target mismatch')
        require(np.max(np.abs(target[3]-[0,0,0,1]))<=1e-9 and np.linalg.norm(target[:3,:3].T@target[:3,:3]-np.eye(3))<=1e-9 and abs(np.linalg.det(target[:3,:3])-1)<=1e-9,'Invalid target pose')
        # Scale translational Jacobian rows by L=1 m before dimensionless SVD.
        j=robot.jacob0(q).copy();j[:3]/=1.0
        sigma=float(np.linalg.svd(j,compute_uv=False)[-1]);errors.append(err)
        return {'sample_id':name,'target_T_base_tool':target.tolist(),'q_witness_rad':q.tolist(),'q_seed_rad':seed.tolist(),
                'reachability':'witness_FK_cross_checked_not_IK_solved','scaled_jacobian_sigma_min':sigma}
    for i in range(100):
        q=rng.uniform(lo,hi);seed=q+rng.uniform(-.05,.05,6)
        groups['normal'].append(sample(f'normal_{i:03d}',q,seed))
    for i in range(20):
        q=np.array(groups['normal'][i]['q_witness_rad'])
        for attempt in range(1000):
            seed=rng.uniform(lo,hi)
            # Compare angular distance modulo 2pi to avoid counting equivalent wraps as wide.
            distance=float(np.max(np.abs((seed-q+np.pi)%(2*np.pi)-np.pi)))
            if distance>=.5:break
        else:raise ValueError('Cannot obtain wide seed')
        record=sample(f'wide_{i:03d}',q,seed);record['paired_normal_id']=groups['normal'][i]['sample_id'];record['max_wrapped_seed_distance_rad']=distance
        groups['wide_initial'].append(record)
    for i in range(20):
        q=rng.uniform(lo,hi);q[4]=(-1 if i%2 else 1)*10**(-7+(i//2)/9*2)
        seed=q+rng.uniform(-.05,.05,6)
        record=sample(f'near_singular_{i:03d}',q,seed)
        require(record['scaled_jacobian_sigma_min']<=1e-5,'Wrist-near-singular criterion not met; no silent resampling')
        groups['near_singular'].append(record)
    anchors=[]
    for name,q in [('zero',[0]*6),('demo_home',[0,-np.pi/2,0,-np.pi/2,0,0]),('asymmetric',[.2,-.6,.8,-.5,.4,-.2])]:
        anchors.append(sample('anchor_'+name,np.array(q),np.array(q)))
    return {'schema_version':1,'purpose':'IK_input_fixture_not_solver_results','seed':20260917,'model_id':model['model_id'],
            'base_frame':'base','tool_frame':'tool0','length_unit':'m','angle_unit':'rad','joint_order':model['joint_names'],
            'generation':'Explicit project-parameter RTB FK, C FK cross-check; no IK and no solve-based selection',
            'group_counts':{k:len(v) for k,v in groups.items()},'groups':groups,'anchors_not_in_group_denominators':anchors,
            'future_acceptance':{'position_tol_m':1e-5,'orientation_tol_rad':1e-4,'timeout_s':.2,'normal_required_success_rate':1.0,
                                 'wide_and_near_singular_success_threshold':'report separately; no false success'},
            'singularity_definition':{'family':'wrist q5 near 0; not all singularity families','characteristic_length_m':1.0,'sigma_min_max':1e-5},
            'ik_execution_status':'not_run'},max(errors)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so')
    parser.add_argument('--output',type=Path,default=ROOT/'results/ik-preparation')
    parser.add_argument('--write-fixture',action='store_true',help='Explicitly regenerate frozen fixture; normal use checks only')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    report={'status':'failed','scope':'IK_target_preparation_only','ik_execution_status':'not_run','utc':datetime.now(timezone.utc).isoformat()}
    try:
        model_path=ROOT/'models/ur5/kinematics.json';profile=json.loads(model_path.read_text())
        targets,max_error=generate(profile,Forward(args.library))
        targets['model_sha256']=hashlib.sha256(model_path.read_bytes()).hexdigest()
        fixture=ROOT/'tests/ik/targets.json'
        text=json.dumps(targets,indent=2,allow_nan=False)+'\n'
        if args.write_fixture:fixture.write_text(text)
        require(fixture.exists(),'Frozen fixture missing; explicitly generate with --write-fixture')
        # Float tolerance allows harmless libm differences across Linux runners, not model changes.
        saved=json.loads(fixture.read_text())
        def compare(a,b):
            if isinstance(a,dict):return isinstance(b,dict) and a.keys()==b.keys() and all(compare(a[k],b[k]) for k in a)
            if isinstance(a,list):return isinstance(b,list) and len(a)==len(b) and all(compare(x,y) for x,y in zip(a,b))
            if type(a) is float:return type(b) in (int,float) and abs(a-b)<=1e-12
            return type(a)==type(b) and a==b
        require(compare(saved,targets),'Frozen fixture differs from deterministic generation')
        report.update(status='passed',group_counts=targets['group_counts'],anchor_count=3,
            c_rtb_max_matrix_error=max_error,fixture_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),
            model_sha256=targets['model_sha256'],script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),rtb_version=version('roboticstoolbox-python'),
            numpy_version=version('numpy'),python=sys.version.split()[0],solver_success_rate=None)
    except Exception as exc:report['error']=f'{type(exc).__name__}: {exc}'
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2));return 0 if report['status']=='passed' else 1
if __name__=='__main__':sys.exit(main())
