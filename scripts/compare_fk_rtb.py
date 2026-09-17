"""Named-angle C/RTB comparison and per-link localization of nominal model differences."""
import argparse
import copy
import csv
import ctypes as C
from datetime import datetime,timezone
import hashlib
from importlib.metadata import version
import inspect
import json
import os
from pathlib import Path
import platform
import sys
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
os.environ['MPLBACKEND']='Agg'
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import Forward,D16

def require(ok,message):
    if not ok:raise ValueError(message)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so')
    parser.add_argument('--output',type=Path,default=ROOT/'results/fk-rtb-comparison')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    report={'status':'failed','scope':'named_pose_FK_accuracy_and_difference_localization',
            'utc':datetime.now(timezone.utc).isoformat(),'python':platform.python_version(),
            'platform':platform.platform(),'statistics_version':1,'timing_status':'not_measured',
            'thresholds':{'position_m':1e-9,'orientation_rad':1e-9,'matrix_element':1e-9}}
    try:
        import numpy as np
        import roboticstoolbox as rtb
        from spatialmath import SE3
        require(version('roboticstoolbox-python')=='1.1.1','Requires locked RTB 1.1.1')
        model_path=ROOT/'models/ur5/kinematics.json';cases_path=ROOT/'tests/kinematics/fk_known_cases.json'
        model=json.loads(model_path.read_text());fixture=json.loads(cases_path.read_text())
        require(fixture['units']=='rad' and fixture['joint_order']==model['joint_names'],'Case conventions mismatch')
        require(len({c['name'] for c in fixture['cases']})==len(fixture['cases']),'Duplicate case name')
        forward=Forward(args.library)
        fn=forward.lib.robot_dh_transform
        fn.argtypes=[C.c_double]*4+[C.POINTER(C.c_double)];fn.restype=C.c_int
        reference=rtb.DHRobot([rtb.RevoluteDH(a=a,d=d,alpha=alpha,offset=offset,flip=sign==-1)
            for a,d,alpha,offset,sign in zip(model['a_m'],model['d_m'],model['alpha_rad'],model['theta_offset_rad'],model['joint_sign'])],name='project_UR5_CB')
        reference.base=SE3(np.array(model['T_base_dh0'],float));reference.tool=SE3(np.array(model['T_dh6_flange'],float)@model['T_flange_tool'])
        builtin=rtb.models.DH.UR5();changed=copy.deepcopy(builtin)
        # Diagnostic one-factor intervention only; never mutate installed RTB or persisted sources.
        changed.links[0].d=model['d_m'][0]
        differences=[]
        for i,(a,b) in enumerate(zip(reference.links,builtin.links)):
            for field in ('a','d','alpha','offset','isflip','mdh'):
                x,y=getattr(a,field),getattr(b,field)
                if x!=y:differences.append({'joint':i+1,'field':field,'project':float(x),'builtin':float(y)})
        require(len(differences)==1 and differences[0]['joint']==1 and differences[0]['field']=='d','Expected only d1 discrepancy; review changed model')
        require(np.max(np.abs(reference.base.A-builtin.base.A))<1e-12 and np.max(np.abs(reference.tool.A-builtin.tool.A))<1e-12,'Base/tool mismatch must be resolved before geometric comparison')
        def measure(actual,target):
            require(actual.shape==(4,4) and target.shape==(4,4) and np.isfinite(actual).all() and np.isfinite(target).all(),'Invalid matrix')
            for t in (actual,target):
                require(np.max(np.abs(t[3]-[0,0,0,1]))<1e-9 and np.linalg.norm(t[:3,:3].T@t[:3,:3]-np.eye(3))<1e-9 and abs(np.linalg.det(t[:3,:3])-1)<1e-9,'Invalid rigid pose')
            relative=target[:3,:3].T@actual[:3,:3]
            sine=.5*np.linalg.norm([relative[2,1]-relative[1,2],relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]])
            angle=float(np.arctan2(sine,np.clip((np.trace(relative)-1)/2,-1,1)))
            pos=float(np.linalg.norm(actual[:3,3]-target[:3,3]));mat=float(np.max(np.abs(actual-target)))
            return {'position_error_m':pos,'orientation_error_rad':angle,'matrix_max_error':mat,
                    'delta_position_m':(actual[:3,3]-target[:3,3]).tolist(),'passes':pos<=1e-9 and angle<=1e-9 and mat<=1e-9}
        records=[];expected_delta=np.array([0,0,builtin.links[0].d-reference.links[0].d])
        for case in fixture['cases']:
            q=case['q_rad'];res=forward(model,q);require(res['code']==0 and res['data'] is not None,'C failed '+case['name'])
            c_pose=np.array(res['data']);rt_pose=reference.fkine(q).A;raw=builtin.fkine(q).A;corrected=changed.fkine(q).A
            links=[];ct=np.array(model['T_base_dh0'],float);bt=builtin.base.A.copy()
            for i in range(6):
                out=D16();code=fn(model['a_m'][i],model['alpha_rad'][i],model['d_m'][i],model['joint_sign'][i]*q[i]+model['theta_offset_rad'][i],out)
                require(code==0,'C single link failed');ct=ct@np.array(out).reshape(4,4);bt=bt@builtin.links[i].A(q[i]).A
                error=measure(bt,ct)
                require(np.linalg.norm(np.array(error['delta_position_m'])-expected_delta)<1e-12,'Difference is not constant d1 offset')
                links.append({'joint':i+1,**error})
            record={'sample_id':case['name'],'q_rad':q,'c_return_code':res['code'],'T_c':c_pose.tolist(),'T_rtb_project':rt_pose.tolist(),
                    'T_rtb_builtin':raw.tolist(),'T_builtin_d1_only_corrected':corrected.tolist(),
                    'c_vs_project':measure(c_pose,rt_pose),'builtin_vs_c':measure(raw,c_pose),
                    'd1_only_corrected_vs_c':measure(corrected,c_pose),'link_localization':links}
            records.append(record)
        def stats(values):
            a=np.array(values);return {'valid_count':len(a),'invalid_count':0,'mean':float(a.mean()),'rmse':float(np.sqrt(np.mean(a*a))),'p95':float(np.quantile(a,.95,method='linear')),'max':float(a.max())}
        def summarize(key):
            return {'expected_count':len(fixture['cases']),'success_count':sum(c[key]['passes'] for c in records),
                    'failure_count':sum(not c[key]['passes'] for c in records),'success_rate':sum(c[key]['passes'] for c in records)/len(fixture['cases']),
                    'position_error_m':stats([c[key]['position_error_m'] for c in records]),
                    'orientation_error_rad':stats([c[key]['orientation_error_rad'] for c in records]),
                    'matrix_max_error':max(c[key]['matrix_max_error'] for c in records)}
        report.update(parameter_differences=differences,base_tool_match=True,expected_builtin_minus_project_translation_m=expected_delta.tolist(),
            comparison={k:summarize(k) for k in ('c_vs_project','builtin_vs_c','d1_only_corrected_vs_c')},
            source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [model_path,cases_path,Path(__file__),ROOT/'src/kinematics/forward.c',ROOT/'include/robot_kinematics.h',ROOT/'src/adapters/c_kinematics.py']},
            library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),rtb_version=version('roboticstoolbox-python'),numpy_version=np.__version__,
            rtb_builtin_source_sha256=hashlib.sha256(Path(inspect.getfile(type(builtin))).read_bytes()).hexdigest(),
            quantile_method='linear h=(n-1)*p',samples_file='samples.json',
            diagnosis='First difference at DH link 1: built-in d1 larger by 0.0003 m; base-frame +Z translation persists through all six links. Correcting only d1 in an in-memory copy removes the mismatch. Original builtin mismatch is recorded as failed, not ignored.')
        (args.output/'samples.json').write_text(json.dumps(records,indent=2,allow_nan=False)+'\n')
        with (args.output/'comparison.csv').open('w',newline='') as file:
            writer=csv.writer(file);writer.writerow(['case','q_rad','C_x_m','C_y_m','C_z_m','project_pos_error_m','project_angle_error_rad','builtin_pos_error_m','builtin_delta_z_m','corrected_pos_error_m'])
            for v in records:writer.writerow([v['sample_id'],json.dumps(v['q_rad']),*[v['T_c'][i][3] for i in range(3)],v['c_vs_project']['position_error_m'],v['c_vs_project']['orientation_error_rad'],v['builtin_vs_c']['position_error_m'],v['builtin_vs_c']['delta_position_m'][2],v['d1_only_corrected_vs_c']['position_error_m']])
        require(all(v['c_vs_project']['passes'] and v['d1_only_corrected_vs_c']['passes'] for v in records),'C/reference comparison failed')
        report['status']='passed'
    except Exception as exc:report['error']=f'{type(exc).__name__}: {exc}'
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2,allow_nan=False));return 0 if report['status']=='passed' else 1

if __name__=='__main__':sys.exit(main())
