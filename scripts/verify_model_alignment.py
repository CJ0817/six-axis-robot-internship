"""Audit pinned UR5 frames against XML kinematics and an explicit RTB DH model."""
import argparse
from datetime import datetime, timezone
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import xml.etree.ElementTree as ET

for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
os.environ['MPLBACKEND']='Agg'
ROOT=Path(__file__).resolve().parents[1]

def require(ok, message):
    if not ok:
        raise ValueError(message)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'results/model-audit')
    parser.add_argument('--library',type=Path,help='Also verify C FK against the independent XML chain and RTB')
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    report={'status':'failed','scope':'nominal_model_alignment_only_not_C_algorithm_or_hardware',
            'utc':datetime.now(timezone.utc).isoformat(),'python':platform.python_version(),
            'platform':platform.platform(),'seed':20260917,'position_threshold_m':1e-9,
            'rotation_matrix_threshold':1e-9,'statistics_version':1}
    try:
        import numpy as np
        import roboticstoolbox as rtb
        from importlib.metadata import version
        from spatialmath import SE3
        require(version('roboticstoolbox-python')=='1.1.1','RTB must match baseline lock')
        p=ROOT/'models/ur5/kinematics.json'; model=json.loads(p.read_text())
        require(model['representation']=='standard_dh' and model['usage']=='simulation_only','Profile scope')
        require(model['dof']==6 and model['joint_sign']==[1]*6 and model['theta_offset_rad']==[0]*6,'Unexpected joint mapping')
        lock=json.loads((ROOT/'environment.lock.json').read_text())
        manifest=json.loads((ROOT/'models/ur5/manifest.json').read_text())
        require(model['sources']['upstream_commit']==lock['robot']['commit']==manifest['upstream_commit'],'Source commit mismatch')
        for name,digest in model['sources']['files_sha256'].items():
            require(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,'Source changed: '+name)
        for name,digest in manifest['sha256'].items():
            require(hashlib.sha256((ROOT/'models/ur5'/name).read_bytes()).hexdigest()==digest,'Asset changed: '+name)
        def transform_valid(t):
            t=np.asarray(t,dtype=float)
            require(t.shape==(4,4) and np.isfinite(t).all(),'Invalid transform')
            require(np.max(np.abs(t[3]-[0,0,0,1]))<=1e-9 and np.linalg.norm(t[:3,:3].T@t[:3,:3]-np.eye(3))<=1e-9 and abs(np.linalg.det(t[:3,:3])-1)<=1e-9,'Nonrigid transform')
            return t
        for name,value in model.items():
            if name.startswith('T_'):transform_valid(value)
        def rot(axis,t):
            # Rodrigues rotation for the arbitrary URDF joint axis.
            axis=np.asarray(axis,float); axis=axis/np.linalg.norm(axis)
            x,y,z=axis; K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            T=np.eye(4);T[:3,:3]=np.eye(3)+math.sin(t)*K+(1-math.cos(t))*(K@K)
            return T
        def origin(j):
            o=j.find('origin')
            xyz=list(map(float,o.get('xyz','0 0 0').split())) if o is not None else [0]*3
            rpy=list(map(float,o.get('rpy','0 0 0').split())) if o is not None else [0]*3
            t=rot([0,0,1],rpy[2])@rot([0,1,0],rpy[1])@rot([1,0,0],rpy[0]);t[:3,3]=xyz
            return t
        joints=ET.parse(ROOT/lock['robot']['urdf']).findall('joint')
        movable=[j for j in joints if j.get('type')!='fixed']
        require([j.get('name') for j in movable]==model['urdf_joint_names']==lock['robot']['joint_order'],'Joint order mismatch')
        require(all(j.get('type')=='revolute' and j.find('axis').get('xyz')=='0 0 1' for j in movable),'Joint axis mismatch')
        for i,j in enumerate(movable):
            for field,attr in [('q_min_rad','lower'),('q_max_rad','upper'),('qd_max_rad_s','velocity')]:
                require(model[field][i]==float(j.find('limit').get(attr)),'Limit mismatch')
        def xml_poses(q):
            values=dict(zip(model['urdf_joint_names'],q));poses={'base_link':np.eye(4)};todo=list(joints)
            while todo:
                progress=False
                for j in list(todo):
                    parent=j.find('parent').get('link');child=j.find('child').get('link')
                    if parent not in poses:continue
                    t=origin(j)
                    if j.get('type')=='revolute':t=t@rot(list(map(float,j.find('axis').get('xyz').split())),values[j.get('name')])
                    poses[child]=poses[parent]@t;todo.remove(j);progress=True
                require(progress,'Disconnected URDF')
            return poses
        def dh_fk(q):
            t=np.asarray(model['T_base_dh0'],float)
            for i,theta in enumerate(q):
                a,d,alpha=model['a_m'][i],model['d_m'][i],model['alpha_rad'][i]
                c,s,ca,sa=math.cos(theta),math.sin(theta),math.cos(alpha),math.sin(alpha)
                t=t@np.array([[c,-s*ca,s*sa,a*c],[s,c*ca,-c*sa,a*s],[0,sa,ca,d],[0,0,0,1]])
            return t@model['T_dh6_flange']@model['T_flange_tool']
        reference=rtb.DHRobot([rtb.RevoluteDH(a=a,d=d,alpha=al) for a,d,al in zip(model['a_m'],model['d_m'],model['alpha_rad'])],name='UR5_project_nominal')
        reference.base=SE3(np.asarray(model['T_base_dh0'],float))
        reference.tool=SE3(np.asarray(model['T_dh6_flange'],float)@model['T_flange_tool'])
        builtin=rtb.models.DH.UR5()
        c_forward=None
        if args.library:
            sys.path.insert(0,str(ROOT/'src'))
            from adapters.c_kinematics import Forward
            c_forward=Forward(args.library)
            unit=Path(args.library).resolve().parent/'fk_unit'
            unit_run=subprocess.run([str(unit)],capture_output=True,text=True,timeout=30)
            report['c_unit']={'exit_code':unit_run.returncode,'stdout':unit_run.stdout,'stderr':unit_run.stderr}
            require(unit_run.returncode==0,'C matrix/known-pose contract tests failed')
            report['c_sources_sha256']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ['include/robot_kinematics.h','src/kinematics/forward.c','src/adapters/c_kinematics.py','tests/kinematics/test_fk.c']}
            report['c_library_sha256']=hashlib.sha256(args.library.read_bytes()).hexdigest()
            report['scope']='C_FK_and_nominal_model_alignment_not_IK_or_hardware'
        c_records=[]
        named=[('zero',[0]*6),('demo_home',[0,-math.pi/2,0,-math.pi/2,0,0]),('asymmetric',[.2,-.6,.8,-.5,.4,-.2])]
        for i in range(6):
            for sign in [-1,1]:
                q=[0.]*6;q[i]=sign*.2;named.append((f'J{i+1}_{sign:+d}',q))
        rng=np.random.default_rng(report['seed']); lo=np.array(model['q_min_rad'])+.05;hi=np.array(model['q_max_rad'])-.05
        named += [(f'random_{i:03d}',q.tolist()) for i,q in enumerate(rng.uniform(lo,hi,(100,6)))]
        records=[]; examples={}
        for name,q in named:
            poses=xml_poses(q);target=np.linalg.inv(poses['base'])@poses['tool0'];actual=dh_fk(q)
            transform_valid(target);transform_valid(actual)
            p_error=float(np.linalg.norm(actual[:3,3]-target[:3,3]));matrix_error=float(np.max(np.abs(actual[:3,:3]-target[:3,:3])))
            relative=target[:3,:3].T@actual[:3,:3]
            # atan2 form avoids acos cancellation at the ~1e-10 rad model-rounding scale.
            sine=.5*np.linalg.norm([relative[2,1]-relative[1,2],relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]])
            angle=float(np.arctan2(sine,np.clip((np.trace(relative)-1)/2,-1,1)))
            rt_error=float(np.max(np.abs(reference.fkine(q).A-actual)))
            flange_error=float(np.max(np.abs(actual@np.linalg.inv(model['T_flange_tool'])-np.linalg.inv(poses['base'])@poses['flange'])))
            base_error=float(np.max(np.abs(poses['base']-model['T_base_link_base'])))
            world_error=float(np.max(np.abs(np.asarray(model['T_world_base'])@actual-poses['tool0'])))
            builtin_error=float(np.linalg.norm(builtin.fkine(q).A[:3,3]-target[:3,3]))
            passed=p_error<=1e-9 and max(matrix_error,rt_error,flange_error,base_error,world_error)<=1e-9
            if c_forward:
                result=c_forward(model,q)
                require(result['code']==0 and result['data'] is not None,'C FK failed: '+name)
                c_pose=np.asarray(result['data']);transform_valid(c_pose)
                c_pos=float(np.linalg.norm(c_pose[:3,3]-target[:3,3]))
                c_rot=float(np.max(np.abs(c_pose[:3,:3]-target[:3,:3])))
                c_relative=target[:3,:3].T@c_pose[:3,:3]
                c_sine=.5*np.linalg.norm([c_relative[2,1]-c_relative[1,2],c_relative[0,2]-c_relative[2,0],c_relative[1,0]-c_relative[0,1]])
                c_angle=float(np.arctan2(c_sine,np.clip((np.trace(c_relative)-1)/2,-1,1)))
                c_ref=float(np.max(np.abs(c_pose-reference.fkine(q).A)))
                c_ok=c_pos<=1e-9 and c_rot<=1e-9 and c_ref<=1e-9
                passed=passed and c_ok
                c_records.append({'sample_id':name,'success':c_ok,'position_error_m':c_pos,'orientation_error_rad':c_angle,'rotation_matrix_max_error':c_rot,'rtb_max_element_error':c_ref})
            records.append({'sample_id':name,'q_rad':q,'success':passed,'position_error_m':p_error,'orientation_error_rad':angle,'rotation_matrix_max_error':matrix_error,'rtb_explicit_max_error':rt_error,'flange_max_error':flange_error,'world_max_error':world_error,'builtin_position_error_m':builtin_error})
            if name in ('zero','demo_home','asymmetric'):examples[name]={'q_rad':q,'T_base_tool_dh':actual.tolist(),'T_base_tool_urdf':target.tolist()}
        def stats(values):
            a=np.array(values);return {'valid_count':len(a),'invalid_count':0,'mean':float(a.mean()),'rmse':float(np.sqrt(np.mean(a*a))),'p95':float(np.quantile(a,.95,method='linear')),'max':float(a.max())}
        report.update(model_id=model['model_id'],expected_count=len(records),success_count=sum(v['success'] for v in records),
                      failure_count=sum(not v['success'] for v in records),position_error_m=stats([v['position_error_m'] for v in records]),
                      orientation_error_rad=stats([v['orientation_error_rad'] for v in records]),rotation_matrix_max_error=max(v['rotation_matrix_max_error'] for v in records),
                      builtin_d1_m=float(builtin.links[0].d),project_d1_m=model['d_m'][0],builtin_position_error_m=stats([v['builtin_position_error_m'] for v in records]),
                      explicit_rtb_max_error=max(v['rtb_explicit_max_error'] for v in records),fixed_examples=examples,
                      source_hashes=model['sources']['files_sha256'],profile_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
                      script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      rtb_version=version('roboticstoolbox-python'),numpy_version=np.__version__,
                      rtb_builtin_source_sha256=hashlib.sha256(Path(inspect.getfile(type(builtin))).read_bytes()).hexdigest(),
                      quantile_method='linear h=(n-1)*p',samples_file='samples.json')
        if c_forward:
            report['c_fk']={'expected_count':len(named),'success_count':sum(v['success'] for v in c_records),'failure_count':sum(not v['success'] for v in c_records),'position_error_m':stats([v['position_error_m'] for v in c_records]),'orientation_error_rad':stats([v['orientation_error_rad'] for v in c_records]),'rotation_matrix_max_error':max(v['rotation_matrix_max_error'] for v in c_records),'rtb_max_element_error':max(v['rtb_max_element_error'] for v in c_records)}
            report['c_fk']['success_rate']=report['c_fk']['success_count']/len(named)
            (args.output/'c-samples.json').write_text(json.dumps(c_records,indent=2,allow_nan=False)+'\n')
        report['success_rate']=report['success_count']/report['expected_count']
        (args.output/'samples.json').write_text(json.dumps(records,indent=2,allow_nan=False)+'\n')
        require(report['failure_count']==0,'Model alignment failed')
        report['status']='passed'
    except Exception as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2,allow_nan=False))
    return 0 if report['status']=='passed' else 1

if __name__=='__main__':
    sys.exit(main())
