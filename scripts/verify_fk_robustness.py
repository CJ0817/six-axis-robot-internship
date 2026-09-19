"""15 boundary poses and 10 isolated malformed C ABI calls."""
import argparse
import ctypes as C
import json
import hashlib
from datetime import datetime,timezone
from pathlib import Path
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import Forward,pack_model,D6,D16
CASES={'null_model':1004,'null_q':1001,'null_output':1001,'length_zero':1001,'length_five':1001,'length_seven':1001,'nan':1001,'infinity':1001,'below_min':1005,'above_max':1005}

def anomalous(name,library,profile):
    f=Forward(library);model=pack_model(profile);q=D6(*([0]*6));out=D16(*([42]*16));pointer=C.byref(model);n=6
    if name=='null_model':pointer=None
    if name=='null_q':q=None
    if name.startswith('length_'):n={'length_zero':0,'length_five':5,'length_seven':7}[name]
    if name=='nan':q[2]=float('nan')
    if name=='infinity':q[2]=float('inf')
    if name=='below_min':q[0]=model.q_min_rad[0]-1e-6
    if name=='above_max':q[5]=model.q_max_rad[5]+1e-6
    before=bytes(out);code=f.fn(pointer,q,n,None if name=='null_output' else out)
    ok=code==CASES[name] and bytes(out)==before
    return {'case':name,'expected_code':CASES[name],'actual_code':code,'output_unchanged':None if name=='null_output' else bytes(out)==before,'status':'passed' if ok else 'failed'}

def main():
    p=argparse.ArgumentParser();p.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so');p.add_argument('--output',type=Path,default=ROOT/'results/fk-robustness');p.add_argument('--case',choices=CASES)
    args=p.parse_args();profile=json.loads((ROOT/'models/ur5/kinematics.json').read_text())
    if args.case:
        r=anomalous(args.case,args.library,profile);print(json.dumps(r));return 0 if r['status']=='passed' else 1
    args.output.mkdir(parents=True,exist_ok=True);report={'status':'failed','boundary':[],'exceptions':[],'utc':datetime.now(timezone.utc).isoformat(),'library_sha256':hashlib.sha256(args.library.read_bytes()).hexdigest(),'model_sha256':hashlib.sha256((ROOT/'models/ur5/kinematics.json').read_bytes()).hexdigest(),'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    try:
        import numpy as np
        import roboticstoolbox as rtb
        from spatialmath import SE3
        f=Forward(args.library)
        robot=rtb.DHRobot([rtb.RevoluteDH(a=a,d=d,alpha=al,offset=of,flip=si==-1) for a,d,al,of,si in zip(profile['a_m'],profile['d_m'],profile['alpha_rad'],profile['theta_offset_rad'],profile['joint_sign'])])
        robot.base=SE3(np.array(profile['T_base_dh0'],float));robot.tool=SE3(np.array(profile['T_dh6_flange'],float)@profile['T_flange_tool'])
        vectors=[]
        for i in range(6):
            for side in ('min','max'):
                q=[0.]*6;q[i]=profile[f'q_{side}_rad'][i];vectors.append((f'J{i+1}_{side}',q))
        vectors += [('all_min',profile['q_min_rad']),('all_max',profile['q_max_rad']),('zero_singular',[0.]*6)]
        for name,q in vectors:
            result=f(profile,q);error=None
            if result['code']==0:error=float(np.max(np.abs(np.array(result['data'])-robot.fkine(q).A)))
            ok=result['code']==0 and error is not None and np.isfinite(error) and error<=1e-9
            report['boundary'].append({'case':name,'q_rad':q,'code':result['code'],'max_matrix_error':error,'status':'passed' if ok else 'failed'})
        for name in CASES:
            start=time.monotonic()
            try:
                run=subprocess.run([sys.executable,str(Path(__file__).resolve()),'--library',str(args.library.resolve()),'--case',name],capture_output=True,text=True,timeout=10)
                if run.returncode==0:record=json.loads(run.stdout)
                else:record={'case':name,'status':'failed','exit_code':run.returncode,'stdout':run.stdout,'stderr':run.stderr}
            except subprocess.TimeoutExpired:record={'case':name,'status':'failed','timeout':True}
            record['actual_wait_s']=time.monotonic()-start;report['exceptions'].append(record)
        report['boundary_passed']=sum(r['status']=='passed' for r in report['boundary']);report['exception_passed']=sum(r['status']=='passed' for r in report['exceptions'])
        report['status']='passed' if report['boundary_passed']==15 and report['exception_passed']==10 else 'failed'
    except Exception as exc:report['error']=f'{type(exc).__name__}: {exc}'
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(report,indent=2));return 0 if report['status']=='passed' else 1
if __name__=='__main__':sys.exit(main())
