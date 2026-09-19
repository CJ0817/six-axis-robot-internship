"""115 frozen poses, 1000 warmups, 100 repeats each; native-C and ctypes wall time."""
import argparse
import csv
import ctypes as C
from datetime import datetime,timezone
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import Forward,pack_model,D6,D16

def summary(values):
    a=sorted(values);h=(len(a)-1)*.95;i=int(h)
    return {'completed_count':len(a),'mean_ms':statistics.mean(a),'rmse_ms':(sum(x*x for x in a)/len(a))**.5,'p95_ms':a[i]+(h-i)*(a[min(i+1,len(a)-1)]-a[i]),'max_ms':max(a),'over_1ms_count':sum(x>1 for x in a),'timeout_count':0}

def main():
    p=argparse.ArgumentParser();p.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so');p.add_argument('--output',type=Path,default=ROOT/'results/fk-performance');args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    report={'status':'failed','utc':datetime.now(timezone.utc).isoformat(),'python':platform.python_version(),'platform':platform.platform(),'cpu_count':os.cpu_count(),'warmups_per_layer':1000,'samples':115,'repetitions_per_sample':100,'thresholds_ms':{'single_call':1.0,'p95':1.0},'percentile':'linear h=(n-1)*p','performance_scope':'observed development host, not hard real-time guarantee'}
    whole=time.monotonic()
    try:
        source=ROOT/'results/model-audit/samples.json';records=json.loads(source.read_text());assert len(records)==115
        model=pack_model(json.loads((ROOT/'models/ur5/kinematics.json').read_text()));qs=[D6(*r['q_rad']) for r in records]
        f=Forward(args.library);out=D16();model_pointer=C.byref(model)
        fixture=args.output/'native-input.bin';fixture.write_bytes(bytes(model)+b''.join(bytes(q) for q in qs))
        report['samples_sha256']=hashlib.sha256(source.read_bytes()).hexdigest();report['library_sha256']=hashlib.sha256(args.library.read_bytes()).hexdigest()
        report['model_sha256']=hashlib.sha256((ROOT/'models/ur5/kinematics.json').read_bytes()).hexdigest()
        report['script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        report['native_driver_sha256']=hashlib.sha256((ROOT/'tests/kinematics/fk_benchmark.c').read_bytes()).hexdigest()
        cpu=Path('/proc/cpuinfo').read_text();report['cpu_model']=next((x.split(':',1)[1].strip() for x in cpu.splitlines() if x.startswith('model name')),'unknown')
        report['compiler']=subprocess.check_output(['gcc-13','--version'],text=True).splitlines()[0]
        report['build_requirement']='Release/O2 or O3, no sanitizer, GCC13; native calls include public input validation'
        start=time.monotonic()
        try:
            run=subprocess.run([str(args.library.resolve().parent/'fk_benchmark'),str(fixture.resolve())],capture_output=True,text=True,timeout=30)
        except subprocess.TimeoutExpired:
            report['native_process_timeout_count']=1;raise
        finally:
            report['native_process_actual_wait_s']=time.monotonic()-start
        report['native_process_timeout_count']=0
        if run.returncode:raise RuntimeError(run.stderr or f'native process exit {run.returncode}')
        (args.output/'c-function.csv').write_text(run.stdout)
        rows=list(csv.DictReader(io.StringIO(run.stdout)));assert len(rows)==11500
        assert all(int(r['code'])==0 and int(r['sample_index'])==i%115 for i,r in enumerate(rows))
        report['c_function']=summary([int(r['elapsed_ns'])/1e6 for r in rows])
        for i in range(1000):
            if f.fn(model_pointer,qs[i%115],6,out):raise RuntimeError('FFI warmup failed')
        timings=[]
        for i in range(11500):
            q=qs[i%115]
            start_ns=time.perf_counter_ns();code=f.fn(model_pointer,q,6,out);elapsed_ns=time.perf_counter_ns()-start_ns
            if code:raise RuntimeError('FFI call failed')
            timings.append(elapsed_ns/1e6)
        with (args.output/'python-to-c.csv').open('w',newline='') as file:
            writer=csv.writer(file);writer.writerow(['sample_index','elapsed_ms','code'])
            for i,t in enumerate(timings):writer.writerow([i%115,t,0])
        report['python_to_c']=summary(timings)
        report['status']='passed' if all(report[k]['max_ms']<=1 and report[k]['p95_ms']<=1 for k in ('c_function','python_to_c')) else 'failed'
        fixture.unlink()
    except Exception as exc:report['error']=f'{type(exc).__name__}: {exc}'
    report['total_process_work_s']=time.monotonic()-whole
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(report,indent=2));return 0 if report['status']=='passed' else 1
if __name__=='__main__':sys.exit(main())
