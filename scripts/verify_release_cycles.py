"""Clean-build release repetition and same-plant continuous dynamics acceptance."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback
from importlib.metadata import version
ROOT=Path(__file__).resolve().parents[1]


def read_report(path):
    """Require an actual fresh successful report, never a process exit alone."""
    data=json.loads(Path(path).read_text())
    if data.get('passed') is not True:
        raise AssertionError('Dynamics report failed: '+str(path))
    return data


def compare_csv(left,right,tolerance):
    """Compare complete numeric/control traces; only wall processing time differs."""
    with Path(left).open(newline='') as a,Path(right).open(newline='') as b:
        aa=list(csv.DictReader(a));bb=list(csv.DictReader(b))
    if not aa or len(aa)!=len(bb): raise AssertionError('Trace sample count mismatch')
    peak=0.;compared=0
    for x,y in zip(aa,bb):
        if x.keys()!=y.keys(): raise AssertionError('Trace schema mismatch')
        for key in x:
            if key=='processing_wall_s': continue
            try: delta=abs(float(x[key])-float(y[key]))
            except ValueError:
                if x[key]!=y[key]: raise AssertionError('Trace value mismatch: '+key)
                continue
            if not math.isfinite(delta) or delta>tolerance:
                raise AssertionError('Repeated trace drift: '+key+' '+str(delta))
            peak=max(peak,delta);compared+=1
    return dict(samples=len(aa),numeric_values=compared,max_abs_delta=peak)


def execute(out,cycles):
    """Start one fresh process; --cycles keeps one physical client inside it."""
    command=[sys.executable,str(ROOT/'scripts/verify_fixed_transfer.py'),'--cycles',str(cycles),'--output',str(out)]
    started=time.perf_counter()
    with out.parent.joinpath(out.name+'.log').open('w') as log:
        code=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=900).returncode
    if code: raise AssertionError('Dynamics subprocess failed: '+out.name+'; inspect log and failure report')
    report=read_report(out/'report.json')
    return report,dict(command=command,process_wall_s=time.perf_counter()-started)


def main():
    """Run release config, emit all failures and return nonzero on any unmet gate."""
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source-commit',required=True)
    args=p.parse_args();out=args.output.resolve()
    if out.exists(): raise FileExistsError('A new empty output directory is required: '+str(out))
    out.mkdir(parents=True)
    c=json.loads((ROOT/'config/release-acceptance.v1.json').read_text())
    result=dict(passed=False,source_commit=args.source_commit,usage='simulation_only',parameters=c,
                environment=dict(python=sys.version,platform=platform.platform(),machine=platform.machine(),
                    pybullet=version('pybullet'),numpy=version('numpy'),pillow=version('Pillow'),
                    imageio_ffmpeg=version('imageio-ffmpeg')),repetitions=[],comparisons=[])
    try:
        for i in range(c['independent_repetitions']):
            folder=out/('repeat-'+str(i+1))
            report,process=execute(folder,1)
            if report['global_ticks']!=2700 or report['zero_target_count']!=4 or report['cycles_completed']!=1:
                raise AssertionError('Incomplete independent transfer')
            result['repetitions'].append(dict(index=i+1,global_ticks=report['global_ticks'],
                duration_s=report['duration_s'],return_error_rad=report['return_max_error_rad'],
                placement_error_m=report['actual_payload_retention_error_m'],**process))
            if i:
                paths=sorted((out/'repeat-1').glob('*-control.csv'))
                paths += [out/'repeat-1'/name for name in ('global-commands.csv','object-feedback.csv','zero-targets.csv')]
                for baseline in paths:
                    comparison=compare_csv(baseline,folder/baseline.name,c['numeric_repeat_tolerance'])
                    result['comparisons'].append(dict(repetition=i+1,file=baseline.name,**comparison))
            print('independent repetition',i+1,'passed',flush=True)
        report,process=execute(out/'continuous',c['continuous_cycles'])
        cycles=report['cycles']
        if report['cycles_completed']!=c['continuous_cycles'] or report['physical_plant_instances']!=1:
            raise AssertionError('Continuous run recreated or omitted a physical plant')
        expected_ticks=0
        for i,cycle in enumerate(cycles):
            if cycle['cycle']!=i+1 or cycle['start_tick']!=expected_ticks:
                raise AssertionError('Continuous cycle clock gap')
            if i and cycle['initial_q_rad']!=cycles[i-1]['final_q_rad']:
                raise AssertionError('Joint state reset between cycles')
            if (cycle['physics_client'],cycle['robot'],cycle['payload'])!=(cycles[0]['physics_client'],cycles[0]['robot'],cycles[0]['payload']):
                raise AssertionError('Continuous plant identity changed')
            expected_ticks=cycle['end_tick']
        task=report['parameters'];period=report['period_s']
        durations=list(task['segment_durations_s']);durations[0]=max(durations)
        expected_duration=c['continuous_cycles']*(sum(durations)+4*task['settle_s'])
        if report['global_ticks']!=round(expected_duration/period) or report['zero_target_count']!=4*c['continuous_cycles']:
            raise AssertionError('Incomplete continuous duration or stop log')
        if len(report['stages'])!=4*c['continuous_cycles']: raise AssertionError('Missing continuous stages')
        if any(x['result']['code']!=3003 for x in report['direct_route_counterexamples']):
            raise AssertionError('Missing carried-payload collision rejection')
        result['continuous']=dict(cycles=cycles,global_ticks=report['global_ticks'],duration_s=report['duration_s'],
            physical_plant_instances=report['physical_plant_instances'],zero_targets=report['zero_target_count'],
            maximum_attachment_error_m=report['maximum_attachment_error_m'],
            minimum_actual_payload_clearance_m=report['minimum_actual_payload_clearance_m'],**process)
        result['max_repeat_delta']=max(x['max_abs_delta'] for x in result['comparisons'])
        result['passed']=True
        print('same-plant continuous cycles',c['continuous_cycles'],'passed; ticks',expected_ticks,flush=True)
    except Exception as exc:
        result['failure']=dict(message=str(exc),traceback=traceback.format_exc())
        print(result['failure']['traceback'],flush=True)
    finally:
        result['source_sha256']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [Path(__file__).resolve(),ROOT/'scripts/verify_fixed_transfer.py',ROOT/'config/release-acceptance.v1.json']}
        (out/'release-tests.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return 0 if result['passed'] else 1


if __name__=='__main__': sys.exit(main())
