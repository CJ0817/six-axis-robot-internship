"""Single test entry: smoke, baseline, available, acceptance. No silent skips."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=['smoke','baseline','available','acceptance'], default='smoke')
    parser.add_argument('--output', type=Path, default=ROOT/'results/test-runs/latest')
    parser.add_argument('--smoke-python', type=Path, default=ROOT/'.venv/bin/python')
    parser.add_argument('--baseline-python', type=Path, default=ROOT/'.venv-baseline/bin/python')
    parser.add_argument('--library', type=Path, default=ROOT/'build/librobot_contract.so')
    args=parser.parse_args()
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    metrics=json.loads((ROOT/'tests/metrics.json').read_text())
    definitions=metrics['metrics']
    ids=[item['id'] for item in definitions]
    if len(set(ids))!=len(ids):
        raise ValueError('Duplicate metric ID')
    selected=['c_abi','scene'] if args.suite=='smoke' else ['rtb'] if args.suite=='baseline' else ['c_abi','scene','rtb']
    specs={
        'c_abi': [str(args.smoke_python.absolute()),str(ROOT/'examples/c_library_demo.py'),'--library',str(args.library.resolve()),'--report',str(output/'c_abi/report.json')],
        'scene': [str(args.smoke_python.absolute()),str(ROOT/'examples/scene_demo.py'),'--headless','--output',str(output/'scene')],
        'rtb': [str(args.baseline_python.absolute()),str(ROOT/'tests/baseline/robotics_toolbox_smoke.py'),'--output',str(output/'rtb')],
    }
    reports={name:output/name/'report.json' for name in specs}
    summary={'suite':args.suite,'utc':datetime.now(timezone.utc).isoformat(),'results':[],
             'metrics_sha256':hashlib.sha256((ROOT/'tests/metrics.json').read_bytes()).hexdigest()}
    try:
        summary['commit']=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        summary['working_tree_dirty']=bool(subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip())
    except (OSError,subprocess.SubprocessError):
        summary['commit']=None
    env=os.environ.copy()
    env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',MPLBACKEND='Agg')
    for name in selected:
        record={'test_id':name,'command':specs[name],'status':'failed'}
        start=time.monotonic()
        reports[name].parent.mkdir(parents=True,exist_ok=True)
        # Remove only the exact prior report, so a failed launch cannot reuse stale success.
        reports[name].unlink(missing_ok=True)
        try:
            with (output/(name+'.log')).open('w') as log:
                proc=subprocess.run(specs[name],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120)
            record['exit_code']=proc.returncode
            if proc.returncode==0 and reports[name].is_file():
                data=json.loads(reports[name].read_text())
                if data.get('status')=='passed':
                    record['status']='passed'
                    record['report']=str(reports[name].relative_to(output))
            if record['status']!='passed':
                record['reason']='nonzero exit, missing report or report not passed; inspect log'
        except (OSError,subprocess.TimeoutExpired,ValueError) as exc:
            record['reason']=f'{type(exc).__name__}: {exc}'
        record['elapsed_s']=time.monotonic()-start
        summary['results'].append(record)
    if args.suite=='acceptance':
        for item in definitions:
            if item['availability']=='pending':
                summary['results'].append({'test_id':item['id'],'status':'blocked','reason':item['pending_reason']})
    statuses=[r['status'] for r in summary['results']]
    code=1 if 'failed' in statuses else 2 if 'blocked' in statuses else 0
    summary['status']='failed' if code==1 else 'blocked' if code==2 else 'passed'
    summary['scope_note']='Available checks do not imply completion of unimplemented algorithm or user desktop acceptance.'
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    return code

if __name__=='__main__':
    sys.exit(main())
