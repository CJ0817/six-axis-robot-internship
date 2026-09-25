"""Run private C candidate-filter and public mixed-result regressions."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--executable',type=Path,default=ROOT/'build/ik_filter_unit');p.add_argument('--output',type=Path,default=ROOT/'results/ik-filter');a=p.parse_args()
    start=time.monotonic();report={'status':'failed','timeout_budget_s':10,'timeout_count':0,'executable':str(a.executable.resolve())}
    try:
        proc=subprocess.run([str(a.executable.resolve())],capture_output=True,text=True,timeout=10)
        report.update(exit_code=proc.returncode,stdout=proc.stdout,stderr=proc.stderr)
        counts=re.search(r'checks=(\d+) failed=(\d+)',proc.stdout)
        if counts:
            report.update(expected=int(counts[1]),failed=int(counts[2]),passed=int(counts[1])-int(counts[2]))
            if proc.returncode==0 and report['failed']==0:report['status']='passed'
    except subprocess.TimeoutExpired:report.update(timeout_count=1,reason='native test timed out')
    except OSError as exc:report['reason']=str(exc)
    report['actual_wait_s']=time.monotonic()-start
    report['source_sha256']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ['src/kinematics/inverse.c','tests/kinematics/test_ik_filter.c']}
    a.output.mkdir(parents=True,exist_ok=True);(a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    return 0 if report['status']=='passed' else 1
if __name__=='__main__':raise SystemExit(main())
