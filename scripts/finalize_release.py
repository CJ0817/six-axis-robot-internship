"""Gate the exact main commit, package raw evidence, then create an immutable version."""
import argparse
from datetime import datetime,timezone
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import time
import zipfile
ROOT=Path(__file__).resolve().parents[1]


def run(args,**kwargs):
    """Run an explicit argument vector and propagate failure; never print credentials."""
    return subprocess.run(args,check=True,**kwargs)


def api(path):
    """Read repository-scoped GitHub metadata via the runner's masked token."""
    return json.loads(subprocess.check_output(['gh','api',path],text=True))


def gate(commit,names):
    """Require all five current-commit push workflows, without accepting stale runs."""
    deadline=time.monotonic()+900
    while time.monotonic()<deadline:
        runs=api('repos/'+os.environ['GITHUB_REPOSITORY']+'/actions/runs?head_sha='+commit+'&per_page=100')['workflow_runs']
        selected={}
        for name in names:
            matching=[r for r in runs if r['name']==name and r['event']=='push' and r['head_sha']==commit]
            if matching: selected[name]=max(matching,key=lambda r:r['id'])
        if any(r['status']=='completed' and r['conclusion']!='success' for r in selected.values()):
            raise AssertionError('Required workflow failed: '+json.dumps({n:r['conclusion'] for n,r in selected.items()}))
        if len(selected)==len(names) and all(r['status']=='completed' and r['conclusion']=='success' for r in selected.values()):
            return selected
        print('Waiting for exact-commit gates:',{n:selected.get(n,{}).get('status','not_started') for n in names},flush=True)
        time.sleep(15)
    raise TimeoutError('Current-commit required workflows did not finish in 900 s')


def archive_artifacts(runs,folder):
    """Persist complete functional/boundary/exception evidence alongside new dynamics."""
    folder.mkdir(parents=True,exist_ok=True)
    records=[]
    for name,r in runs.items():
        items=api('repos/'+os.environ['GITHUB_REPOSITORY']+'/actions/runs/'+str(r['id'])+'/artifacts')['artifacts']
        if not items: raise AssertionError('Required workflow has no retained evidence: '+name)
        for item in items:
            if item['expired']: raise AssertionError('Evidence expired before release')
            dest=folder/(str(r['id'])+'-'+str(item['id'])+'.zip')
            with dest.open('wb') as stream:
                run(['gh','api','repos/'+os.environ['GITHUB_REPOSITORY']+'/actions/artifacts/'+str(item['id'])+'/zip'],stdout=stream)
            digest=hashlib.sha256(dest.read_bytes()).hexdigest()
            expected=item.get('digest')
            if expected and expected!='sha256:'+digest: raise AssertionError('Artifact checksum mismatch')
            with zipfile.ZipFile(dest) as z:
                if z.testzip(): raise AssertionError('Corrupted workflow evidence ZIP')
            records.append(dict(workflow=name,run_id=r['id'],artifact_id=item['id'],file=dest.name,sha256=digest,bytes=dest.stat().st_size))
    return records


def main():
    """Publish only tested, current main; refuse moved tags, stale sources and overwrites."""
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--commit',required=True)
    p.add_argument('--evidence',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if os.environ.get('GITHUB_REF')!='refs/heads/main': raise AssertionError('Freeze only main')
    c=json.loads((ROOT/'config/release-acceptance.v1.json').read_text())
    tag=c['tag'];repo=os.environ['GITHUB_REPOSITORY'];out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=True)
    dynamic=json.loads((args.evidence/'dynamics/release-tests.json').read_text())
    clean=json.loads((args.evidence/'clean-source.json').read_text())
    if dynamic.get('passed') is not True or dynamic['source_commit']!=args.commit or clean['source_commit']!=args.commit:
        raise AssertionError('Fresh source/dynamics proof missing or for another commit')
    current=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    if current!=args.commit: raise AssertionError('Packaging checkout differs from tested source')
    runs=gate(args.commit,c['required_workflows'])
    evidence=out/'evidence'
    shutil.copytree(args.evidence,evidence,dirs_exist_ok=True)
    artifacts=archive_artifacts(runs,evidence/'workflow-artifacts')
    tracked=subprocess.check_output(['git','ls-files','-z']).decode().split('\0')
    source_manifest={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in tracked if name}
    (out/'source-manifest.json').write_text(json.dumps(dict(commit=args.commit,files_sha256=source_manifest),indent=2)+'\n')
    summary=dict(version=tag,source_commit=args.commit,utc=datetime.now(timezone.utc).isoformat(),status='passed_simulation_scope',
        clean_source=clean,dynamics=dynamic,required_workflows={n:dict(id=r['id'],conclusion=r['conclusion'],url=r['html_url']) for n,r in runs.items()},
        evidence_artifacts=artifacts,exclusions=c['exclusions'],freeze_run=os.environ['GITHUB_SERVER_URL']+'/'+repo+'/actions/runs/'+os.environ['GITHUB_RUN_ID'])
    (out/'TEST_RESULTS.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    lines=['# '+tag+' final delivery','','Source commit: '+args.commit,'Scope: UR5/PyBullet simulation; Linux x86_64 frozen C ABI.','','## Acceptance results','','|Category|Result|','|---|---|']
    for name,r in runs.items(): lines.append('|'+name+'|PASS: '+r['html_url']+'|')
    lines.extend(['|Fresh archive/build/output|PASS: '+str(clean['tracked_files'])+' source files verified|',
        '|Independent full transfers|PASS: '+str(len(dynamic['repetitions']))+' x 54 s|',
        '|Repeated numeric trace max delta|'+str(dynamic['max_repeat_delta'])+' (limit 1e-9)|',
        '|One-plant continuous cycles|PASS: '+str(len(dynamic['continuous']['cycles']))+' cycles / '+str(dynamic['continuous']['duration_s'])+' s / '+str(dynamic['continuous']['global_ticks'])+' intervals|',
        '|Continuous zero-target writes|'+str(dynamic['continuous']['zero_targets'])+'|','','## Scope limitations',''])
    for item in c['exclusions']: lines.append('- '+item['metric']+': '+item['reason'])
    lines.extend(['','## Files and reproduction','','Source, configuration, models, examples and interface documentation are in the delivery source tree. Raw new traces/reports/videos are under acceptance/dynamics/. Raw functional/boundary/exception logs from every required workflow are under acceptance/workflow-artifacts/. Source hashes are in source-manifest.json. Package hashes are in SHA256SUMS. Read docs/release-delivery.md and docs/integration-issues.md before porting.',
        '','The version tag points exactly to the tested source commit. No existing tag is moved and no branch history is rewritten. The release contains the original single-transfer demo and the full continuous-run demo. No hardware safety, real grasp friction or hard real-time guarantee is claimed.','',
        'Previously repaired issues retained in regression: complete trajectory certification, candidate rejection, common cycle mismatch rejection, current NaN/previous sample context, measured-orientation transitions and explicit phase-zero targets. This clean acceptance adds repeat drift and persistent-plant reset/clock checks.',''])
    (out/'FINAL_REPORT.md').write_text('\n'.join(lines))
    source_zip=out/(tag+'-source.zip')
    run(['git','archive','--format=zip','--output',str(source_zip),args.commit])
    delivery=out/(tag+'-delivery.tar.gz')
    prefix='six-axis-robot-'+tag
    with tarfile.open(delivery,'w:gz') as tar:
        with zipfile.ZipFile(source_zip) as z:
            for item in z.infolist():
                if item.is_dir(): continue
                data=z.read(item)
                info=tarfile.TarInfo(prefix+'/'+item.filename);info.size=len(data);info.mode=0o755 if (item.external_attr>>16)&0o111 else 0o644
                tar.addfile(info,io.BytesIO(data))
        tar.add(evidence,arcname=prefix+'/acceptance')
        for name in ('FINAL_REPORT.md','TEST_RESULTS.json','source-manifest.json'):
            tar.add(out/name,arcname=prefix+'/'+name)
    shutil.copyfile(evidence/'dynamics/repeat-1/demo.mp4',out/'demo.mp4')
    shutil.copyfile(evidence/'dynamics/continuous/demo.mp4',out/'continuous-demo.mp4')
    assets=[source_zip,delivery,out/'FINAL_REPORT.md',out/'TEST_RESULTS.json',out/'source-manifest.json',out/'demo.mp4',out/'continuous-demo.mp4']
    (out/'SHA256SUMS').write_text(''.join(hashlib.sha256(path.read_bytes()).hexdigest()+'  '+path.name+'\n' for path in assets))
    assets.append(out/'SHA256SUMS')
    remote=subprocess.check_output(['git','ls-remote','origin','refs/heads/main'],text=True).split()[0]
    if remote!=args.commit: raise AssertionError('Main moved during acceptance; do not freeze stale source')
    existing=subprocess.check_output(['git','ls-remote','origin','refs/tags/'+tag],text=True).strip()
    if existing: raise AssertionError('Tag already exists; never move or overwrite it')
    run(['git','config','user.name','github-actions[bot]'])
    run(['git','config','user.email','41898282+github-actions[bot]@users.noreply.github.com'])
    run(['git','tag','-a',tag,args.commit,'-m','Validated internship simulation freeze; source '+args.commit])
    run(['git','push','origin','refs/tags/'+tag])
    run(['gh','release','create',tag,'--repo',repo,'--verify-tag','--title',tag+' - validated simulation delivery','--notes-file',str(out/'FINAL_REPORT.md'),*[str(path) for path in assets]])
    print(json.dumps(dict(tag=tag,commit=args.commit,status='frozen',release='https://github.com/'+repo+'/releases/tag/'+tag)),flush=True)


if __name__=='__main__': main()
