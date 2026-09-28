import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class FrozenWorkspace(unittest.TestCase):
    def test_changed_target_rejected_without_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            for name in ['workspace120.json','workspace120-special.json']:shutil.copyfile(ROOT/'tests/ik'/name,tmp/name)
            path=tmp/'workspace120.json';data=json.loads(path.read_text());data['samples'][0]['target_T_base_tool'][0][3]+=.001;path.write_text(json.dumps(data))
            before=hashlib.sha256(path.read_bytes()).hexdigest()
            r=subprocess.run([sys.executable,str(ROOT/'scripts/prepare_workspace120.py'),'--fixture-dir',str(tmp),'--output',str(tmp/'report')],capture_output=True,text=True,timeout=30)
            self.assertEqual(r.returncode,1,r.stdout+r.stderr)
            self.assertEqual(json.loads((tmp/'report/report.json').read_text())['status'],'failed')
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),before)
if __name__=='__main__':unittest.main()
