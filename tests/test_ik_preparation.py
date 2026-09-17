"""Frozen inputs must fail verification when modified, not be silently overwritten."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

class FrozenIKTargets(unittest.TestCase):
    def test_changed_target_is_rejected_without_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for name in ['scripts/prepare_ik_targets.py','src/adapters/c_kinematics.py',
                         'models/ur5/kinematics.json','tests/ik/targets.json']:
                destination=root/name;destination.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy(ROOT/name,destination)
            fixture=root/'tests/ik/targets.json';data=json.loads(fixture.read_text())
            data['groups']['normal'][0]['target_T_base_tool'][0][3]+=.001
            fixture.write_text(json.dumps(data));before=hashlib.sha256(fixture.read_bytes()).hexdigest()
            proc=subprocess.run([sys.executable,str(root/'scripts/prepare_ik_targets.py'),
                                 '--library',str(ROOT/'build/librobot_contract.so')],capture_output=True,text=True)
            self.assertEqual(proc.returncode,1,proc.stdout+proc.stderr)
            report=json.loads((root/'results/ik-preparation/report.json').read_text())
            self.assertEqual(report['status'],'failed')
            self.assertIn('Frozen fixture differs',report['error'])
            self.assertEqual(before,hashlib.sha256(fixture.read_bytes()).hexdigest())
if __name__=='__main__':unittest.main()
