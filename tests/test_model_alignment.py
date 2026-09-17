"""The independent URDF chain must detect an incorrect DH dimension."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

class ModelAlignmentRegression(unittest.TestCase):
    def test_wrong_d1_fails_against_urdf(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            shutil.copytree(ROOT/'models/ur5',root/'models/ur5')
            shutil.copy(ROOT/'environment.lock.json',root/'environment.lock.json')
            (root/'scripts').mkdir()
            shutil.copy(ROOT/'scripts/verify_model_alignment.py',root/'scripts/verify_model_alignment.py')
            profile=root/'models/ur5/kinematics.json'
            data=json.loads(profile.read_text());data['d_m'][0]=0.089459
            profile.write_text(json.dumps(data))
            proc=subprocess.run([sys.executable,str(root/'scripts/verify_model_alignment.py')],capture_output=True,text=True)
            self.assertEqual(proc.returncode,1,proc.stdout+proc.stderr)
            report=json.loads((root/'results/model-audit/report.json').read_text())
            self.assertEqual(report['status'],'failed')
            self.assertEqual(report['failure_count'],115)
            self.assertGreater(report['position_error_m']['max'],0.00029)

if __name__=='__main__':
    unittest.main()
