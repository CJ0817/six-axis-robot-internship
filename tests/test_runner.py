"""Failure propagation regression: an old success must not mask a failed launch."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

class RunnerFailureTest(unittest.TestCase):
    def test_missing_interpreter_rejects_stale_success(self):
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory)
            (out/'rtb').mkdir()
            (out/'rtb/report.json').write_text('{"status":"passed"}')
            proc=subprocess.run([sys.executable,str(ROOT/'scripts/run_tests.py'),
                                 '--suite','baseline','--baseline-python',str(out/'missing-python'),
                                 '--output',str(out)],capture_output=True,text=True)
            self.assertEqual(proc.returncode,1,proc.stdout+proc.stderr)
            summary=json.loads((out/'summary.json').read_text())
            self.assertEqual(summary['status'],'failed')
            self.assertFalse((out/'rtb/report.json').exists())
            self.assertEqual(summary['results'][0]['status'],'failed')

if __name__=='__main__':
    unittest.main()
