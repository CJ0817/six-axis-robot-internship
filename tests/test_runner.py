"""Failure propagation regression: an old success must not mask a failed launch."""
import importlib.util
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
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

    def test_timeout_is_failed_and_records_actual_wait(self):
        spec=importlib.util.spec_from_file_location('runner_under_test',ROOT/'scripts/run_tests.py')
        runner=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runner)
        with tempfile.TemporaryDirectory() as directory:
            argv=['run_tests.py','--suite','baseline','--output',directory]
            # Simulate the timeout exception; no real 120-second wait is needed.
            with patch.object(sys,'argv',argv), \
                 patch.object(runner.subprocess,'check_output',return_value=''), \
                 patch.object(runner.subprocess,'run',side_effect=subprocess.TimeoutExpired('test',120)), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(runner.main(),1)
            summary=json.loads((Path(directory)/'summary.json').read_text())
            record=summary['results'][0]
            self.assertEqual(summary['process_timeout_count'],1)
            self.assertTrue(record['timed_out'])
            self.assertEqual(record['timeout_budget_s'],120)
            self.assertGreaterEqual(record['actual_wait_s'],0)
            self.assertLess(record['actual_wait_s'],120)
            self.assertEqual(record['actual_wait_s'],record['process_wall_s'])
            self.assertIsNone(record['c_function_ms'])
            self.assertIsNone(record['python_to_c_ms'])

if __name__=='__main__':
    unittest.main()
