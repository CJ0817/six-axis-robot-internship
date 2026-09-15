"""Integration regression for caller-relative and space-containing Python paths."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class InstallPaths(unittest.TestCase):
    def test_relative_python_from_other_directory(self):
        with tempfile.TemporaryDirectory(prefix='robot paths ') as directory:
            path = Path(directory)
            (path/'python with spaces').symlink_to(Path(sys.executable).absolute())
            env = dict(os.environ, ROBOT_PYTHON='./python with spaces')
            for script in ['setup.sh', 'setup_baseline.sh']:
                run = subprocess.run(['bash', str(ROOT/'scripts'/script), '--check'],
                                     cwd=path, env=env, capture_output=True, text=True)
                self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
                self.assertIn('Installation prerequisites and Python path: passed', run.stdout)

if __name__ == '__main__':
    unittest.main()
