"""Extract pinned build bootstrap packages, preserving hashes from the full lock."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
blocks = re.split(r'(?=^[a-zA-Z0-9_.-]+==)', (root/'requirements/baseline.lock').read_text(), flags=re.M)
wanted = {'numpy', 'setuptools', 'wheel', 'pip'}
selected = [block for block in blocks if block.split('==')[0] in wanted]
if len(selected) != len(wanted):
    raise ValueError('Missing or duplicate bootstrap dependency')
(root/'requirements/baseline-build.lock').write_text(
    '# Build bootstrap extracted from baseline.lock; do not edit independently.\n' + ''.join(selected))
