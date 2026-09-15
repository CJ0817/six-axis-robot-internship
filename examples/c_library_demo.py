"""Standalone ctypes call to the minimal C library; standard library only."""
import argparse
import ctypes
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

def run(library):
    lib = ctypes.CDLL(str(library.resolve()))
    copy = lib.robot_copy_joints
    pointer = ctypes.POINTER(ctypes.c_double)
    copy.argtypes = [pointer, ctypes.c_size_t, pointer]
    copy.restype = ctypes.c_int
    vector = ctypes.c_double * 6
    values = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    output = vector(*([42.] * 6))
    code = copy(vector(*values), 6, output)
    require(code == 0 and list(output) == values, 'C/Python roundtrip mismatch')
    checks = [{'case': 'six_joint_roundtrip', 'code': code, 'input_rad': values, 'output_rad': list(output)}]
    for name, source, count in [('wrong_length', vector(*values), 5),
                                ('nan', vector(0, 0, float('nan'), 0, 0, 0), 6),
                                ('infinity', vector(0, 0, 0, float('inf'), 0, 0), 6),
                                ('null_input', None, 6)]:
        output = vector(*([42.] * 6))
        code = copy(source, count, output)
        require(code == 1001 and list(output) == [42.] * 6, f'{name}: wrong code or changed output')
        checks.append({'case': name, 'code': code, 'output_unchanged': True})
    code = copy(vector(*values), 6, None)
    require(code == 1001, 'null_output: wrong code')
    checks.append({'case': 'null_output', 'code': code})
    return checks

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, default=ROOT / 'build/librobot_contract.so')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    report = {'status': 'failed', 'python': platform.python_version(), 'platform': platform.platform(),
              'utc': datetime.now(timezone.utc).isoformat(), 'library_name': args.library.name,
              'sizeof_double': ctypes.sizeof(ctypes.c_double), 'sizeof_size_t': ctypes.sizeof(ctypes.c_size_t)}
    try:
        require(args.library.is_file(), 'Dynamic library missing; compile it first (docs/c-library-quickstart.md)')
        report['library_sha256'] = hashlib.sha256(args.library.read_bytes()).hexdigest()
        report['source_sha256'] = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                                  for name in ['src/common/abi_probe.c', 'include/robot_contract.h', 'examples/c_library_demo.py']}
        report['checks'] = run(args.library)
        report['status'] = 'passed'
    except (OSError, AttributeError, RuntimeError) as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    output = json.dumps(report, indent=2)
    print(output)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output + '\n', encoding='utf-8')
    return 0 if report['status'] == 'passed' else 1

if __name__ == '__main__':
    sys.exit(main())
