"""Verify the reference toolchain, not the internship's unimplemented C solvers."""
import argparse
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[2]

def require(ok, message):
    if not ok:
        raise RuntimeError(message)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = {'status': 'failed', 'scope': 'reference_dependency_readiness_only',
              'internship_algorithm_acceptance': 'not_run', 'python': platform.python_version(),
              'platform': platform.platform(), 'seed': 20260915, 'sample_count': 100,
              'utc': datetime.now(timezone.utc).isoformat()}
    try:
        require(platform.python_version() == '3.11.9', 'Baseline requires Python 3.11.9')
        lock = ROOT / 'requirements/baseline.lock'
        pinned = dict(re.findall(r'^([A-Za-z0-9_.-]+)==([^\s;\\]+)', lock.read_text(), re.M))
        require(bool(pinned), 'Empty dependency lock')
        installed = {name: version(name) for name in pinned}
        require(installed == pinned, 'Installed package versions do not match baseline.lock')
        report['versions'] = installed
        report['lock_sha256'] = hashlib.sha256(lock.read_bytes()).hexdigest()
        report['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        os.environ['MPLBACKEND'] = 'Agg'
        for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
            os.environ[name] = '1'
        import numpy as np
        import roboticstoolbox as rtb
        robot = rtb.models.DH.UR5()
        require(robot.n == 6, 'Reference UR5 must have six joints')
        rng = np.random.default_rng(report['seed'])
        q = rng.uniform(-np.pi, np.pi, (100, 6))
        transforms, jacobians, elapsed = [], [], []
        for joint in q[:10]:
            robot.fkine(joint)
        for joint in q:
            start = time.perf_counter_ns()
            transform = robot.fkine(joint).A
            elapsed.append((time.perf_counter_ns()-start)/1e6)
            jacobian = robot.jacob0(joint)
            require(transform.shape == (4, 4) and jacobian.shape == (6, 6), 'Reference output shape')
            require(np.isfinite(transform).all() and np.isfinite(jacobian).all(), 'Nonfinite reference')
            require(np.allclose(transform[3], [0, 0, 0, 1], atol=1e-12, rtol=0), 'Invalid homogeneous transform')
            require(np.linalg.norm(transform[:3,:3].T @ transform[:3,:3]-np.eye(3)) <= 1e-9, 'Invalid rotation')
            transforms.append(transform.tolist())
            jacobians.append(jacobian.tolist())
        known_q = np.array([0.2, -0.6, 0.8, -0.5, 0.4, -0.2])
        target = robot.fkine(known_q)
        solution = robot.ikine_LM(target, q0=known_q+0.01, ilimit=200, slimit=1, tol=1e-12)
        require(bool(solution.success), 'Reference IK did not converge')
        actual = robot.fkine(solution.q).A
        position_error = float(np.linalg.norm(actual[:3,3]-target.A[:3,3]))
        rotation = target.A[:3,:3] @ actual[:3,:3].T
        angle_error = float(np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1)))
        require(position_error <= 1e-5 and angle_error <= 1e-4, 'Reference IK roundtrip mismatch')
        dataset = {'purpose': 'reference_only_not_C_algorithm_test', 'model': 'rtb.models.DH.UR5',
                   'frame': 'RTB default DH base/tool; NOT asserted aligned to project URDF',
                   'joint_order': ['J1','J2','J3','J4','J5','J6'], 'units': {'q': 'rad', 'translation': 'm'},
                   'sampling': '100 independent uniform samples in [-pi,pi], seed=20260915; readiness set only',
                   'q_rad': q.tolist(), 'T_reference': transforms, 'J_reference': jacobians,
                   'base_transform': robot.base.A.tolist(), 'tool_transform': robot.tool.A.tolist(),
                   'dh': [{'a_m': float(link.a), 'd_m': float(link.d), 'alpha_rad': float(link.alpha),
                           'offset_rad': float(link.offset)} for link in robot.links]}
        artifact = args.output / 'ur5-reference-samples.json'
        artifact.write_text(json.dumps(dataset, indent=2)+'\n')
        report.update(status='passed', model='rtb.models.DH.UR5', fk_finite_samples=100, jacobian_finite_samples=100,
                      ik_position_error_m=position_error, ik_orientation_error_rad=angle_error,
                      reference_fk_p95_ms=float(np.percentile(elapsed,95)), performance_is_informational=True,
                      reference_file=artifact.name, reference_sha256=hashlib.sha256(artifact.read_bytes()).hexdigest(),
                      urdf_alignment='pending; do not compare against project C/URDF until frames and DH verified')
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['status']=='passed' else 1

if __name__ == '__main__':
    sys.exit(main())
