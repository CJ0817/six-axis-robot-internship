"""Version, model, C ABI and PyBullet smoke gate. Exit nonzero on any mismatch."""
import argparse
import ctypes
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import platform
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
LOCK = json.loads((ROOT / 'environment.lock.json').read_text())

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

def command(*args):
    return subprocess.check_output(args, text=True).strip()

def versions():
    data = {'python': platform.python_version(), 'platform': platform.platform(),
            'machine': platform.machine(), 'pip': version('pip'), 'gcc': command('gcc-13', '-dumpfullversion'),
            'gxx': command('g++-13', '-dumpfullversion'),
            'cmake': command('cmake', '--version').splitlines()[0].split()[-1],
            'ninja': command('ninja', '--version'),
            'packages': {name: version(name) for name in LOCK['packages']}}
    require(sys.platform == 'linux' and data['machine'] == 'x86_64', 'Locked baseline is Linux x86_64')
    for key in ('python', 'pip', 'gcc', 'gxx', 'cmake', 'ninja'):
        require(data[key] == LOCK[key], f'{key}: expected {LOCK[key]}, got {data[key]}')
    require(data['packages'] == LOCK['packages'], 'Python package versions differ from lock')
    return data

def model_check():
    folder = ROOT / 'models/ur5'
    manifest = json.loads((folder / 'manifest.json').read_text())
    require(manifest['upstream_commit'] == LOCK['robot']['commit'], 'Model commit mismatch')
    for name, digest in manifest['sha256'].items():
        require(hashlib.sha256((folder / name).read_bytes()).hexdigest() == digest, f'Model checksum: {name}')
    tree = ET.parse(folder / 'ur5.urdf')
    names = [joint.attrib['name'] for joint in tree.findall('joint') if joint.attrib['type'] != 'fixed']
    require(names == LOCK['robot']['joint_order'], 'URDF joint order mismatch')
    for mesh in tree.findall('.//mesh'):
        require((folder / mesh.attrib['filename']).is_file(), 'Missing mesh')
    return {'movable_joints': names, 'verified_assets': len(manifest['sha256']), 'upstream_commit': manifest['upstream_commit']}

def c_abi():
    lib = ctypes.CDLL(str(ROOT / 'build/librobot_contract.so'))
    fn = lib.robot_copy_joints
    pointer = ctypes.POINTER(ctypes.c_double)
    fn.argtypes = (pointer, ctypes.c_size_t, pointer)
    fn.restype = ctypes.c_int
    values = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    array = ctypes.c_double * 6
    incoming, outgoing = array(*values), array(*([42.] * 6))
    require(fn(incoming, 6, outgoing) == 0 and list(outgoing) == values, 'C/Python roundtrip failed')
    for count, invalid in ((5, values), (6, [0., 0., float('nan'), 0., 0., 0.])):
        incoming, outgoing = array(*invalid), array(*([42.] * 6))
        require(fn(incoming, count, outgoing) == 1001 and list(outgoing) == [42.] * 6, 'C failure contract violated')
    return {'roundtrip': 'passed', 'invalid_length_and_nan': 'passed'}

def bullet_smoke():
    import numpy as np
    import pybullet as p
    client = p.connect(p.DIRECT)
    require(client >= 0, 'Cannot connect to PyBullet DIRECT')
    try:
        p.setGravity(0, 0, -9.81, physicsClientId=client)
        p.setTimeStep(1.0 / 240, physicsClientId=client)
        robot = p.loadURDF(str(ROOT / LOCK['robot']['urdf']), useFixedBase=True,
                           flags=p.URDF_USE_INERTIA_FROM_FILE, physicsClientId=client)
        info = [p.getJointInfo(robot, i, physicsClientId=client) for i in range(p.getNumJoints(robot, physicsClientId=client))]
        mapping = {row[1].decode(): row[0] for row in info if row[2] != p.JOINT_FIXED}
        require(set(mapping) == set(LOCK['robot']['joint_order']), 'PyBullet movable joints mismatch')
        joints = [mapping[name] for name in LOCK['robot']['joint_order']]
        require(any(row[12].decode() == 'tool0' for row in info), 'Missing tool0 link')
        for row in info:
            if row[12].decode() in ('base', 'flange', 'tool0'):
                require(p.getDynamicsInfo(robot, row[0], physicsClientId=client)[0] == 0,
                        'Coordinate-only link must not acquire default mass')
        initial = [0., -np.pi/2, 0., -np.pi/2, 0., 0.]
        for joint, q in zip(joints, initial):
            p.resetJointState(robot, joint, q, physicsClientId=client)
        target = list(initial)
        target[0] = 0.1
        forces = [info[j][10] for j in joints]
        require(all(force > 0 for force in forces), 'Missing effort limits')
        p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL, targetPositions=target,
                                   forces=forces, physicsClientId=client)
        cube_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05]*3, physicsClientId=client)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=cube_shape, basePosition=[2, 0, 0.05], physicsClientId=client)
        for _ in range(480):
            p.stepSimulation(physicsClientId=client)
        final = np.array([p.getJointState(robot, j, physicsClientId=client)[0] for j in joints])
        require(np.isfinite(final).all(), 'Nonfinite joint state')
        error = float(np.max(np.abs(final - target)))
        require(error < 0.02, f'PyBullet internal position servo smoke error: {error}')
        view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0.5], 2, 45, -25, 0, 2)
        projection = p.computeProjectionMatrixFOV(60, 1, 0.1, 10)
        picture = p.getCameraImage(128, 128, view, projection, renderer=p.ER_TINY_RENDERER, physicsClientId=client)
        require(np.asarray(picture[2]).size == 128*128*4, 'Renderer size mismatch')
        pixels = int(np.count_nonzero(np.asarray(picture[4]) == robot))
        require(pixels > 0, 'UR5 not visible to renderer')
        return {'connection': 'DIRECT', 'movable_joints': 6, 'steps': 480, 'dt_s': 1/240,
                'max_internal_servo_error_rad': error, 'rendered_robot_pixels': pixels,
                'api_version': p.getAPIVersion(), 'own_controller_tested': False}
    finally:
        p.disconnect(client)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--versions-only', action='store_true')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    report = {'status': 'failed', 'checks': {}}
    try:
        report['checks']['versions'] = versions()
        if not args.versions_only:
            report['checks']['model'] = model_check()
            report['checks']['c_abi'] = c_abi()
            report['checks']['pybullet'] = bullet_smoke()
        report['status'] = 'passed'
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        data = json.dumps(report, indent=2)
        print(data)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(data + '\n')
    return 0 if report['status'] == 'passed' else 1

if __name__ == '__main__':
    sys.exit(main())
