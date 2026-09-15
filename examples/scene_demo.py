"""UR5 scene demo: --headless for reproducible evidence, --gui for keyboard use.
Keys: 1/2 camera presets, J single-joint jog, Q/Escape save and quit.
"""
import argparse
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import platform
import struct
import sys
import time
import zlib
import numpy as np
import pybullet as p

ROOT = Path(__file__).resolve().parents[1]
PRESETS = {
    'front': dict(distance=1.8, yaw=40, pitch=-25, target=[0.1, 0, 0.5]),
    'side': dict(distance=1.8, yaw=130, pitch=-25, target=[0.1, 0, 0.5]),
}
OBJECT_POSITION = [0.45, 0.25, 0.12]

def require(ok, message):
    if not ok:
        raise RuntimeError(message)

def png(path, rgba):
    """Write RGBA PNG with standard library, keeping dependency lock unchanged."""
    height, width, _ = rgba.shape
    def chunk(tag, data):
        return struct.pack('!I', len(data)) + tag + data + struct.pack('!I', zlib.crc32(tag + data) & 0xffffffff)
    rows = b''.join(b'\0' + row.tobytes() for row in rgba.astype(np.uint8))
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 6, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))

def create_scene(client):
    lock = json.loads((ROOT / 'environment.lock.json').read_text())
    robot = p.loadURDF(str(ROOT / lock['robot']['urdf']), useFixedBase=True,
                       flags=p.URDF_USE_INERTIA_FROM_FILE, physicsClientId=client)
    infos = [p.getJointInfo(robot, i, physicsClientId=client) for i in range(p.getNumJoints(robot, physicsClientId=client))]
    mapping = {i[1].decode(): i[0] for i in infos if i[2] != p.JOINT_FIXED}
    require(set(mapping) == set(lock['robot']['joint_order']), 'Unexpected movable joints')
    joints = [mapping[name] for name in lock['robot']['joint_order']]
    target = [0., -np.pi/2, 0., -np.pi/2, 0., 0.]
    for joint, q in zip(joints, target):
        p.resetJointState(robot, joint, q, physicsClientId=client)
    forces = [infos[j][10] for j in joints]
    p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL, targetPositions=target, forces=forces, physicsClientId=client)
    shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.08]*3, physicsClientId=client)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.08]*3, rgbaColor=[0.95, 0.25, 0.05, 1], physicsClientId=client)
    require(shape >= 0 and visual >= 0, 'Object shape creation failed')
    obj = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, baseVisualShapeIndex=visual,
                           basePosition=OBJECT_POSITION, physicsClientId=client)
    require(obj >= 0 and obj != robot, 'Object body creation failed')
    ids = [p.getBodyUniqueId(i, physicsClientId=client) for i in range(p.getNumBodies(physicsClientId=client))]
    require(obj in ids, 'Object not registered in physics world')
    return robot, obj, joints, target, forces

def object_check(client, obj):
    pos, quat = p.getBasePositionAndOrientation(obj, physicsClientId=client)
    error = float(np.max(np.abs(np.array(pos) - OBJECT_POSITION)))
    require(error <= 1e-9, 'Object position mismatch')
    require(np.max(np.abs(np.array(quat)-[0, 0, 0, 1])) <= 1e-9, 'Object orientation mismatch')
    return {'body_id': obj, 'expected_position_m': OBJECT_POSITION, 'actual_position_m': list(pos),
            'max_position_error_m': error, 'position_tolerance_m': 1e-9, 'creation_passed': True}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--headless', action='store_true')
    mode.add_argument('--gui', action='store_true')
    parser.add_argument('--output', type=Path, default=ROOT / 'results/scene-demo')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = {'status': 'failed', 'execution_mode': 'GUI' if args.gui else 'DIRECT',
              'host': platform.platform(), 'python': platform.python_version(),
              'utc': datetime.now(timezone.utc).isoformat(),
              'pybullet': version('pybullet'), 'numpy': np.__version__,
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'model_sha256': hashlib.sha256((ROOT / 'models/ur5/ur5.urdf').read_bytes()).hexdigest(),
              'user_windows_validation': 'not_established_by_this_run',
              'gui_keyboard_verified': False, 'presets': PRESETS, 'events': [], 'captures': []}
    client = -1
    try:
        lock = json.loads((ROOT / 'environment.lock.json').read_text())
        require(platform.python_version() == lock['python'], 'Python version differs from lock')
        require(version('pybullet') == lock['packages']['pybullet'] and np.__version__ == lock['packages']['numpy'], 'Package version differs from lock')
        client = p.connect(p.GUI if args.gui else p.DIRECT)
        require(client >= 0, 'PyBullet connection failed')
        p.setGravity(0, 0, -9.81, physicsClientId=client)
        p.setTimeStep(1/240, physicsClientId=client)
        robot, obj, joints, target, forces = create_scene(client)
        report['object_before'] = object_check(client, obj)
        if args.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client)
        frames = {}
        def capture(name, trigger):
            preset = PRESETS[name]
            if args.gui:
                p.resetDebugVisualizerCamera(preset['distance'], preset['yaw'], preset['pitch'], preset['target'], physicsClientId=client)
                camera = p.getDebugVisualizerCamera(physicsClientId=client)
                view, projection = camera[2], camera[3]
            else:
                view = p.computeViewMatrixFromYawPitchRoll(preset['target'], preset['distance'], preset['yaw'], preset['pitch'], 0, 2)
                projection = p.computeProjectionMatrixFOV(60, 4/3, 0.05, 10)
            image = p.getCameraImage(640, 480, view, projection, renderer=p.ER_TINY_RENDERER, physicsClientId=client)
            rgba = np.asarray(image[2], dtype=np.uint8).reshape(480, 640, 4)
            mask = np.asarray(image[4]).reshape(480, 640)
            robot_pixels, object_pixels = int(np.count_nonzero(mask == robot)), int(np.count_nonzero(mask == obj))
            require(robot_pixels >= 20 and object_pixels >= 20, f'Invisible robot/object in {name}: {robot_pixels}/{object_pixels}')
            filename = f'{len(report["captures"]):02d}-{name}.png'
            path = args.output / filename
            png(path, rgba)
            frames[name] = rgba.copy()
            report['captures'].append({'preset': name, 'trigger': trigger, 'file': filename,
                                       'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                       'robot_pixels': robot_pixels, 'object_pixels': object_pixels,
                                       'visible_pixel_threshold': 20, 'object': object_check(client, obj)})
            report['events'].append({'action': 'switch_view', 'preset': name, 'trigger': trigger})
        def jog():
            target[0] = 0.1 if target[0] == 0 else 0.
            p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL, targetPositions=target, forces=forces, physicsClientId=client)
            for _ in range(480):
                p.stepSimulation(physicsClientId=client)
                if args.gui:
                    time.sleep(1/240)
            final = [p.getJointState(robot, j, physicsClientId=client)[0] for j in joints]
            error = float(np.max(np.abs(np.array(final)-target)))
            require(error < 0.02, 'Single-joint jog failed')
            report['events'].append({'action': 'jog', 'target_j1_rad': target[0], 'max_error_rad': error})
        capture('front', 'initial')
        if args.headless:
            jog()
            capture('side', 'automatic')
            capture('front', 'automatic')
        else:
            print('Focus PyBullet window: 1=front, 2=side, J=jog, Q/Escape=save and quit', flush=True)
            pressed = set()
            while p.isConnected(client):
                keys = p.getKeyboardEvents(physicsClientId=client)
                for key, name in ((ord('1'), 'front'), (ord('2'), 'side')):
                    if keys.get(key, 0) & p.KEY_WAS_TRIGGERED:
                        capture(name, 'keyboard')
                        pressed.add(name)
                if keys.get(ord('j'), 0) & p.KEY_WAS_TRIGGERED:
                    jog()
                if any(keys.get(key, 0) & p.KEY_WAS_TRIGGERED for key in (ord('q'), 27)):
                    break
                p.stepSimulation(physicsClientId=client)
                time.sleep(1/240)
            report['gui_keyboard_verified'] = pressed == set(PRESETS)
            require(report['gui_keyboard_verified'], 'Must switch both presets using 1 and 2 before quitting')
        require(set(frames) == set(PRESETS), 'Missing camera preset')
        report['view_mean_abs_rgb_difference'] = float(np.mean(np.abs(frames['front'][:, :, :3].astype(float)-frames['side'][:, :, :3])))
        require(report['view_mean_abs_rgb_difference'] > 0.5, 'Two views are not visibly distinct')
        report['object_after'] = object_check(client, obj)
        report['status'] = 'passed'
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if client >= 0 and p.isConnected(client):
            p.disconnect(client)
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'passed' else 1

if __name__ == '__main__':
    sys.exit(main())
