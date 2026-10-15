"""Exercise C PID contracts and save three reproducible closed-loop baselines."""
import argparse
import csv
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from control.pid import PID, Config, State, Sample


def sha(path):
    """Return SHA256 of exact on-disk bytes for trace provenance."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def contracts(library, config):
    """Assert recurrence against positional PID and independent negative cases."""
    checks = []

    def check(name, condition):
        """Record and assert one actual observation, raising on failure."""
        checks.append({'name': name, 'passed': bool(condition)})
        if not condition:
            raise AssertionError(name)

    p = PID(library, dict(config, kp=2, ki=0.5, kd=0.01,
                          output_min=-100, output_max=100))
    integral, old = 0.0, 0.0
    for k in range(200):
        error = 0.05 * math.sin(0.13*k)
        integral += 0.5*config['dt_s']*error
        expected = 2*error + integral + 0.01/config['dt_s']*(error-old)
        result = p.step(error, 0)
        check('positional_equivalence_' + str(k), result['code'] == 0 and
              abs(result['data']['output']-expected) < 1e-11)
        old = error
    p = PID(library, dict(config, kp=0, ki=1, kd=0,
                          integral_min=-0.1, integral_max=0.1))
    for sign in (1, -1):
        for _ in range(100):
            result = p.step(sign, 0)
        check('integral_clamp_' + str(sign),
              abs(result['data']['integral']-sign*0.1) < 1e-12 and
              result['data']['integral_limited'])
    p = PID(library, dict(config, kp=2, ki=1, kd=0))
    for sign in (1, -1):
        p.reset()
        for _ in range(1000):
            result = p.step(sign*100, 0)
        check('saturation_antiwindup_' + str(sign),
              result['data']['output'] == sign and
              result['data']['integral'] == 0 and
              result['data']['output_limited'])
        check('saturation_release_' + str(sign),
              abs(p.step(0, 0)['data']['output']) < 1e-12)
    p.step(0.1, 0)
    check('reset_success', p.reset() == 0)
    check('reset_all_history', all(getattr(p.state, n) == 0 for n in
          ('error_1', 'error_2', 'integral', 'raw_output', 'output')))
    check('repeat_after_reset', p.step(0.1, 0) == PID(library, dict(
          config, kp=2, ki=1, kd=0)).step(0.1, 0))
    for label, reference, actual in [('nan', math.nan, 0), ('inf', 0, math.inf),
                                     ('overflow', 1e308, -1e308),
                                     ('bool', True, 0), ('string', '0', 0)]:
        before = bytes(p.state)
        result = p.step(reference, actual)
        check('invalid_' + label, result == {'code': 1001, 'data': None} and
              bytes(p.state) == before)
    for field, value, expected in [('dt_s', 0, 1006), ('dt_s', math.inf, 1006),
                                   ('kp', -1, 1001), ('ki', math.nan, 1001),
                                   ('kd', -1, 1001), ('integral_min', 1, 1001),
                                   ('output_max', -2, 1001)]:
        bad = Config(**dict(config, **{field: value}))
        before = bytes(p.state)
        check('bad_config_' + field + str(value),
              p.lib.robot_pid_reset(C.byref(bad), C.byref(p.state)) == expected
              and bytes(p.state) == before)
    out = Sample(7, 7, 7, 7, 7, 7)
    before_out = bytes(out)
    check('null_state', p.lib.robot_pid_step(None, 0, 0, C.byref(out)) == 1001
          and bytes(out) == before_out)
    before = bytes(p.state)
    check('null_result', p.lib.robot_pid_step(C.byref(p.state), 0, 0, None) == 1001
          and bytes(p.state) == before)
    check('null_reset_config', p.lib.robot_pid_reset(None, C.byref(p.state)) == 1001
          and bytes(p.state) == before)
    check('null_reset_state', p.lib.robot_pid_reset(C.byref(p.config), None) == 1001)
    for label, field, value in [('uninitialized', 'initialized', 0),
                              ('nan_history', 'error_1', math.nan),
                              ('integral_outside', 'integral', 100),
                              ('output_outside', 'output', 100)]:
        p.reset()
        setattr(p.state, field, value)
        before = bytes(p.state)
        check(label, p.lib.robot_pid_step(C.byref(p.state), 0, 0, C.byref(out)) == 4001
              and bytes(p.state) == before and bytes(out) == before_out)
    p.reset()
    before = bytes(p.state)
    check('invalid_direct_c_output_untouched',
          p.lib.robot_pid_step(C.byref(p.state), math.nan, 0, C.byref(out)) == 1001
          and bytes(p.state) == before and bytes(out) == before_out)
    return checks


class Plant:
    """Integrator or UR5 J1 torque experiment; all external I/O is here."""
    def __init__(self, kind, config, initial):
        """Build/reset an isolated plant from explicit frozen simulation config."""
        self.kind, self.config, self.q, self.v = kind, config, initial, 0.0
        self.client = None
        if kind == 'pybullet':
            import pybullet as pb
            self.pb = pb
            self.client = pb.connect(pb.DIRECT)
            if self.client < 0:
                raise RuntimeError('PyBullet DIRECT connection failed')
            pb.setGravity(*config['pybullet']['gravity_m_s2'], physicsClientId=self.client)
            self.substeps = config['pybullet']['physics_substeps']
            pb.setTimeStep(config['dt_s']/self.substeps, physicsClientId=self.client)
            self.robot = pb.loadURDF(str(ROOT/'models/ur5/ur5.urdf'),
                                    useFixedBase=True, physicsClientId=self.client)
            movable = [j for j in range(pb.getNumJoints(self.robot, self.client))
                       if pb.getJointInfo(self.robot, j, self.client)[2] == pb.JOINT_REVOLUTE]
            if len(movable) != 6:
                raise RuntimeError('UR5 must have six revolute joints')
            self.joint = movable[0]
            if pb.getJointInfo(self.robot, self.joint, self.client)[1] != b'shoulder_pan_joint':
                raise RuntimeError('Unexpected J1 order')
            pose = [initial, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
            for j, q in zip(movable, pose):
                pb.resetJointState(self.robot, j, q, physicsClientId=self.client)
                pb.setJointMotorControl2(self.robot, j, pb.POSITION_CONTROL,
                    targetPosition=q, force=config['pybullet']['other_joint_force_nm'],
                    physicsClientId=self.client)
            pb.setJointMotorControl2(self.robot, self.joint, pb.VELOCITY_CONTROL,
                                    force=0, physicsClientId=self.client)

    def read(self):
        """Return current measured joint angle and angular velocity in SI units."""
        if self.kind == 'pybullet':
            self.q, self.v = self.pb.getJointState(self.robot, self.joint, self.client)[:2]
        return self.q, self.v

    def advance(self, output):
        """Hold output for dt; integrator uses velocity, PyBullet uses J1 torque."""
        if self.kind == 'integrator':
            self.v = output
            self.q += self.config['dt_s']*output
        else:
            for _ in range(self.substeps):
                self.pb.setJointMotorControl2(self.robot, self.joint, self.pb.TORQUE_CONTROL,
                                             force=output, physicsClientId=self.client)
                self.pb.stepSimulation(physicsClientId=self.client)

    def close(self):
        """Disconnect the exact owned client if present."""
        if self.client is not None:
            self.pb.disconnect(self.client)


def reference(name, case, t):
    """Frozen reference in rad: step after .5s, one sine then hold, or constant."""
    if name == 'sine':
        return (case['amplitude_rad']*math.sin(2*math.pi*case['frequency_hz']*t)
                if t < case['stop_s'] else 0.0)
    if name == 'step' and t < case['start_s']:
        return 0.0
    return case['target_rad']


def statistics(rows, name, case, dt):
    """Compute all-sample RMSE/max, last-second bias, overshoot and settling."""
    errors = [r['error_rad'] for r in rows]
    start = case.get('start_s', case.get('stop_s', 0))
    target = case.get('target_rad', 0)
    amplitude = abs(target-case['initial_rad'])
    band = max(0.001, 0.02*amplitude)
    tail_bad = [r['time_s'] for r in rows if r['time_s'] >= start and
                abs(r['actual_rad']-target) > band]
    settled = None if tail_bad and tail_bad[-1] >= rows[-1]['time_s']-dt else (
        (tail_bad[-1]+dt-start) if tail_bad else 0.0)
    active = [r for r in rows if r['time_s'] >= start]
    overshoot = (max(0, max(r['actual_rad']-target for r in active))/amplitude*100
                 if name != 'sine' and amplitude else None)
    crossings = [next((r['time_s'] for r in active if
                 r['actual_rad'] >= case['initial_rad']+fraction*amplitude), None)
                 for fraction in (0.1, 0.9)] if name != 'sine' else [None, None]
    return {'sample_count': len(rows), 'rmse_rad': math.sqrt(sum(e*e for e in errors)/len(errors)),
            'max_abs_error_rad': max(map(abs, errors)), 'final_error_rad': errors[-1],
            'last_second_mean_error_rad': sum(errors[-round(1/dt):])/round(1/dt),
            'overshoot_percent': overshoot,
            'rise_10_90_s': crossings[1]-crossings[0] if None not in crossings else None,
            'settling_time_s': settled, 'settling_band_rad': band,
            'output_limited_count': sum(r['output_limited'] for r in rows),
            'integral_limited_count': sum(r['integral_limited'] for r in rows)}


def plot(rows, destination, unit):
    """Render actual sampled reference/feedback/output into a two-panel SVG."""
    from html import escape
    width, height = 900, 500
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
             '<rect width="100%" height="100%" fill="white"/>']
    for top, keys, labels, colors, title in [
            (35, ['reference_rad', 'actual_rad'], ['reference', 'actual'], ['#64748b', '#2563eb'], 'Position (rad)'),
            (275, ['output'], ['limited output'], ['#b45309'], 'Output ('+unit+')')]:
        values = [r[key] for r in rows for key in keys]
        lo, hi = min(values), max(values)
        span = hi-lo or 1
        lo, hi = lo-0.1*span, hi+0.1*span
        for j in range(5):
            y = top+j*42
            value = hi-(hi-lo)*j/4
            parts.append(f'<path d="M75 {y} H870" stroke="#e2e8f0"/><text x="5" y="{y+4}" font-size="12">{value:.3f}</text>')
        parts.append(f'<text x="75" y="{top-12}" font-size="14">{escape(title)}</text>')
        for key, label, color in zip(keys, labels, colors):
            points = ' '.join(f"{75+795*r['time_s']/rows[-1]['time_s']:.2f},{top+168*(hi-r[key])/(hi-lo):.2f}" for r in rows)
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.5"/>')
            parts.append(f'<text x="{600+keys.index(key)*130}" y="{top-12}" fill="{color}" font-size="13">{label}</text>')
    for j in range(9):
        parts.append(f'<text x="{75+795*j/8}" y="470" font-size="12">{rows[-1]["time_s"]*j/8:.1f}</text>')
    parts.append('<text x="390" y="495" font-size="14">Simulation time (s)</text></svg>')
    destination.write_text(''.join(parts), encoding='utf-8')



def main():
    """Run every selected baseline; save failed reports and return nonzero on error."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=ROOT/'config/single-joint-pid.v1.json')
    parser.add_argument('--plant', choices=['integrator', 'pybullet'], default='integrator')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = {'status': 'failed', 'plant': args.plant, 'cases': [],
              'environment': {'python': sys.version, 'platform': platform.platform()},
              'scope': 'Single joint functional baseline; not full CTRL-01/02/03/04 acceptance'}
    try:
        config = json.loads(args.config.read_text(encoding='utf-8'))
        report.update(config=config, config_sha256=sha(args.config),
                      library_sha256=sha(args.library),
                      source_sha256=sha(ROOT/'src/control/pid.c'))
        report['contracts'] = contracts(args.library, config['integrator']['pid'])
        if args.plant == 'pybullet':
            import importlib.metadata
            report['environment']['pybullet'] = importlib.metadata.version('pybullet')
            report['model_sha256'] = sha(ROOT/'models/ur5/ur5.urdf')
        dt = config['dt_s']
        count = round(config['duration_s']/dt)
        assert dt == config[args.plant]['pid']['dt_s']
        for name, case in config['cases'].items():
            pid = PID(args.library, config[args.plant]['pid'])
            rows = []
            plant = Plant(args.plant, config, case['initial_rad'])
            try:
                for k in range(count):
                    t = k*dt
                    actual, velocity = plant.read()
                    ref = reference(name, case, t)
                    result = pid.step(ref, actual)
                    if result['code']:
                        raise RuntimeError('PID failed: ' + str(result))
                    data = result['data']
                    rows.append({'node': k, 'time_s': t, 'reference_rad': ref,
                        'actual_rad': actual, 'velocity_rad_s': velocity,
                        'error_rad': data['error'], 'raw_output': data['raw_output'],
                        'output': data['output'], 'integral': data['integral'],
                        'integral_limited': data['integral_limited'],
                        'output_limited': data['output_limited'], 'code': 0})
                    plant.advance(data['output'])
            finally:
                plant.close()
                if rows:
                    path = args.output/(name+'.csv')
                    with path.open('w', newline='', encoding='utf-8') as stream:
                        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                        writer.writeheader()
                        writer.writerows(rows)
            stats = statistics(rows, name, case, dt)
            gates = config['functional_gates']
            cfg = config[args.plant]['pid']
            passed = (len(rows) == count and all(
                all(math.isfinite(v) for v in r.values()) and
                abs(r['actual_rad']) <= gates['maximum_abs_position_rad'] and
                cfg['output_min'] <= r['output'] <= cfg['output_max'] and
                cfg['integral_min'] <= r['integral'] <= cfg['integral_max']
                for r in rows) and abs(stats['final_error_rad']) <= gates['final_error_rad'])
            report['cases'].append({'case': name, 'status': 'passed' if passed else 'failed',
                                    'statistics': stats, 'csv_sha256': sha(path)})
            plot(rows, args.output/(name+'.svg'), config[args.plant]['output_unit'])
        report['status'] = 'passed' if all(c['status'] == 'passed' for c in report['cases']) else 'failed'
    except Exception as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
        traceback.print_exc()
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'status': report['status'], 'plant': args.plant, 'cases': report['cases']}))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
