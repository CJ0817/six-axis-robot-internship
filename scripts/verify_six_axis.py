"""Plan -> continuous IK -> certification -> interpolation -> six C PIDs -> UR5 feedback."""
import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback
from importlib.metadata import version
import numpy as np
import pybullet as p
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from adapters.c_kinematics import ContractError
from planning.cartesian import CartesianPlanner
from planning.collision import StaticScene
from control.six_axis import SixAxisExecutor, TickClock, interpolate, load_execution_config


class SixAxisPlant:
    """Independent plant client; collision certification can never teleport it."""
    def __init__(self, config, profile, fault=None):
        self.config, self.fault, self.tick = config, fault, 0
        self.dt_s = config['dt_s']
        self.commands, self.stopped = 0, False
        self.client = p.connect(p.DIRECT)
        try:
            self.robot = p.loadURDF(str(ROOT/'models/ur5/ur5.urdf'), useFixedBase=True,
                                    physicsClientId=self.client)
            mapping = {p.getJointInfo(self.robot, i, physicsClientId=self.client)[1].decode(): i
                       for i in range(p.getNumJoints(self.robot, physicsClientId=self.client))}
            self.joints = [mapping[n] for n in profile['urdf_joint_names']]
            p.setGravity(*config['gravity_m_s2'], physicsClientId=self.client)
            p.setTimeStep(config['dt_s']/config['physics_substeps'], physicsClientId=self.client)
            p.setPhysicsEngineParameter(numSolverIterations=100, deterministicOverlappingPairs=1,
                                        physicsClientId=self.client)
            for joint, q in zip(self.joints, config['initial_q_rad']):
                p.resetJointState(self.robot, joint, q, physicsClientId=self.client)
            self.stop()
            self.stopped = False
        except Exception:
            self.close()
            raise

    def read(self):
        states = p.getJointStates(self.robot, self.joints, physicsClientId=self.client)
        feedback = dict(tick=self.tick, time_s=self.tick*self.config['dt_s'],
                        q_rad=[s[0] for s in states], qd_rad_s=[s[1] for s in states])
        if self.tick == 5:
            if self.fault == 'stale_feedback': feedback['tick'] -= 1
            if self.fault == 'wrong_feedback_time': feedback['time_s'] += .003
            if self.fault == 'nonfinite_feedback': feedback['q_rad'][2] = math.nan
            if self.fault == 'missing_feedback': feedback.pop('qd_rad_s')
            if self.fault == 'feedback_overspeed': feedback['qd_rad_s'][3] = .2
            if self.fault == 'tracking_error': feedback['q_rad'][0] += .1
        return feedback

    def advance(self, velocity):
        if self.stopped: raise RuntimeError('Motion command after stop')
        p.setJointMotorControlArray(self.robot, self.joints, p.VELOCITY_CONTROL,
            targetVelocities=velocity, forces=self.config['motor_forces_nm'], physicsClientId=self.client)
        self.commands += 1
        for _ in range(self.config['physics_substeps']):
            p.stepSimulation(physicsClientId=self.client)
        self.tick += 1

    def stop(self):
        p.setJointMotorControlArray(self.robot, self.joints, p.VELOCITY_CONTROL,
            targetVelocities=[0.]*6, forces=self.config['motor_forces_nm'], physicsClientId=self.client)
        self.stopped = True

    def close(self):
        if p.isConnected(self.client): p.disconnect(self.client)


class FaultClock(TickClock):
    def __init__(self, dt, delta):
        super().__init__(dt)
        self.delta = delta

    def stamp(self):
        if self.tick == 5:
            return self.tick+self.delta, (self.tick+self.delta)*self.dt
        return super().stamp()


def write_trace(path, rows):
    flat = []
    for row in rows:
        item = {}
        for key, value in row.items():
            if isinstance(value, list):
                item.update({key+'_J'+str(i+1): v for i, v in enumerate(value)})
            else: item[key] = value
        flat.append(item)
    if flat:
        with path.open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=flat[0].keys())
            writer.writeheader(); writer.writerows(flat)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, default=ROOT/'build/librobot_contract.so')
    parser.add_argument('--planning-library', type=Path, default=ROOT/'build/librobot_planning.so')
    parser.add_argument('--control-library', type=Path, default=ROOT/'build/librobot_control.so')
    parser.add_argument('--output', type=Path, default=ROOT/'results/six-axis-integration')
    args = parser.parse_args(); out = args.output; out.mkdir(parents=True, exist_ok=True)
    config_path = ROOT/'config/six-axis-execution.v1.json'
    report = dict(usage='simulation_only', engine='UR5 / PyBullet '+version('pybullet'),
        clock='one configured planning/IK/interpolation/control/feedback period; integer simulation tick, not wall deadline',
        control='six C position PID outer loops + qd feedforward -> bounded velocity -> PyBullet velocity inner servos',
        trials=[], passed=False)
    scene = None
    try:
        config = load_execution_config(config_path)
        report['parameters'] = config
        planner = CartesianPlanner(args.library, args.planning_library,
                                   config_path=config['planning_config_path']); planner.setup()
        scene = StaticScene(json.loads((ROOT/'scenes/static-apf.v1.json').read_text()), planner.profile)
        start = planner.selector._fk(planner.model, config['initial_q_rad'])
        goal = start.copy(); goal[:3, 3] += config['goal_translation_m']
        angle = config['goal_world_z_rotation_rad']; c, s = math.cos(angle), math.sin(angle)
        goal[:3, :3] = np.array([[c,-s,0],[s,c,0],[0,0,1]]) @ start[:3, :3]
        planned = planner.plan(start.tolist(), goal.tolist(), config['initial_q_rad'], scene,
                               duration_s=config['trajectory_duration_s'])
        if planned['code']: raise ContractError(planned['code'], planned['message'])
        data = planned['data']
        report['planning'] = dict(code=0, status=data['trajectory_status'],
            trajectory_sha256=hashlib.sha256(json.dumps({k:data[k] for k in
                ('coefficients_rad','segment_times_s')}, sort_keys=True).encode()).hexdigest(),
            ik_nodes=len(data['ik_nodes']), ik_failed=sum(x['code'] != 0 for x in data['ik_nodes']),
            sample_period_s=data['sample_period_s'], duration_s=data['segment_times_s'][-1],
            joint_excursion_rad=np.ptp(np.array(data['q_rad']), axis=0).tolist(),
            continuous_collision_proof=data['continuous_collision_proof'],
            continuous_tcp_proof=data['continuous_tcp_proof'])
        report['cycle_consistency'] = dict(planning_s=planner.config['sample_period_s'],
            ik_s=planner.selector.config.dt_s, interpolation_s=config['dt_s'],
            control_s=config['pid']['dt_s'], feedback_s=config['dt_s'])
        cases = [('normal-r'+str(i+1), 0) for i in range(config['repeats'])]
        cases += [(n, 1006) for n in ('duplicate_tick','skipped_tick','reversed_tick','stale_feedback','wrong_feedback_time')]
        cases += [(n, 4001) for n in ('nonfinite_feedback','missing_feedback','feedback_overspeed','tracking_error','reference_jump','controller_invalid','controller_jump')]
        cases += [('collision_feedback',3003), ('uncertified',3001), ('tampered',3001)]
        cases += [('cycle_mismatch',1006), ('feedback_cycle_mismatch',1006)]
        for name, expected in cases:
            plant = SixAxisPlant(config, planner.profile, name)
            try:
                delta = {'duplicate_tick':-1, 'skipped_tick':1, 'reversed_tick':-2}.get(name)
                clock = FaultClock(config['dt_s'], delta) if delta is not None else TickClock(config['dt_s'])
                execution_config = copy.deepcopy(config)
                if name == 'cycle_mismatch':
                    execution_config['dt_s'] /= 2
                    execution_config['pid']['dt_s'] = execution_config['dt_s']
                    plant.dt_s = execution_config['dt_s']
                    clock = TickClock(execution_config['dt_s'])
                if name == 'feedback_cycle_mismatch': plant.dt_s /= 2
                executor = SixAxisExecutor(args.control_library, execution_config, plant, scene, planner.profile, clock)
                if name == 'collision_feedback':
                    class CollisionFault:
                        def metadata(self): return scene.metadata()
                        def check(self, q):
                            result = scene.check(q)
                            if plant.tick == 5: result.update(passed=False, minimum_distance_lower_bound_m=-.001)
                            return result
                    executor.scene = CollisionFault()
                if name == 'reference_jump':
                    def jump(d,t):
                        value = interpolate(d,t)
                        if abs(t-5*config['dt_s'])<1e-10: value[0][0] += .02
                        return value
                    executor.sampler = jump
                if name.startswith('controller_'):
                    original = executor.controllers[0].step
                    def bad_step(*values):
                        value = original(*values)
                        if plant.tick == 5:
                            value['data']['output'] = math.nan if name == 'controller_invalid' else .03
                        return value
                    executor.controllers[0].step = bad_step
                attempt = planned
                if name in ('uncertified', 'tampered'):
                    attempt = copy.deepcopy(planned)
                    if name == 'uncertified': attempt['data']['trajectory_status'] = 'candidate'
                    else: attempt['data']['coefficients_rad'][0][0][0] += .001
                result = executor.run(attempt)
                success = result['code'] == expected and result['data'] is None
                trial = dict(name=name, expected_code=expected, actual_code=result['code'],
                    message=result['message'], sample_count=len(executor.rows), motion_commands=plant.commands,
                    stopped=plant.stopped, events=executor.events)
                if expected:
                    preflight = name in ('uncertified','tampered','cycle_mismatch','feedback_cycle_mismatch')
                    success &= bool(executor.events) and plant.commands == (0 if preflight else 5)
                    success &= expected==3001 or plant.stopped
                    event = executor.events[0]
                    if name == 'controller_invalid':
                        success &= event['observed_command'][0] == {'nonfinite':'NaN'}
                        success &= event['raw_control_outputs'][0]['data']['output'] == {'nonfinite':'NaN'}
                        success &= event['validated_command'] is None and event['previous_sample']['tick']==4
                    if name in ('duplicate_tick','skipped_tick','reversed_tick'):
                        success &= all(event[k] is None for k in ('observed_feedback','observed_reference',
                            'raw_control_outputs','observed_command','validated_command'))
                        success &= event['previous_sample']['tick']==4
                    if preflight:
                        success &= event['previous_sample'] is None
                        success &= all(event[k] is None for k in ('observed_clock','observed_feedback',
                            'observed_reference','raw_control_outputs','observed_command','validated_command'))
                else:
                    rows=executor.rows
                    q=np.array([r['q_rad'] for r in rows]); ref=np.array([r['reference_rad'] for r in rows])
                    command=np.array([r['command_rad_s'] for r in rows]); vel=np.array([r['qd_rad_s'] for r in rows])
                    trial.update(rmse_rad=np.sqrt(np.mean((q-ref)**2, axis=0)).tolist(),
                        final_error_rad=(ref[-1]-q[-1]).tolist(),
                        actual_excursion_rad=np.ptp(q,axis=0).tolist(),
                        peak_actual_velocity_rad_s=np.max(abs(vel),axis=0).tolist(),
                        peak_command_velocity_rad_s=np.max(abs(command),axis=0).tolist(),
                        peak_command_delta_rad_s=np.max(abs(np.diff(command,axis=0)),axis=0).tolist(),
                        peak_reference_delta_rad=np.max(abs(np.diff(ref,axis=0)),axis=0).tolist(),
                        minimum_clearance_m=min(r['clearance_m'] for r in rows),
                        wall_processing_peak_s=max(r['processing_wall_s'] for r in rows),
                        wall_processing_over_period_count=sum(r['processing_wall_s']>config['dt_s'] for r in rows),
                        timing_error_count=0, command_jump_count=0, control_anomaly_count=len(executor.events))
                    success &= not executor.events and np.all(np.ptp(q,axis=0)>1e-5) and plant.stopped
                    success &= np.max(abs(ref[-1]-q[-1])) < .001
                trial['passed'] = bool(success)
                report['trials'].append(trial)
                write_trace(out/(name+'.csv'), executor.rows)
                (out/(name+'-events.json')).write_text(json.dumps(executor.events,indent=2,allow_nan=False)+'\n')
                print(name, result['code'], success, flush=True)
            finally: plant.close()
        try:
            SixAxisExecutor(args.control_library, {}, None, scene, planner.profile)
            missing = False
        except ContractError as exc: missing = exc.code == 1004
        report['missing_config_rejected'] = missing
        report['passed'] = all(t['passed'] for t in report['trials']) and missing
    except Exception as exc:
        report['failure'] = dict(message=str(exc), traceback=traceback.format_exc())
        print(report['failure']['traceback'], flush=True)
    finally:
        if scene: scene.close()
        report['source_sha256'] = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [config_path, ROOT/'config/cartesian-planning.v1.json', Path(__file__).resolve(), ROOT/'src/control/six_axis.py',
                         args.library, args.planning_library, args.control_library] if path.is_file()}
        (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__': sys.exit(main())
