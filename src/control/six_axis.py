"""Certified six-axis simulation consumer, driven by one integer-tick clock.

Outer C PID produces rad/s with reference velocity feedforward. The plant owns
the inner velocity servo. This module is not a hardware or real-time driver.
"""
import copy
import json
import math
from pathlib import Path
import time
import numpy as np
from adapters.c_kinematics import ContractError
from planning.execution import execute_trajectory, failure
from control.pid import FeedforwardPID

ROOT = Path(__file__).resolve().parents[2]


def load_execution_config(path=ROOT/'config/six-axis-execution.v1.json',
                          planning_path=None):
    """Resolve the planner's single period into control, feedback and PID config.

    Missing files return 1004; independently supplied conflicting periods return
    1006. The planner must use the returned planning_config_path as well.
    """
    from planning.cartesian import configuration
    try:
        config = json.loads(Path(path).read_text())
        if planning_path is None:
            planning_path = Path(path).resolve().parent/config['planning_config']
        period = configuration(planning_path)['sample_period_s']
        pid = config['pid']
    except (OSError, KeyError) as exc:
        raise ContractError(1004, 'Missing execution/cycle configuration') from exc
    for value in (config.get('dt_s', period), pid.get('dt_s', period)):
        if type(value) not in (int, float) or value != period:
            raise ContractError(1006, 'Execution/PID period differs from planning/IK period')
    config['dt_s'] = pid['dt_s'] = period
    config['planning_config_path'] = str(Path(planning_path).resolve())
    return config


def check_cycle(data, dt):
    """Reject unequal certified planning/IK, interpolation and execution periods."""
    certificate_period = data.get('certification', {}).get('sample_period_s')
    for period in (data.get('sample_period_s', certificate_period), certificate_period):
        if type(period) not in (int, float) or period != dt:
            raise ContractError(1006, 'Certified planning/IK period differs from execution period')
    times = np.asarray(data['segment_times_s'], float)
    samples = np.asarray(data['time_s'], float)
    for grid in (times, samples):
        if grid.ndim != 1 or len(grid) < 2 or not np.all(np.isfinite(grid)) or not np.allclose(
                grid, np.arange(len(grid))*dt, atol=1e-10, rtol=0):
            raise ContractError(1006, 'Planning/interpolation nodes do not follow the execution cycle')
    if len(times) != len(samples):
        raise ContractError(1006, 'Planning/interpolation node counts differ')
    if 'ik_nodes' in data:
        nodes = data['ik_nodes']
        if len(nodes) != len(samples)-1 or any(
                node['node_index'] != i or not math.isfinite(node['time_s']) or
                abs(node['time_s']-i*dt) > 1e-10 for i, node in enumerate(nodes, 1)):
            raise ContractError(1006, 'IK nodes do not follow the execution cycle')


def snapshot(value):
    """Copy observations into strict JSON; preserve invalid numbers explicitly."""
    if isinstance(value, np.ndarray): return snapshot(value.tolist())
    if isinstance(value, np.generic): return snapshot(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite': 'NaN' if math.isnan(value) else ('+Infinity' if value > 0 else '-Infinity')}
    if isinstance(value, dict): return {k: snapshot(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [snapshot(v) for v in value]
    return copy.deepcopy(value)


def empty_observations():
    """No observation from the preceding tick may appear as current data."""
    return dict(observed_clock=None, observed_feedback=None, observed_reference=None,
                raw_control_outputs=None, observed_command=None, validated_command=None)


class TickClock:
    """Only advance after a complete command/physics interval."""
    def __init__(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ContractError(1006, 'Invalid control period')
        self.dt, self.tick = dt, 0

    def stamp(self):
        return self.tick, self.tick * self.dt

    def advance(self):
        self.tick += 1


def interpolate(data, t):
    """Evaluate original certified coefficients; no resampling or refitting."""
    times = np.asarray(data['segment_times_s'], float)
    co = np.asarray(data['coefficients_rad'], float)
    if not math.isfinite(t) or t < 0:
        raise ContractError(1006, 'Invalid interpolation time')
    end = times[-1]
    i = min(max(0, int(np.searchsorted(times, min(t, end), side='right'))-1), len(co)-1)
    dt = times[i+1]-times[i]
    u = (min(t, end)-times[i])/dt
    values = [np.array([np.polynomial.polynomial.polyval(u,
        np.polynomial.polynomial.polyder(axis, m=o))/dt**o for axis in co[i]]) for o in range(3)]
    if t > end:
        values[1:] = [np.zeros(6), np.zeros(6)]
    return values


class SixAxisExecutor:
    """Fail closed, retaining traces and events separately from executable data."""
    def __init__(self, library, config, plant, scene, profile, clock=None):
        required = {'dt_s', 'hold_s', 'speed_limit_rad_s', 'acceleration_limit_rad_s2',
                    'tracking_error_limit_rad', 'feedback_speed_limit_rad_s', 'pid'}
        if not isinstance(config, dict) or not required <= config.keys():
            raise ContractError(1004, 'Incomplete six-axis execution configuration')
        if any(type(config[k]) not in (int, float) or not math.isfinite(config[k]) or config[k] <= 0
               for k in required-{'pid'}):
            raise ContractError(1001, 'Positive finite execution settings required')
        if config['pid']['dt_s'] != config['dt_s'] or config['pid']['output_max'] != config['speed_limit_rad_s'] or config['pid']['output_min'] != -config['speed_limit_rad_s']:
            raise ContractError(1006, 'PID/clock/velocity limits disagree')
        self.config, self.plant, self.scene, self.profile = config, plant, scene, profile
        self.clock = clock or TickClock(config['dt_s'])
        if self.clock.dt != config['dt_s'] or self.clock.tick != 0:
            raise ContractError(1006, 'Clock must start at tick zero with configured period')
        self.controllers = [FeedforwardPID(library, config['pid'], 1., 0.) for _ in range(6)]
        self.rows, self.events = [], []
        self.sampler = interpolate
        self.started = False

    @staticmethod
    def vector(value, label):
        array = np.asarray(value, float)
        if array.shape != (6,) or not np.all(np.isfinite(array)):
            raise ContractError(4001, 'Invalid six-axis '+label)
        return array

    def run(self, result):
        """The only execution entry; authenticate before any motion command."""
        if self.started:
            return failure(ContractError(4001, 'Executor cannot be reused'), 'six_axis_execution')
        self.started = True
        dispatched = execute_trajectory(result, self.scene, self._consume)
        if dispatched['code'] and not self.events:
            self.events.append(dict(stage='execution_gate', code=dispatched['code'], tick=0,
                                    message=dispatched['message'], previous_sample=None,
                                    **empty_observations()))
        # Neither success nor failure returns executable trajectory data.
        return dispatched

    def _consume(self, data):
        dt = self.config['dt_s']
        previous_q = previous_v = previous_command = None
        previous_sample = None
        observed = empty_observations()
        stage, tick = 'cycle_consistency', 0
        try:
            check_cycle(data, dt)
            if self.clock.dt != dt or self.plant.dt_s != dt or any(
                    controller.config.dt_s != dt for controller in self.controllers):
                raise ContractError(1006, 'Clock/plant feedback/PID period differs from certified cycle')
            total = data['segment_times_s'][-1] + self.config['hold_s']
            if abs(total/dt-round(total/dt)) > 1e-8:
                raise ContractError(1006, 'Trajectory end must align with control clock')
            for tick in range(round(total/dt)+1):
                observed = empty_observations()
                wall = time.perf_counter()
                stage = 'clock'
                actual_tick, t = self.clock.stamp()
                observed['observed_clock'] = dict(tick=actual_tick, time_s=t)
                if actual_tick != tick or not math.isfinite(t) or abs(t-tick*dt) > 1e-12:
                    raise ContractError(1006, 'Duplicate/skipped/reversed control tick')
                stage = 'feedback'
                feedback = self.plant.read()
                observed['observed_feedback'] = snapshot(feedback)
                if feedback['tick'] != tick or abs(feedback['time_s']-t) > 1e-12 or not math.isfinite(feedback['time_s']):
                    raise ContractError(1006, 'Feedback sequence/time is stale or inconsistent')
                q = self.vector(feedback['q_rad'], 'feedback position')
                v = self.vector(feedback['qd_rad_s'], 'feedback velocity')
                if np.any(q < np.array(self.profile['q_min_rad'])) or np.any(q > np.array(self.profile['q_max_rad'])):
                    raise ContractError(3002, 'Feedback joint position exceeds bounds')
                if np.max(abs(v)) > self.config['feedback_speed_limit_rad_s']:
                    raise ContractError(4001, 'Feedback speed exceeds low-speed bound')
                stage = 'interpolation'
                references = self.sampler(data, t)
                observed['observed_reference'] = snapshot(references)
                ref, vel, acc = [self.vector(x, 'reference') for x in references]
                if np.max(abs(vel)) > self.config['speed_limit_rad_s'] or np.max(abs(acc)) > self.config['acceleration_limit_rad_s2']:
                    raise ContractError(3002, 'Reference exceeds low-speed execution envelope')
                if previous_q is not None and (np.max(abs(ref-previous_q)) > self.config['speed_limit_rad_s']*dt+1e-8 or np.max(abs(vel-previous_v)) > self.config['acceleration_limit_rad_s2']*dt+1e-8):
                    raise ContractError(4001, 'Reference command discontinuity')
                if np.max(abs(ref-q)) > self.config['tracking_error_limit_rad']:
                    raise ContractError(4001, 'Tracking error exceeds bound')
                stage = 'collision_feedback'
                collision = self.scene.check(q.tolist())
                if not collision['passed']:
                    raise ContractError(3003, 'Feedback whole-arm/self collision')
                stage = 'control'
                outputs = [None]*6
                raw_command = [None]*6
                observed['raw_control_outputs'] = outputs
                observed['observed_command'] = raw_command
                for axis, (p, r, a, vr, ar) in enumerate(zip(self.controllers, ref, q, vel, acc)):
                    outputs[axis] = p.step(float(r), float(a), float(vr), float(ar))
                    value = outputs[axis].get('data')
                    raw_command[axis] = value.get('output') if isinstance(value, dict) else None
                if any(x['code'] for x in outputs):
                    raise ContractError(4001, 'C controller rejected feedback/reference')
                command = self.vector(raw_command, 'controller output')
                observed['validated_command'] = command.tolist()
                if np.max(abs(command)) > self.config['speed_limit_rad_s']+1e-12:
                    raise ContractError(4001, 'Controller output exceeds bound')
                if previous_command is not None and np.max(abs(command-previous_command)) > self.config['acceleration_limit_rad_s2']*dt+1e-8:
                    raise ContractError(4001, 'Velocity command discontinuity')
                row = dict(tick=tick, time_s=t, feedback_tick=feedback['tick'], feedback_time_s=feedback['time_s'],
                    q_rad=q.tolist(), qd_rad_s=v.tolist(), reference_rad=ref.tolist(),
                    reference_velocity_rad_s=vel.tolist(), reference_acceleration_rad_s2=acc.tolist(),
                    command_rad_s=command.tolist(), integral=[x['data']['integral'] for x in outputs],
                    limited=[x['data']['output_limited'] for x in outputs],
                    clearance_m=collision['minimum_distance_lower_bound_m'])
                self.rows.append(row)
                if tick < round(total/dt):
                    stage = 'command_physics'
                    self.plant.advance(command.tolist())
                    self.clock.advance()
                row['processing_wall_s'] = time.perf_counter()-wall
                previous_q, previous_v, previous_command = ref, vel, command
                previous_sample = snapshot(dict(tick=tick, time_s=t, **observed))
        except (ContractError, KeyError, TypeError, ValueError, ArithmeticError, RuntimeError) as exc:
            code = getattr(exc, 'code', 4001)
            self.events.append(dict(stage=stage, code=code, tick=tick, time_s=tick*dt, message=str(exc),
                previous_sample=previous_sample, **snapshot(observed)))
            raise ContractError(code, str(exc)) from exc
        finally:
            # Stop changes the servo target but never advances physical time.
            self.plant.stop()
