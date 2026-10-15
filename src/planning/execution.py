"""Simulation execution gate for fully certified, unmodified trajectories.

Certificates are process-local capabilities, not durable security credentials.
Reloaded JSON must be certified again in the current process and scene.
"""
import copy
import hashlib
import hmac
import json
from pathlib import Path
import secrets

from adapters.c_kinematics import ContractError

_KEY = secrets.token_bytes(32)
_FIELDS = ('time_s', 'q_rad', 'qd_rad_s', 'qdd_rad_s2',
           'coefficients_rad', 'segment_times_s', 'effective_limits', 'scene',
           'collision_checked', 'cartesian_limits_checked',
           'continuous_tcp_proof', 'continuous_collision_proof',
           'tcp_continuous_bounds', 'certified_minimum_distance_lower_bound_m')


def _json(value):
    """Return canonical finite JSON bytes for hashing evidence and payloads."""
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def _scene(scene):
    """Bind geometry and scene settings; exclude only the query counter."""
    metadata = scene.metadata()
    metadata.pop('calls', None)
    return metadata


def _payload(data):
    """Bind all executable samples, polynomials, constraints and proof flags."""
    return {key: data[key] for key in _FIELDS}


def _signature(data, certificate):
    """Authenticate the executable payload and certification context together."""
    return hmac.new(_KEY, _json([_payload(data), certificate]), hashlib.sha256).hexdigest()


def seal(data, verifier, scene):
    """Mark a trajectory only after complete certification; return the same dict."""
    paths = [verifier.limits_path, verifier.model_path, verifier.config_path,
             verifier.kinematics_library, verifier.planning_library]
    context = {str(Path(path).resolve()): hashlib.sha256(Path(path).read_bytes()).hexdigest()
               for path in paths}
    certificate = dict(version=1, usage='simulation_only', context_sha256=context,
                       scene=_scene(scene),sample_period_s=verifier.config['sample_period_s'])
    certificate['token'] = _signature(data, certificate)
    data.update(trajectory_status='certified', executable=True, certification=certificate)
    return data


def failure(exc, stage, records=None):
    """Fail closed; preserve stage, reason and attempt history, never motion data."""
    return dict(code=getattr(exc, 'code', 1004 if isinstance(exc, (OSError, KeyError, ImportError)) else 1001),
                message=str(exc), data=None, next_state=None,
                details=dict(stage=stage, failure_log=getattr(exc,'certification_log',[dict(stage=stage, message=str(exc))]),
                             attempts=records or [], stop_segment=True, replan_required=True))


def execute_trajectory(result, scene, consumer):
    """Validate before invoking a simulation consumer once with a private copy.

    Inputs: successful certified Result, current StaticScene and callable consumer.
    Reject candidates, failures, forged/modified data or changed context with 3001;
    missing configuration/dependency returns 1004. No consumer call on rejection.
    Consumer is responsible for simulation sampling; this is not hardware approval.
    """
    try:
        if not isinstance(result, dict) or result.get('code') != 0 or not isinstance(result.get('data'), dict):
            raise ContractError(3001, 'Successful certified trajectory required')
        data = copy.deepcopy(result['data'])
        if data.get('trajectory_status') != 'certified' or data.get('executable') is not True:
            raise ContractError(3001, 'Candidate/uncertified trajectory cannot execute')
        certificate = copy.deepcopy(data.get('certification', {}))
        token = certificate.pop('token', '')
        if not token or not hmac.compare_digest(token, _signature(data, certificate)):
            raise ContractError(3001, 'Trajectory/certificate was modified or needs recertification')
        if data.get('sample_period_s',certificate['sample_period_s'])!=certificate['sample_period_s'] or data.get('actual_duration_s',data['segment_times_s'][-1])!=data['segment_times_s'][-1]:
            raise ContractError(3001, 'Trajectory timing metadata changed; recertification required')
        if scene is None:
            raise ContractError(1004, 'Execution collision scene required')
        if certificate['scene'] != _scene(scene):
            raise ContractError(3001, 'Scene/geometry changed; recertification required')
        for name, expected in certificate['context_sha256'].items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest() != expected:
                raise ContractError(3001, 'Configuration/model/library changed; recertification required')
        if not callable(consumer):
            raise ContractError(1001, 'Simulation consumer must be callable')
        consumer(data)
        return dict(code=0, message='Certified trajectory dispatched', data=None, next_state=None, details={})
    except (ContractError, OSError, KeyError, ValueError, TypeError, ArithmeticError, ImportError) as exc:
        return failure(exc, 'execution_gate')
