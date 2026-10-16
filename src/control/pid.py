"""Explicit ctypes binding for the independent C single-joint PID library."""
import ctypes as C
import math
from pathlib import Path


class Config(C.Structure):
    """C config layout; gains and output units are chosen by the plant config."""
    _fields_ = [(n, C.c_double) for n in (
        'dt_s', 'kp', 'ki', 'kd', 'integral_min', 'integral_max',
        'output_min', 'output_max')]


class State(C.Structure):
    """Caller-owned C history, reset before use; not a device safety state."""
    _fields_ = [('config', Config)] + [(n, C.c_double) for n in (
        'error_1', 'error_2', 'integral', 'raw_output', 'output')] + [
        ('initialized', C.c_uint32)]


class Sample(C.Structure):
    """One successful C sample, including both saturation diagnostics."""
    _fields_ = [(n, C.c_double) for n in (
        'error', 'integral', 'raw_output', 'output')] + [
        ('integral_limited', C.c_uint32), ('output_limited', C.c_uint32)]


class PID:
    """Load explicit library/config, own history and expose uniform Results."""
    def __init__(self, library, config):
        """Args: library path and all eight numeric Config fields; reset or raise."""
        self.library = Path(library).resolve()
        self.lib = C.CDLL(str(self.library))
        self.lib.robot_pid_reset.argtypes = [C.POINTER(Config), C.POINTER(State)]
        self.lib.robot_pid_reset.restype = C.c_int
        self.lib.robot_pid_step.argtypes = [C.POINTER(State), C.c_double,
                                          C.c_double, C.POINTER(Sample)]
        self.lib.robot_pid_step.restype = C.c_int
        if set(config) != {n for n, _ in Config._fields_}:
            raise ValueError('All and only Config fields are required')
        if any(type(v) not in (int, float) for v in config.values()):
            raise ValueError('Config values must be numeric, not bool')
        self.config = Config(**config)
        self.state = State()
        code = self.reset()
        if code:
            raise ValueError('PID reset rejected config: ' + str(code))

    def reset(self):
        """Clear I, two errors and output; return C status, preserve on failure."""
        return self.lib.robot_pid_reset(C.byref(self.config), C.byref(self.state))

    def step(self, reference_rad, actual_rad):
        """Take finite rad scalars; return code/data, never stale success data."""
        if any(type(v) not in (int, float) for v in (reference_rad, actual_rad)):
            return {'code': 1001, 'data': None}
        sample = Sample()
        try:
            code = self.lib.robot_pid_step(C.byref(self.state), reference_rad,
                                          actual_rad, C.byref(sample))
        except (OverflowError, TypeError, ValueError):
            code = 1001
        return {'code': code, 'data': None if code else {
            n: getattr(sample, n) for n, _ in Sample._fields_}}


class FeedforwardSample(C.Structure):
    """Additive result layout; torque components and total saturation flags."""
    _fields_ = [(n,C.c_double) for n in ('error','integral','feedback_raw',
        'velocity_feedforward','acceleration_feedforward','total_raw','output')] + [
        ('integral_limited',C.c_uint32),('output_limited',C.c_uint32)]


class FeedforwardPID(PID):
    """Same PID/reset, plus explicit velocity and acceleration feedforward gains."""
    def __init__(self,library,config,kv,ka):
        """Load C library; kv/ka finite >=0, using the plant's output units."""
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<0 for v in (kv,ka)):
            raise ValueError('Finite nonnegative feedforward gains required')
        super().__init__(library,config)
        self.kv,self.ka=kv,ka
        self.lib.robot_pid_step_ff.argtypes=[C.POINTER(State)]+[C.c_double]*6+[C.POINTER(FeedforwardSample)]
        self.lib.robot_pid_step_ff.restype=C.c_int

    def step(self,reference_rad,actual_rad,reference_velocity_rad_s,reference_acceleration_rad_s2):
        """Return total-clamped output and components; invalid input never advances."""
        values=(reference_rad,actual_rad,reference_velocity_rad_s,reference_acceleration_rad_s2)
        if any(type(v) not in (int,float) for v in values):
            return {'code':1001,'data':None}
        sample=FeedforwardSample()
        try:code=self.lib.robot_pid_step_ff(C.byref(self.state),*values,self.kv,self.ka,C.byref(sample))
        except (OverflowError,TypeError,ValueError):code=1001
        return {'code':code,'data':None if code else {n:getattr(sample,n) for n,_ in sample._fields_}}
