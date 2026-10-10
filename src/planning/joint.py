"""C11静止关节轨迹的Python入口；共享参数加载、解析极值、统一Result。"""
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
from adapters.c_kinematics import ContractError, D6, numbers
from planning.limits import DEFAULT_CONFIG, DEFAULT_MODEL, load_planning_limits

METHODS = {'cubic': 1, 'quintic': 2, 'trapezoidal': 3}

class CPlan(C.Structure):
    """独立规划v0.1结构；与robot_joint_plan逐字段对应，不属于运动学ABI。"""
    _fields_ = [('method', C.c_int)] + [(n, C.c_double) for n in
        ('duration_s', 'minimum_duration_s', 'ramp_s', 'cruise_s')] + [
        ('q_start', D6), ('delta', D6)]


def _time(value, name, allow_none=False):
    """检查[1e-9,1e6]秒标量；允许的None仅表示自动定时，非法抛1006。"""
    if allow_none and value is None:
        return 0.0
    if type(value) not in (int, float) or not math.isfinite(value) or value < 1e-9 or value > 1e6:
        raise ContractError(1006, 'Invalid ' + name)
    return float(value)

class JointPlanner:
    """加载独立C库；参数路径默认指向冻结配置，规划时重新核对配置/模型绑定。"""
    def __init__(self, library, config_path=DEFAULT_CONFIG, model_path=DEFAULT_MODEL):
        """入参：独立规划动态库路径、配置与模型路径。加载错误不伪造成功。"""
        self.library = Path(library).resolve()
        self.lib = C.CDLL(str(self.library))
        self.config_path, self.model_path = Path(config_path), Path(model_path)
        self.fn = self.lib.robot_plan_joint
        self.fn.argtypes = [C.c_int] + [C.POINTER(C.c_double)]*6 + [C.c_size_t,
            C.c_double, C.c_int, C.c_double, C.c_double, C.POINTER(CPlan)]
        self.fn.restype = C.c_int
        self.evaluate = self.lib.robot_joint_evaluate
        self.evaluate.argtypes = [C.POINTER(CPlan), C.c_double] + [C.POINTER(C.c_double)]*3 + [C.c_size_t]
        self.evaluate.restype = C.c_int

    def at(self, plan, t):
        """解析求值：plan为CPlan，t单位秒。返回q/qd/qdd三个6维列表；失败抛统一码。"""
        q, v, a = D6(), D6(), D6()
        code = self.evaluate(C.byref(plan), t, q, v, a, 6)
        if code:
            raise ContractError(code, 'C trajectory evaluation rejected')
        return list(q), list(v), list(a)

    def plan_joint(self, q_start_rad, q_goal_rad, options):
        """生成静止到静止、六轴同步的完整关节轨迹。

        入参：两组J1..J6角度(rad)；options必填method、sample_period_s、
        duration_s(None自动)、duration_policy(strict/stretch)、max_duration_s。
        可选请求缩放默认1、joint_margin_rad默认0、collision_policy仅unchecked。
        非零边界速度/加速度尚不支持，拒绝未知字段。缩放仅由共享加载器应用一次。
        返回统一Result：成功data含采样/局部系数/解析峰值；失败data=None。
        本入口只保证关节位置/速度/加速度，不宣称TCP限值或碰撞已验证。
        """
        try:
            if not isinstance(options, dict):
                raise ContractError(1001, 'options must be object')
            required = ('method', 'sample_period_s', 'duration_s', 'duration_policy', 'max_duration_s')
            if any(k not in options for k in required):
                raise ContractError(1001, 'Missing trajectory option')
            allowed = set(required) | {'velocity_scale', 'acceleration_scale', 'joint_margin_rad', 'collision_policy'}
            if set(options)-allowed:
                raise ContractError(1008, 'Unsupported option/boundary condition')
            method = options['method']
            if not isinstance(method, str) or method not in METHODS:
                raise ContractError(1008, 'Unsupported method')
            if options['duration_policy'] not in ('strict', 'stretch'):
                raise ContractError(1001, 'Unknown duration policy')
            if options['duration_s'] is None and options['duration_policy'] != 'stretch':
                raise ContractError(1001, 'Automatic duration requires stretch')
            if options.get('collision_policy', 'unchecked') != 'unchecked':
                raise ContractError(1008, 'Collision checker not implemented')
            dt = _time(options['sample_period_s'], 'sample_period_s')
            requested = _time(options['duration_s'], 'duration_s', True)
            maximum = _time(options['max_duration_s'], 'max_duration_s')
            limits = load_planning_limits(self.config_path, self.model_path,
                velocity_scale=options.get('velocity_scale', 1.0),
                acceleration_scale=options.get('acceleration_scale', 1.0))
            model = json.loads(self.model_path.read_bytes())
            margin = options.get('joint_margin_rad', 0.0)
            if type(margin) not in (int, float) or not math.isfinite(margin) or margin < 0:
                raise ContractError(1001, 'Invalid position margin')
            lo = [x+margin for x in numbers(model['q_min_rad'], 6)]
            hi = [x-margin for x in numbers(model['q_max_rad'], 6)]
            p = CPlan()
            code = self.fn(METHODS[method], D6(*numbers(q_start_rad, 6)), D6(*numbers(q_goal_rad, 6)),
                D6(*lo), D6(*hi), D6(*limits.joint_velocity_rad_s), D6(*limits.joint_acceleration_rad_s2),
                6, requested, int(options['duration_policy']=='stretch'), dt, maximum, C.byref(p))
            if code:
                raise ContractError(code, 'C planner rejected request')
            T = p.duration_s
            if not math.isfinite(T/dt) or T/dt > 100000:
                raise ContractError(3001, 'Sampling budget exceeded (100000 intervals)')
            data = self._serialize(p, method, dt, lo, hi, limits)
            return {'code': 0, 'message': 'OK', 'data': data, 'details': {
                'requested_duration_s': options['duration_s'], 'actual_duration_s': T,
                'minimum_duration_s': p.minimum_duration_s,
                'duration_stretched': requested>0 and T>requested,
                'library_sha256': hashlib.sha256(self.library.read_bytes()).hexdigest()}}
        except (ContractError, KeyError, ArithmeticError, OSError, ValueError) as exc:
            return {'code': getattr(exc, 'code', 1004 if isinstance(exc, (KeyError, OSError)) else 1001),
                    'message': str(exc), 'data': None, 'details': {'module': 'planning'}}

    def _serialize(self, p, method, dt, lo, hi, limits):
        """按解析极值确认整段关节约束，再序列化采样和归一化局部升幂系数(M,6,6)。"""
        T = p.duration_s
        moving = any(p.delta)
        if method=='trapezoidal' and moving:
            segment_times = [0.0, p.ramp_s]
            if p.cruise_s>0:
                segment_times.append(p.ramp_s+p.cruise_s)
            segment_times.append(T)
            kinds = ['acceleration', 'cruise', 'deceleration'] if p.cruise_s>0 else ['acceleration', 'deceleration']
            vs = 1.0/(p.ramp_s+p.cruise_s)
            acs = vs/p.ramp_s
            coefficients = []
            for t in segment_times[:-1]:
                q, v, a = self.at(p, t)
                coefficients.append([[q[i], v[i], a[i]/2, 0.0, 0.0, 0.0] for i in range(6)])
        else:
            segment_times = [0.0, T]; kinds = ['hold' if not moving else method]
            vs = 1.5/T if method=='cubic' else 1.875/T if method=='quintic' else 0.0
            acs = 6.0/T**2 if method=='cubic' else (10/math.sqrt(3))/T**2 if method=='quintic' else 0.0
            coefficients = [[[p.q_start[i], 0.0,
                3*p.delta[i]/T**2 if method=='cubic' else 0.0,
                -2*p.delta[i]/T**3 if method=='cubic' else 10*p.delta[i]/T**3,
                0.0 if method=='cubic' else -15*p.delta[i]/T**4,
                0.0 if method=='cubic' else 6*p.delta[i]/T**5] for i in range(6)]]
        peak_v = [abs(d)*vs for d in p.delta]; peak_a = [abs(d)*acs for d in p.delta]
        # 三种静止轨迹s单调，位置极值必在起终点；速度/加速度峰值为封闭公式。
        limits.check_joint_rates(peak_v, peak_a)
        q_end = [p.q_start[i]+p.delta[i] for i in range(6)]
        if any(min(p.q_start[i], q_end[i])<lo[i] or max(p.q_start[i], q_end[i])>hi[i] for i in range(6)):
            raise ContractError(1005, 'Continuous position bound failed')
        count = math.floor(T/dt)
        critical = [T/2] if method=='cubic' else [T/2, T*(3-math.sqrt(3))/6, T*(3+math.sqrt(3))/6] if method=='quintic' else []
        times = sorted(set([k*dt for k in range(count+1) if k*dt<T]+segment_times+critical))
        q, v, a = [], [], []
        for t in times:
            x, y, z = self.at(p, t); q.append(x); v.append(y); a.append(z)
        # 保持既有Trajectory约定：u=(t-t0)/(t1-t0)，导数再除段时长。
        coefficients = [[[c*(segment_times[j+1]-segment_times[j])**k for k,c in enumerate(axis)]
                         for axis in segment] for j,segment in enumerate(coefficients)]
        if not all(math.isfinite(x) for seg in coefficients for axis in seg for x in axis):
            raise ContractError(1001, 'Nonfinite polynomial coefficients')
        return {'method': method, 'boundary': 'rest_to_rest',
            'continuity': 'C2' if method=='quintic' else 'C1',
            'time_s': times, 'q_rad': q, 'qd_rad_s': v, 'qdd_rad_s2': a,
            'segment_times_s': segment_times, 'segment_kind': kinds,
            'coefficients_rad': coefficients, 'coefficient_convention': 'ascending powers of normalized segment time u=(t-t0)/(t1-t0)',
            'acceleration_sample_side': 'right at switches; left at final endpoint',
            'effective_limits': limits.as_dict(),
            'continuous_joint_check': {'passed': True, 'proof': 'monotone position and closed-form extrema',
                'peak_velocity_rad_s': peak_v, 'peak_acceleration_rad_s2': peak_a},
            'collision_checked': False, 'cartesian_limits_checked': False,
            'triangular': method=='trapezoidal' and moving and p.cruise_s==0.0}
