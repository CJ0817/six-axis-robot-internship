"""规划器和测试共用的仿真限值入口；缺项必须失败，不采用无限制默认值。"""
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
from adapters.c_kinematics import ContractError

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / 'config/planning-limits.v1.json'
DEFAULT_MODEL = ROOT / 'models/ur5/kinematics.json'


def _required(mapping, key):
    """读取必填字段。参数：mapping字典、key字段名；返回值；缺失/null抛1004。"""
    if not isinstance(mapping, dict) or key not in mapping or mapping[key] is None:
        raise ContractError(1004, 'Planning parameter missing: ' + key)
    return mapping[key]


def _positive(value, name, maximum=None):
    """校验正有限标量，拒绝bool和无限值；越界或错误类型抛1001。"""
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ContractError(1001, 'Expected positive finite ' + name)
    if maximum is not None and value > maximum:
        raise ContractError(1001, name + ' exceeds allowed maximum')
    return float(value)


def _vector(value, count, name, positive=False):
    """校验固定长度数值列表；positive为真时每项>0。返回tuple，非法抛1001。"""
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ContractError(1001, 'Invalid shape: ' + name)
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ContractError(1001, 'Nonfinite/non-numeric: ' + name)
    if positive and any(v <= 0 for v in value):
        raise ContractError(1001, 'Nonpositive: ' + name)
    return tuple(float(v) for v in value)


@dataclass(frozen=True)
class PlanningLimits:
    """一次加载后的实际使用上限；冻结值，不修改原始模型或C ABI。"""
    version: str
    config_sha256: str
    model_sha256: str
    velocity_scale: float
    acceleration_scale: float
    joint_velocity_rad_s: tuple
    joint_acceleration_rad_s2: tuple
    linear_velocity_m_s: float
    angular_velocity_rad_s: float
    linear_acceleration_m_s2: float
    angular_acceleration_rad_s2: float
    tcp_frame: str = 'tool0'
    tcp_point: str = 'tool0_origin'
    expression_frame: str = 'base'
    limit_metric: str = 'euclidean_norm'

    def as_dict(self):
        """返回可写入报告的字段副本，包含缩放后的实际上限和配置/模型哈希。"""
        return asdict(self)

    def check_joint_rates(self, velocity, acceleration):
        """检查逐轴速度/加速度绝对值。入参6维rad/s、rad/s²；通过返回True。

        非法输入抛1001，超任一实际使用上限抛3002；不裁剪、不自动放宽。
        """
        v = _vector(velocity, 6, 'joint velocity')
        a = _vector(acceleration, 6, 'joint acceleration')
        if any(abs(x) > limit for values, limits in (
                (v, self.joint_velocity_rad_s), (a, self.joint_acceleration_rad_s2))
               for x, limit in zip(values, limits)):
            raise ContractError(3002, 'Joint trajectory rate exceeded')
        return True

    def check_tcp_rates(self, linear_velocity, angular_velocity,
                        linear_acceleration, angular_acceleration):
        """检查tool0原点的四个base系3维向量模长。单位依次m/s、rad/s、m/s²、rad/s²。

        角速度为空间旋转角速度，不是欧拉角导数；角加速度为其时间导数。
        返回True或抛1001/3002；各分量合格但模长超限仍失败。
        """
        for name, value, maximum in (
                ('linear velocity', linear_velocity, self.linear_velocity_m_s),
                ('angular velocity', angular_velocity, self.angular_velocity_rad_s),
                ('linear acceleration', linear_acceleration, self.linear_acceleration_m_s2),
                ('angular acceleration', angular_acceleration, self.angular_acceleration_rad_s2)):
            if math.hypot(*_vector(value, 3, name)) > maximum:
                raise ContractError(3002, 'TCP vector norm exceeded: ' + name)
        return True


def load_planning_limits(config_path=DEFAULT_CONFIG, model_path=DEFAULT_MODEL,
                         usage='simulation_only', velocity_scale=1.0, acceleration_scale=1.0):
    """规划/测试共用的必需配置闸门。

    入参：两个JSON路径；usage仅simulation_only；请求缩放在(0,1]。
    返回PlanningLimits，实际上限=基础值×配置缩放×请求缩放。
    缺文件/必填项/null抛1004，非法数值1001，单位1002，轴序1003，
    TCP定义1007，硬件使用或版本不支持1008。未配置不得启动正式规划。
    """
    try:
        raw = Path(config_path).read_bytes()
        model_raw = Path(model_path).read_bytes()
    except OSError as exc:
        raise ContractError(1004, 'Planning configuration/model unavailable') from exc
    try:
        config = json.loads(raw); model = json.loads(model_raw)
    except (ValueError, UnicodeError) as exc:
        raise ContractError(1001, 'Invalid configuration JSON') from exc
    if usage != 'simulation_only' or _required(config, 'usage') != 'simulation_only':
        raise ContractError(1008, 'This configuration is simulation only')
    if type(_required(config, 'schema_version')) is not int or config['schema_version'] != 1:
        raise ContractError(1008, 'Unsupported planning schema')
    if _required(config, 'version') != '1.0.0' or _required(config, 'status') != 'frozen_simulation_only':
        raise ContractError(1008, 'Planning parameters not frozen/supported')
    revision = _required(config, 'revision')
    if not isinstance(revision, str) or not revision.strip():
        raise ContractError(1001, 'Planning revision required')
    model_hash = hashlib.sha256(model_raw).hexdigest()
    if _required(config, 'model_sha256') != model_hash or _required(config, 'model_id') != _required(model, 'model_id'):
        raise ContractError(1004, 'Configuration model binding mismatch')
    order = ['J' + str(i) for i in range(1, 7)]
    if _required(config, 'joint_names') != order or _required(model, 'joint_names') != order:
        raise ContractError(1003, 'Planning joint order mismatch')
    expected_units = dict(joint_velocity='rad/s', joint_acceleration='rad/s^2',
                          linear_velocity='m/s', linear_acceleration='m/s^2',
                          angular_velocity='rad/s', angular_acceleration='rad/s^2')
    units = _required(config, 'units')
    if any(_required(units, k) != v for k, v in expected_units.items()):
        raise ContractError(1002, 'Planning units mismatch')
    tcp = _required(config, 'tcp')
    expected_tcp = dict(frame='tool0', point='tool0_origin', expression_frame='base', limit_metric='euclidean_norm')
    if any(_required(tcp, k) != v for k, v in expected_tcp.items()):
        raise ContractError(1007, 'TCP/frame/metric differs from frozen profile')
    transform = _required(tcp, 'T_tool0_tcp')
    if not isinstance(transform, list) or len(transform) != 4:
        raise ContractError(1001, 'TCP transform must be 4x4')
    transform = [list(_vector(row, 4, 'TCP transform row')) for row in transform]
    if transform != [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]:
        raise ContractError(1007, 'A new TCP needs a separate parameter revision')
    if _required(model, 'tool_frame') != 'tool0' or _required(model, 'base_frame') != 'base':
        raise ContractError(1007, 'Model frame mismatch')
    sources = _required(config, 'sources')
    for key in ('joint_velocity', 'joint_acceleration', 'tcp_limits', 'scales'):
        if not isinstance(_required(sources, key), str) or not sources[key].strip():
            raise ContractError(1001, 'Source description required: ' + key)
    joints = _required(config, 'joint_limits'); cart = _required(config, 'tcp_limits')
    qd = _vector(_required(joints, 'velocity_rad_s'), 6, 'joint maximum velocity', True)
    qdd = _vector(_required(joints, 'acceleration_rad_s2'), 6, 'joint maximum acceleration', True)
    upstream_qd = _vector(_required(model, 'qd_max_rad_s'), 6, 'model maximum velocity', True)
    if any(v > bound for v, bound in zip(qd, upstream_qd)):
        raise ContractError(1001, 'Planning joint speed exceeds model bound')
    scales = _required(config, 'scales')
    vs = _positive(_required(scales, 'velocity'), 'configured velocity scale', 1.0)*_positive(velocity_scale, 'request velocity scale', 1.0)
    accs = _positive(_required(scales, 'acceleration'), 'configured acceleration scale', 1.0)*_positive(acceleration_scale, 'request acceleration scale', 1.0)
    return PlanningLimits(config['version'], hashlib.sha256(raw).hexdigest(), model_hash, vs, accs,
                          tuple(v*vs for v in qd), tuple(v*accs for v in qdd),
                          _positive(_required(cart, 'linear_velocity_m_s'), 'TCP linear velocity')*vs,
                          _positive(_required(cart, 'angular_velocity_rad_s'), 'TCP angular velocity')*vs,
                          _positive(_required(cart, 'linear_acceleration_m_s2'), 'TCP linear acceleration')*accs,
                          _positive(_required(cart, 'angular_acceleration_rad_s2'), 'TCP angular acceleration')*accs)
