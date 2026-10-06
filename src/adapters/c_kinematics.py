"""ctypes binding: explicit validated model data, no hidden UR5 defaults."""
import ctypes as C
import math
from pathlib import Path

D6=C.c_double*6
D16=C.c_double*16

class FKModel(C.Structure):
    """C正逆解共享的704字节几何模型；字段布局必须与robot_fk_model一致。"""
    _fields_=[(name,D6) for name in ('a_m','alpha_rad','d_m','theta_offset_rad')]+[
        ('joint_sign',C.c_int*6),('T_base_dh0',D16),('T_dh6_flange',D16),('T_flange_tool',D16),
        ('q_min_rad',D6),('q_max_rad',D6),('pose_validation_tol',C.c_double)]

class ContractError(ValueError):
    def __init__(self,code,message):
        """保存接口校验失败的统一错误码。

        参数：code 为 contract.json 中的整数码；message 为可读说明。
        返回：无；异常对象提供 code 属性，供上层转换成失败 Result。"""
        super().__init__(message);self.code=code

def numbers(value,n):
    """检查固定长度的有限数值数组，不做单位转换或角度折返。

    参数：value 为 list/tuple；n 为要求的元素个数。
    返回：原数组；形状、类型或 NaN/Inf 不合法时抛出 ContractError(1001)。"""
    if not isinstance(value,(list,tuple)) or len(value)!=n:
        raise ContractError(1001,'Invalid array shape')
    if any(type(x) not in (int,float) or not math.isfinite(x) for x in value):
        raise ContractError(1001,'Expected finite numeric array')
    return value

def pack_model(profile):
    """把工程模型字典按冻结 ABI 打包成 ctypes 结构。

    参数：profile 含标准 DH、六轴顺序、m/rad 单位、base/tool0 标签、
    固定变换和关节限位；详见 models/ur5/kinematics.json。
    返回：FKModel；缺字段抛 KeyError，错误单位/轴序/坐标/类型抛 ContractError。
    本方法做包装校验，完整几何合法性由 C 接口继续检查。"""
    if not isinstance(profile,dict):raise ContractError(1004,'Model is not configured')
    for field,expected,code in [('representation','standard_dh',1008),('length_unit','m',1002),
        ('angle_unit','rad',1002),('base_frame','base',1007),('tool_frame','tool0',1007),
        ('joint_names',[f'J{i}' for i in range(1,7)],1003)]:
        if profile[field]!=expected:raise ContractError(code,'Mismatch: '+field)
    if type(profile['dof']) is not int or profile['dof']!=6:raise ContractError(1001,'Need six joints')
    model=FKModel()
    for name in ('a_m','alpha_rad','d_m','theta_offset_rad','q_min_rad','q_max_rad'):
        setattr(model,name,D6(*numbers(profile[name],6)))
    signs=profile['joint_sign']
    if not isinstance(signs,list) or len(signs)!=6 or any(type(v) is not int or v not in (-1,1) for v in signs):
        raise ContractError(1001,'Invalid joint signs')
    model.joint_sign=(C.c_int*6)(*signs)
    for name in ('T_base_dh0','T_dh6_flange','T_flange_tool'):
        matrix=profile[name]
        if not isinstance(matrix,list) or len(matrix)!=4:raise ContractError(1001,'Expected 4x4 matrix')
        setattr(model,name,D16(*(v for row in matrix for v in numbers(row,4))))
    tol=profile['pose_validation_tol']
    if type(tol) not in (int,float) or not math.isfinite(tol):raise ContractError(1001,'Invalid tolerance')
    model.pose_validation_tol=tol
    return model

class Forward:
    def __init__(self,library):
        """加载 C 动态库，并声明正解函数的参数和返回类型。

        参数：library 为 librobot_contract.so 的文件路径。
        返回：无；保存 lib 和 fn。文件或符号不可用时由 ctypes 抛出异常。"""
        self.lib=C.CDLL(str(Path(library).resolve()))
        self.fn=self.lib.robot_forward
        self.fn.argtypes=[C.POINTER(FKModel),C.POINTER(C.c_double),C.c_size_t,C.POINTER(C.c_double)]
        self.fn.restype=C.c_int

    def __call__(self,profile,q_rad):
        """求六关节配置的工具位姿，并将 C 错误封装为统一 Result。

        参数：profile 为显式工程模型；q_rad 为 J1～J6 的六个角度，单位 rad。
        返回：code=0 时 data 为 base→tool0 的 4×4 矩阵；
        失败时 data=None，details 提供校验字段或模块信息，不复用旧输出。"""
        try:
            q=D6(*numbers(q_rad,6));model=pack_model(profile);out=D16()
            code=self.fn(C.byref(model),q,6,out)
            if code:return {'code':code,'message':'C FK rejected input','data':None,'details':{'module':'kinematics'}}
            return {'code':0,'message':'OK','data':[list(out)[i:i+4] for i in range(0,16,4)],
                    'details':{'frame':'T_base_tool','base_link':'base','tool_link':'tool0'}}
        except KeyError as exc:
            return {'code':1004,'message':'Missing model field','data':None,'details':{'field':str(exc)}}
        except (ContractError,OverflowError,TypeError) as exc:
            return {'code':getattr(exc,'code',1001),'message':str(exc),'data':None,'details':{'module':'kinematics'}}

class IKOptionsV1(C.Structure):
    """冻结的72字节IK选项；枚举、预算与误差阈值对应C头文件。"""
    _fields_=[('method',C.c_uint32),('branch_policy',C.c_uint32)]+[
        (name,C.c_double) for name in ('position_tol_m','orientation_tol_rad','joint_margin_rad','timeout_s')]+[
        ('max_iterations',C.c_uint32),('line_search_max_steps',C.c_uint32)]+[
        (name,C.c_double) for name in ('characteristic_length_m','damping','max_step_rad')]

class IKResultV1(C.Structure):
    """冻结的504字节IK结果；仅成功码0时前solution_count个候选有效。"""
    _fields_=[('q_rad',D6),('candidates_rad',D6*8),('branch_ids',C.c_uint32*8),
              ('solution_count',C.c_uint32),('selected_index',C.c_uint32),
              ('position_error_m',C.c_double),('orientation_error_rad',C.c_double),
              ('elapsed_s',C.c_double),('iterations',C.c_uint32),('reserved',C.c_uint32)]
