"""ctypes binding: explicit validated model data, no hidden UR5 defaults."""
import ctypes as C
import math
from pathlib import Path

D6=C.c_double*6
D16=C.c_double*16

class FKModel(C.Structure):
    _fields_=[(name,D6) for name in ('a_m','alpha_rad','d_m','theta_offset_rad')]+[
        ('joint_sign',C.c_int*6),('T_base_dh0',D16),('T_dh6_flange',D16),('T_flange_tool',D16),
        ('q_min_rad',D6),('q_max_rad',D6),('pose_validation_tol',C.c_double)]

class ContractError(ValueError):
    def __init__(self,code,message):
        super().__init__(message);self.code=code

def numbers(value,n):
    if not isinstance(value,(list,tuple)) or len(value)!=n:
        raise ContractError(1001,'Invalid array shape')
    if any(type(x) not in (int,float) or not math.isfinite(x) for x in value):
        raise ContractError(1001,'Expected finite numeric array')
    return value

def pack_model(profile):
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
        self.lib=C.CDLL(str(Path(library).resolve()))
        self.fn=self.lib.robot_forward
        self.fn.argtypes=[C.POINTER(FKModel),C.POINTER(C.c_double),C.c_size_t,C.POINTER(C.c_double)]
        self.fn.restype=C.c_int

    def __call__(self,profile,q_rad):
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
