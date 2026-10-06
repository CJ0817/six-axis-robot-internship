"""State-aware selection above frozen C ABI v1; pure planning, no actuation."""
import ctypes as C
from dataclasses import dataclass, asdict
import math
import time
import numpy as np
from .c_kinematics import Forward, FKModel, IKOptionsV1, IKResultV1, D6, D16, pack_model, numbers, ContractError

@dataclass(frozen=True)
class SelectionConfig:
    """上层选解参数；角度rad、时间s、位置m，默认值与约束见ik-stateful-selection.md。"""
    travel_weight: float = 1.0
    limit_weight: float = 0.05
    singular_weight: float = 0.05
    max_step_rad: float = 0.15
    dt_s: float = 0.02
    joint_margin_rad: float = 0.0
    limit_scale_rad: float = 0.2
    sigma_soft: float = 0.02
    sigma_hard: float = 1e-5
    near_step_scale: float = 0.25
    characteristic_length_m: float = 1.0
    difference_step_rad: float = 1e-6
    position_tol_m: float = 1e-5
    orientation_tol_rad: float = 1e-4
    timeout_s: float = 0.2
    dls_enabled: bool = True
    dls_max_iterations: int = 24
    dls_line_search_steps: int = 8
    dls_damping_min: float = 0.001
    dls_damping_max: float = 0.1

    def validate(self):
        """校验选解和局部阻尼配置，防止无效权重或预算进入求解。

        参数：使用本实例的字段；角度 rad，周期/预算 s，位移 m。
        返回：无；不合法时抛 ContractError(1001)，不自动调整阈值。"""
        numeric=(v for k,v in asdict(self).items() if k!='dls_enabled')
        if type(self.dls_enabled) is not bool or any(type(v) not in (int,float) or not math.isfinite(v) for v in numeric):
            raise ContractError(1001,'Config must contain finite numbers')
        if any(type(v) is not int or v<1 for v in (self.dls_max_iterations,self.dls_line_search_steps)):
            raise ContractError(1001,'DLS budgets must be positive integers')
        if any(v<0 for v in (self.travel_weight,self.limit_weight,self.singular_weight,self.joint_margin_rad)) or self.travel_weight+self.limit_weight+self.singular_weight<=0:
            raise ContractError(1001,'Invalid weights/margin')
        if any(v<=0 for v in (self.max_step_rad,self.dt_s,self.limit_scale_rad,self.sigma_hard,self.characteristic_length_m,self.difference_step_rad,self.position_tol_m,self.orientation_tol_rad,self.timeout_s)):
            raise ContractError(1001,'Positive configuration required')
        if not self.sigma_hard<self.sigma_soft or not 0<self.near_step_scale<=1 or self.orientation_tol_rad>math.pi or self.difference_step_rad>1e-3:
            raise ContractError(1001,'Invalid singular/step/angle thresholds')
        if self.dls_damping_min<=0 or self.dls_damping_max<self.dls_damping_min:
            raise ContractError(1001,'Invalid DLS damping interval')

def pose_error(a,b):
    """分别计算两刚体位姿的位置距离和相对旋转角。

    参数：a、b 为同一坐标系下合法的 4×4 NumPy 位姿矩阵。
    返回：(位置误差 m，姿态误差 rad)，均为非负标量；不混成加权误差。"""
    R=a[:3,:3].T @ b[:3,:3]
    v=np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/2
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),math.atan2(float(np.linalg.norm(v)),float(np.clip((np.trace(R)-1)/2,-1,1)))

class StatefulIKSelector:
    def __init__(self,library,config=None):
        """建立 C FK/解析 IK 绑定，保存上层选解配置。

        参数：library 为 C 动态库路径；config 为 SelectionConfig，可省略采用显式默认值。
        返回：无；实例不保存上一关节状态，状态由调用者传入 select。"""
        self.config=config or SelectionConfig()
        self.forward=Forward(library)
        self.inverse=self.forward.lib.robot_inverse_v1
        self.inverse.restype=C.c_int
        self.inverse.argtypes=[C.POINTER(FKModel),C.POINTER(C.c_double),C.c_size_t,C.POINTER(C.c_double),C.c_size_t,C.POINTER(IKOptionsV1),C.POINTER(IKResultV1)]

    def _fk(self,model,q):
        """调用 C 正解，为回代和数值雅可比提供统一几何计算。

        参数：model 为已打包 FKModel；q 为六个关节角，单位 rad。
        返回：4×4 NumPy base→tool0 位姿；C 非零码抛 ContractError，禁止使用输出。"""
        out=D16();code=self.forward.fn(C.byref(model),D6(*q),6,out)
        if code:raise ContractError(code,'C FK rejected selection input')
        return np.array(list(out)).reshape(4,4)

    def jacobian(self,model,q):
        """通过 C FK 差分估计工具原点的空间雅可比。

        参数：model 为 FKModel；q 为限位内六关节角(rad)。
        返回：6×6 NumPy 矩阵；前三行线速度除以 characteristic_length_m，
        后三行为空间角速度。通常中心差分，关节端点采用限位内单侧差分。
        此函数不是冻结 ABI 的 C 解析雅可比。"""
        c=self.config;q=np.array(q,dtype=float);T=self._fk(model,q);J=np.empty((6,6))
        for i in range(6):
            left=q[i]-model.q_min_rad[i];right=model.q_max_rad[i]-q[i]
            h=c.difference_step_rad
            if left>=h and right>=h:
                a=q.copy();b=q.copy();a[i]-=h;b[i]+=h;A=self._fk(model,a);B=self._fk(model,b);delta=2*h
            elif right>=left and right>0:
                h=min(h,right);b=q.copy();b[i]+=h;A=T;B=self._fk(model,b);delta=b[i]-q[i]
            elif left>0:
                h=min(h,left);a=q.copy();a[i]-=h;A=self._fk(model,a);B=T;delta=q[i]-a[i]
            else:raise ContractError(1001,'No finite-difference interval')
            if delta<=0:raise ContractError(1001,'Joint interval below floating-point resolution')
            dR=((B[:3,:3]-A[:3,:3])/delta) @ T[:3,:3].T
            J[:3,i]=(B[:3,3]-A[:3,3])/delta/c.characteristic_length_m
            J[3:,i]=np.array([dR[2,1]-dR[1,2],dR[0,2]-dR[2,0],dR[1,0]-dR[0,1]])/2
        return J

    def sigma_min(self,model,q):
        """用雅可比的最小奇异值评估当前位置的奇异风险。

        参数：model 为 FKModel；q 为六关节角(rad)。
        返回：缩放雅可比的最小奇异值（无量纲）；越小越接近奇异。
        数值非有限时抛 ContractError(2002)。"""
        sigma=float(np.linalg.svd(self.jacobian(model,q),compute_uv=False)[-1])
        if not math.isfinite(sigma):raise ContractError(2002,'Nonfinite singularity estimate')
        return sigma

    @staticmethod
    def _pose_twist(current,target,scale):
        """将当前到目标的位姿差转换为阻尼迭代使用的六维残差。

        参数：current、target 为同系4×4矩阵；scale 为正的特征长度(m)。
        返回：[位置差/scale, 空间旋转对数]，形状(6,)；
        旋转差接近π时抛2003，避免采用不唯一的旋转方向。"""
        rotation=target[:3,:3] @ current[:3,:3].T
        skew=np.array([rotation[2,1]-rotation[1,2],rotation[0,2]-rotation[2,0],rotation[1,0]-rotation[0,1]])/2
        sine=float(np.linalg.norm(skew));angle=math.atan2(sine,float(np.clip((np.trace(rotation)-1)/2,-1,1)))
        if angle>math.pi-1e-4:raise ContractError(2003,'DLS orientation error near pi is ambiguous')
        angular=skew*(angle/sine if sine>1e-10 else 1.0)
        return np.r_[(target[:3,3]-current[:3,3])/scale,angular]

    def _dls_local(self,model,target,prev,lo,hi,cap,started):
        """在上一状态附近做有界阻尼最小二乘，寻找可回代的非奇异代表。

        参数：model/target 为模型和目标位姿；prev 为上一已接受q(rad)；
        lo/hi 为收缩限位；cap 为相对prev的逐轴最大变化(rad)；
        started 为整个select开始的单调时钟，用于共享总预算。
        返回：(solution, diagnostics)。solution仅在双误差、限位、限步与
        奇异路径保护通过时有效，否则为None；diagnostics记录迭代和拒绝原因。
        不会把部分迭代结果当作成功解，也不修改调用者状态。"""
        c=self.config;end=np.minimum(hi,prev+cap);begin=np.maximum(lo,prev-cap)
        q=prev.copy();history=[]
        def path_guard(candidate):
            """检查上一状态到试探解的腕奇异穿越及离散路径风险。

            参数：candidate 为试探的六关节角(rad)。
            返回：1/4、1/2、3/4及终点中的最小sigma；
            严格内部穿越theta5=kπ时返回None。采样不等于连续无奇异证明。"""
            t0=model.joint_sign[4]*prev[4]+model.theta_offset_rad[4]
            t1=model.joint_sign[4]*candidate[4]+model.theta_offset_rad[4]
            low,high=sorted((t0,t1));cross=(math.floor(low/math.pi)+1)*math.pi
            if low<cross<high:return None
            return min(self.sigma_min(model,prev+f*(candidate-prev)) for f in (.25,.5,.75,1.0))
        for iteration in range(c.dls_max_iterations+1):
            if time.monotonic()-started>=c.timeout_s:return None,dict(reason='timeout',iterations=iteration,history=history)
            current=self._fk(model,q);pe,re=pose_error(current,target)
            J=self.jacobian(model,q);s=float(np.linalg.svd(J,compute_uv=False)[-1])
            if not all(math.isfinite(x) for x in (pe,re,s)):
                return None,dict(reason='nonfinite',iterations=iteration,history=history)
            if pe<=c.position_tol_m and re<=c.orientation_tol_rad and s>c.sigma_hard:
                path_sigma=path_guard(q)
                if path_sigma is not None and path_sigma>c.sigma_hard:
                    return dict(q_rad=q.tolist(),mode='dls_near_singular',selected_candidate_index=None,position_error_m=pe,orientation_error_rad=re,sigma_min=s),dict(reason='accepted',iterations=iteration,path_sigma_min=path_sigma,history=history)
            if iteration==c.dls_max_iterations:break
            twist=self._pose_twist(current,target,c.characteristic_length_m)
            lam=c.dls_damping_min+(c.dls_damping_max-c.dls_damping_min)*max(0.0,1.0-s/c.sigma_soft)**2
            try:step=J.T @ np.linalg.solve(J@J.T+lam*lam*np.eye(6),twist)
            except np.linalg.LinAlgError:return None,dict(reason='linear_solve',iterations=iteration,history=history)
            if not np.all(np.isfinite(step)):return None,dict(reason='nonfinite_step',iterations=iteration,history=history)
            old=float(np.linalg.norm(twist));accepted=False
            for trial in range(c.dls_line_search_steps):
                if time.monotonic()-started>=c.timeout_s:return None,dict(reason='timeout',iterations=iteration,history=history)
                proposal=np.clip(q+step*(0.5**trial),begin,end)
                if np.array_equal(proposal,q):continue
                trial_sigma=path_guard(proposal)
                if trial_sigma is None or trial_sigma<=c.sigma_hard:continue
                new=float(np.linalg.norm(self._pose_twist(self._fk(model,proposal),target,c.characteristic_length_m)))
                if math.isfinite(new) and new<old-1e-14:
                    q=proposal;history.append(dict(damping=lam,residual=new,step_scale=0.5**trial));accepted=True;break
            if not accepted:break
        return None,dict(reason='not_converged',iterations=len(history),history=history)

    def select(self,profile,target,state):
        """依据上一状态，对解析候选评分并实施连续性和奇异保护。

        参数：profile 为工程模型字典；target 为 base→tool0 的4×4目标；
        state 必含 q_rad[6](rad) 和非负整数 step_index。
        返回：code=0时data含关节解、模式、回代误差和sigma，next_state为
        新q及步号+1；失败时data/next_state均为None，details保留原因。
        输入state永不修改。解析奇异失败可尝试有界DLS；失败后调用方应
        停止原段，从保留状态另行重规划。此接口不向机器人发送命令。"""
        c=self.config;diagnostics=[];started=time.monotonic()
        def fail(code,message,**details):
            """构造不推进状态的失败返回。

            参数：code/message为统一码和说明；details为诊断字段。
            返回：data及next_state均为None的Result，附已检查候选诊断。"""
            return dict(code=code,message=message,data=None,next_state=None,details=dict(candidates=diagnostics,**details))
        try:
            c.validate();model=pack_model(profile)
            if not isinstance(state,dict) or 'q_rad' not in state or type(state.get('step_index')) is not int or state['step_index']<0:
                raise ContractError(1001,'state requires q_rad[6] and nonnegative integer step_index')
            prev=np.array(numbers(state['q_rad'],6),dtype=float)
            target=np.array([numbers(row,4) for row in target],dtype=float)
            if target.shape!=(4,4):raise ContractError(1001,'Target must be 4x4')
            # Validate rigid target even if a singular hold could be used.
            inv=D16();fn=self.forward.lib.robot_transform_inverse
            fn.argtypes=[C.POINTER(C.c_double),C.c_double,C.POINTER(C.c_double)];fn.restype=C.c_int
            if fn(D16(*target.ravel()),model.pose_validation_tol,inv):raise ContractError(1001,'Invalid target transform')
            speed=np.array(numbers(profile['qd_max_rad_s'],6),dtype=float)
            if np.any(speed<=0):raise ContractError(1001,'Positive velocity limits required')
            lo=np.array(model.q_min_rad)+c.joint_margin_rad;hi=np.array(model.q_max_rad)-c.joint_margin_rad
            if np.any(lo>hi):raise ContractError(1001,'Empty margin-shrunk joint range')
            if np.any(prev<lo) or np.any(prev>hi):raise ContractError(1005,'Previous state outside configured limits')
            prevT=self._fk(model,prev);previous_sigma=self.sigma_min(model,prev)
            base_cap=np.minimum(c.max_step_rad,speed*c.dt_s)
            if not np.all(np.isfinite(base_cap)) or np.any(base_cap<=0):raise ContractError(1001,"Invalid discrete step cap")
            if time.monotonic()-started>=c.timeout_s:return fail(2002,"Selection time budget exhausted")
            p0,r0=pose_error(prevT,target)
            if previous_sigma<=c.sigma_hard and p0<=c.position_tol_m and r0<=c.orientation_tol_rad:
                q=prev.tolist()
                return dict(code=0,message='Hold existing singular state; target already satisfied',data=dict(q_rad=q,mode='singular_hold',selected_candidate_index=None,position_error_m=p0,orientation_error_rad=r0,sigma_min=previous_sigma),next_state=dict(q_rad=q.copy(),step_index=state['step_index']+1),details=dict(candidates=[],previous_sigma_min=previous_sigma,step_cap_rad=base_cap.tolist(),hold=True))
            options=IKOptionsV1(1,1,c.position_tol_m,c.orientation_tol_rad,c.joint_margin_rad,max(1e-15,c.timeout_s-(time.monotonic()-started)),0,0,0,0,0)
            out=IKResultV1();code=self.inverse(C.byref(model),D16(*target.ravel()),16,D6(*prev),6,C.byref(options),C.byref(out))
            if code:
                if code==2003 and c.dls_enabled:
                    cap=base_cap*(c.near_step_scale if previous_sigma<c.sigma_soft else 1.0)
                    solution,dls=self._dls_local(model,target,prev,lo,hi,cap,started)
                    if solution is not None:
                        return dict(code=0,message='Bounded DLS fallback passed FK and singular guards',data=solution,next_state=dict(q_rad=solution['q_rad'].copy(),step_index=state['step_index']+1),details=dict(candidates=[],previous_sigma_min=previous_sigma,step_cap_rad=cap.tolist(),dls=dls,analytic_code=code))
                    return fail(2002 if dls['reason']=='timeout' else 2003,'Analytic IK singular; bounded DLS did not produce a safe solution',previous_sigma_min=previous_sigma,dls=dls,analytic_code=code)
                return fail(code,'C candidate generation failed; state not advanced',previous_sigma_min=previous_sigma)
            eligible=[]
            for k in range(out.solution_count):
                if time.monotonic()-started>=c.timeout_s:return fail(2002,"Selection time budget exhausted")
                q=np.array(out.candidates_rad[k]);delta=q-prev;back=self._fk(model,q);pe,re=pose_error(back,target)
                sigma=self.sigma_min(model,q);margin=float(np.min(np.minimum(q-lo,hi-q)))
                near=min(previous_sigma,sigma)<c.sigma_soft
                cap=base_cap*(c.near_step_scale if near else 1.0)
                travel=float(np.mean((delta/base_cap)**2))
                limit_cost=(c.limit_scale_rad/max(margin,1e-12))**2
                singular_cost=(c.sigma_soft/max(sigma,1e-12))**2
                cost=c.travel_weight*travel+c.limit_weight*limit_cost+c.singular_weight*singular_cost
                reason='accepted'
                if not all(math.isfinite(v) for v in (pe,re,sigma,margin,cost)):reason='nonfinite'
                elif margin<0:reason='joint_limit'
                elif pe>c.position_tol_m or re>c.orientation_tol_rad:reason='fk_residual'
                elif sigma<=c.sigma_hard:reason='singular_protection'
                elif np.any(np.abs(delta)>cap+1e-12):reason='continuity_step'
                path_sigma_min=min(previous_sigma,sigma)
                if reason=='accepted':
                    # The UR5 wrist is exactly singular at theta5=k*pi. Detect
                    # every strict interior crossing, not just sampled points.
                    t0=model.joint_sign[4]*prev[4]+model.theta_offset_rad[4]
                    t1=model.joint_sign[4]*q[4]+model.theta_offset_rad[4]
                    low,high=sorted((t0,t1))
                    crossing=(math.floor(low/math.pi)+1)*math.pi
                    if low<crossing<high:reason='wrist_singularity_crossing'
                    else:
                        samples=[self.sigma_min(model,prev+fraction*delta) for fraction in (.25,.5,.75)]
                        path_sigma_min=min([path_sigma_min]+samples)
                        if min(samples)<=c.sigma_hard:reason='sampled_path_singularity'
                        elif path_sigma_min<c.sigma_soft:
                            cap=base_cap*c.near_step_scale
                            if np.any(np.abs(delta)>cap+1e-12):reason='continuity_step'
                item=dict(candidate_index=k,q_rad=q.tolist(),position_error_m=pe,orientation_error_rad=re,fk_passed=pe<=c.position_tol_m and re<=c.orientation_tol_rad,delta_rad=delta.tolist(),limit_distance_rad=margin,sigma_min=sigma,path_sigma_min=path_sigma_min,travel_cost=travel,limit_cost=limit_cost,singular_cost=singular_cost,total_cost=cost,step_cap_rad=cap.tolist(),eligible=reason=='accepted',reason=reason)
                diagnostics.append(item)
                if item['eligible']:eligible.append(item)
            if time.monotonic()-started>=c.timeout_s:return fail(2002,"Selection time budget exhausted")
            if not eligible:
                singular_block=any(d['reason'] in ('singular_protection','wrist_singularity_crossing','sampled_path_singularity') for d in diagnostics)
                if singular_block and c.dls_enabled:
                    cap=base_cap*(c.near_step_scale if previous_sigma<c.sigma_soft else 1.0)
                    solution,dls=self._dls_local(model,target,prev,lo,hi,cap,started)
                    if solution is not None:
                        return dict(code=0,message='Bounded DLS fallback passed FK and singular guards',data=solution,next_state=dict(q_rad=solution['q_rad'].copy(),step_index=state['step_index']+1),details=dict(candidates=diagnostics,previous_sigma_min=previous_sigma,step_cap_rad=cap.tolist(),dls=dls))
                    return fail(2002 if dls['reason']=='timeout' else 2003,'No candidate passed singular guards or bounded DLS',previous_sigma_min=previous_sigma,dls=dls)
                return fail(2003 if any(d['reason'] in ('singular_protection','wrist_singularity_crossing','sampled_path_singularity') for d in diagnostics) else 2002,'No candidate satisfies selection guards',previous_sigma_min=previous_sigma)
            chosen=min(eligible,key=lambda d:(d['total_cost'],tuple(d['q_rad'])))
            mode='near_singular_restricted' if chosen['path_sigma_min']<c.sigma_soft else 'regular'
            return dict(code=0,message='OK',data=dict(q_rad=chosen['q_rad'].copy(),mode=mode,selected_candidate_index=chosen['candidate_index'],position_error_m=chosen['position_error_m'],orientation_error_rad=chosen['orientation_error_rad'],sigma_min=chosen['sigma_min']),next_state=dict(q_rad=chosen['q_rad'].copy(),step_index=state['step_index']+1),details=dict(candidates=diagnostics,previous_sigma_min=previous_sigma,weights=dict(travel=c.travel_weight,limit=c.limit_weight,singular=c.singular_weight)))
        except KeyError as exc:return fail(1004,'Missing model/state field: '+str(exc))
        except (ContractError,ValueError,TypeError,OverflowError,np.linalg.LinAlgError) as exc:return fail(getattr(exc,'code',1001),str(exc))
