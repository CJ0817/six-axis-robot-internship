"""State-aware selection above frozen C ABI v1; pure planning, no actuation."""
import ctypes as C
from dataclasses import dataclass, asdict
import math
import time
import numpy as np
from .c_kinematics import Forward, FKModel, IKOptionsV1, IKResultV1, D6, D16, pack_model, numbers, ContractError

@dataclass(frozen=True)
class SelectionConfig:
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

    def validate(self):
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in asdict(self).values()):
            raise ContractError(1001,'Config must contain finite numbers')
        if any(v<0 for v in (self.travel_weight,self.limit_weight,self.singular_weight,self.joint_margin_rad)) or self.travel_weight+self.limit_weight+self.singular_weight<=0:
            raise ContractError(1001,'Invalid weights/margin')
        if any(v<=0 for v in (self.max_step_rad,self.dt_s,self.limit_scale_rad,self.sigma_hard,self.characteristic_length_m,self.difference_step_rad,self.position_tol_m,self.orientation_tol_rad,self.timeout_s)):
            raise ContractError(1001,'Positive configuration required')
        if not self.sigma_hard<self.sigma_soft or not 0<self.near_step_scale<=1 or self.orientation_tol_rad>math.pi or self.difference_step_rad>1e-3:
            raise ContractError(1001,'Invalid singular/step/angle thresholds')

def pose_error(a,b):
    R=a[:3,:3].T @ b[:3,:3]
    v=np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/2
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),math.atan2(float(np.linalg.norm(v)),float(np.clip((np.trace(R)-1)/2,-1,1)))

class StatefulIKSelector:
    def __init__(self,library,config=None):
        self.config=config or SelectionConfig()
        self.forward=Forward(library)
        self.inverse=self.forward.lib.robot_inverse_v1
        self.inverse.restype=C.c_int
        self.inverse.argtypes=[C.POINTER(FKModel),C.POINTER(C.c_double),C.c_size_t,C.POINTER(C.c_double),C.c_size_t,C.POINTER(IKOptionsV1),C.POINTER(IKResultV1)]

    def _fk(self,model,q):
        out=D16();code=self.forward.fn(C.byref(model),D6(*q),6,out)
        if code:raise ContractError(code,'C FK rejected selection input')
        return np.array(list(out)).reshape(4,4)

    def sigma_min(self,model,q):
        """Scaled spatial numerical Jacobian; central or one-sided at limits."""
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
        sigma=float(np.linalg.svd(J,compute_uv=False)[-1])
        if not math.isfinite(sigma):raise ContractError(2002,'Nonfinite singularity estimate')
        return sigma

    def select(self,profile,target,state):
        """Return code/data/next_state/details; never mutate the previous state."""
        c=self.config;diagnostics=[];started=time.monotonic()
        def fail(code,message,**details):
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
            if code:return fail(code,'C candidate generation failed; state not advanced',previous_sigma_min=previous_sigma)
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
                return fail(2003 if any(d['reason'] in ('singular_protection','wrist_singularity_crossing','sampled_path_singularity') for d in diagnostics) else 2002,'No candidate satisfies selection guards',previous_sigma_min=previous_sigma)
            chosen=min(eligible,key=lambda d:(d['total_cost'],tuple(d['q_rad'])))
            mode='near_singular_restricted' if chosen['path_sigma_min']<c.sigma_soft else 'regular'
            return dict(code=0,message='OK',data=dict(q_rad=chosen['q_rad'].copy(),mode=mode,selected_candidate_index=chosen['candidate_index'],position_error_m=chosen['position_error_m'],orientation_error_rad=chosen['orientation_error_rad'],sigma_min=chosen['sigma_min']),next_state=dict(q_rad=chosen['q_rad'].copy(),step_index=state['step_index']+1),details=dict(candidates=diagnostics,previous_sigma_min=previous_sigma,weights=dict(travel=c.travel_weight,limit=c.limit_weight,singular=c.singular_weight)))
        except KeyError as exc:return fail(1004,'Missing model/state field: '+str(exc))
        except (ContractError,ValueError,TypeError,OverflowError,np.linalg.LinAlgError) as exc:return fail(getattr(exc,'code',1001),str(exc))
