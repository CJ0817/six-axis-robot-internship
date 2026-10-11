"""静态关节空间人工势场基线；全连杆距离势能，显式失败，不保证全局最优。"""
import math
import numpy as np
from adapters.c_kinematics import numbers, ContractError


def static_apf(planner,start,goal,scene):
    """静态势场寻找关节途经点；返回统一Result及全部势能迭代。

    入参：CartesianPlanner、起终q(6,)rad、StaticScene。吸引势0.5*k*||q-goal||²，
    斥力势0.5*eta*(1/d-1/rho)²*||q-goal||²，d为全机器人最近距离扣除clearance。
    目标距离衰减避免目标附近斥力导致静态目标无法收敛；仍可能陷入局部极小。
    数值中心差分求6轴梯度；回溯每一步细分碰撞检查、必须降低势能。
    达到容差后追加精确终点。堵塞端点3003，局部极小/迭代预算3001，
    失败data=None且不提供部分可执行轨迹；历史仅供诊断。
    """
    history=[];retained=None
    try:
        planner.setup();c=planner.config['apf'];keys=('attraction_gain','repulsion_gain','influence_distance_m',
            'gradient_step_rad','maximum_step_rad','goal_tolerance_rad','maximum_iterations','line_search_steps')
        if any(k not in c or c[k] is None for k in keys):raise ContractError(1004,'Missing APF settings')
        if any(type(c[k]) not in (int,float) or not math.isfinite(c[k]) or c[k]<=0 for k in keys):raise ContractError(1001,'Invalid APF settings')
        if any(type(c[k]) is not int for k in ('maximum_iterations','line_search_steps')):raise ContractError(1001,'Invalid APF budget')
        if c['influence_distance_m']>scene.query-scene.clearance:raise ContractError(1001,'APF influence exceeds distance query')
        q=np.array(numbers(start,6),float);goal=np.array(numbers(goal,6),float)
        lo=np.array(planner.profile['q_min_rad']);hi=np.array(planner.profile['q_max_rad'])
        if np.any(q<lo) or np.any(q>hi) or np.any(goal<lo) or np.any(goal>hi):raise ContractError(1005,'APF endpoint outside joint range')
        retained=q.tolist()
        if not scene.check(q.tolist())['passed'] or not scene.check(goal.tolist())['passed']:raise ContractError(3003,'APF start/goal blocked')
        def potential(x):
            check=scene.check(x.tolist());distance=check['minimum_distance_lower_bound_m']-scene.clearance
            if distance<=0:return math.inf,check,0.
            attraction=.5*c['attraction_gain']*float(np.dot(x-goal,x-goal))
            repulsion=.5*c['repulsion_gain']*(1/distance-1/c['influence_distance_m'])**2*float(np.dot(x-goal,x-goal)) if distance<c['influence_distance_m'] else 0.
            return attraction+repulsion,check,repulsion
        def edge_safe(a,b):
            steps=max(2,math.ceil(float(np.max(abs(b-a)))/planner.config['collision_max_joint_step_rad']))
            return all(scene.check((a+(b-a)*s/steps).tolist())['passed'] for s in range(1,steps+1))
        path=[q.tolist()]
        for iteration in range(c['maximum_iterations']):
            value,check,repulsion=potential(q);error=float(np.max(abs(q-goal)))
            history.append(dict(iteration=iteration,q_rad=q.tolist(),potential=value,repulsion=repulsion,
                goal_error_rad=error,minimum_distance_lower_bound_m=check['minimum_distance_lower_bound_m']))
            if error<=c['goal_tolerance_rad']:
                if not edge_safe(q,goal):raise ContractError(3003,'APF final edge blocked')
                if np.max(abs(q-goal))>0:path.append(goal.tolist())
                return dict(code=0,message='APF goal reached',data=dict(waypoints_rad=path,iterations=history),details=dict(scene=scene.metadata()))
            gradient=np.empty(6)
            for i in range(6):
                h=min(c['gradient_step_rad'],q[i]-lo[i],hi[i]-q[i])
                if h<=1e-12:raise ContractError(3001,'APF finite difference blocked at joint bound')
                left=q.copy();right=q.copy();left[i]-=h;right[i]+=h
                gradient[i]=(potential(right)[0]-potential(left)[0])/(2*h)
            norm=float(np.linalg.norm(gradient))
            if not math.isfinite(norm) or norm<1e-12:raise ContractError(3001,'APF local minimum/nonfinite gradient')
            step=-gradient/norm*min(c['maximum_step_rad'],float(np.linalg.norm(q-goal)))
            accepted=False
            for power in range(c['line_search_steps']):
                candidate=q+step*2**(-power)
                if np.any(candidate<lo) or np.any(candidate>hi):continue
                energy=potential(candidate)[0]
                if energy<value-1e-14 and edge_safe(q,candidate):q=candidate;accepted=True;break
            if not accepted:raise ContractError(3001,'APF local minimum/backtracking exhausted')
            path.append(q.tolist());retained=q.tolist()
        raise ContractError(3001,'APF iteration budget exhausted')
    except (ContractError,KeyError,ValueError,ArithmeticError) as exc:
        return dict(code=getattr(exc,'code',1004 if isinstance(exc,KeyError) else 1001),message=str(exc),data=None,
            details=dict(iterations=history,last_valid_q_rad=retained,stop_segment=True,replan_required=True))


def time_parameterize(planner,waypoints,scene):
    """势场途经点按每段静止五次同步定时，固定周期拼接，再整体认证。

    每段保守放慢，节点数有硬上限；成功含限值/碰撞证据，失败不输出部分轨迹。
    这是初期可重复的停顿基线，不宣称不停顿/最优节拍。
    """
    try:
        planner.setup();dt=planner.config['sample_period_s'];samples=[]
        for start,goal in zip(waypoints[:-1],waypoints[1:]):
            start=np.array(numbers(start,6));goal=np.array(numbers(goal,6));delta=goal-start
            v=np.array(planner.limits.joint_velocity_rad_s);a=np.array(planner.limits.joint_acceleration_rad_s2)
            duration=max(.5,float(np.max(1.875*abs(delta)/v)),float(np.max(np.sqrt((10/math.sqrt(3))*abs(delta)/a))))*2
            n=math.ceil(duration/dt)
            if len(samples)+n+1>planner.config['maximum_nodes']:raise ContractError(3001,'APF time/node budget exhausted')
            for k in range(n):
                u=k/n;s=u**3*(10-15*u+6*u*u);samples.append((start+delta*s).tolist())
        samples.append(list(waypoints[-1]))
        data=planner.certify(samples,scene);data['sample_period_s']=dt;data['method']='static_joint_apf_rest_segments'
        return dict(code=0,message='OK',data=data,details={})
    except (ContractError,ValueError,ArithmeticError,IndexError) as exc:
        return dict(code=getattr(exc,'code',1001),message=str(exc),data=None,details=dict(stop_segment=True,replan_required=True))
