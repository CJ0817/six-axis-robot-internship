"""有界子目标势场、几何平滑与固定周期缩时；所有输出重新经过共同验收。"""
import copy
import json
import math
from pathlib import Path
import numpy as np
from adapters.c_kinematics import ContractError, numbers
from planning.apf import static_apf
from planning.cartesian import evaluate

ROOT=Path(__file__).resolve().parents[2]
DEFAULT=ROOT/'config/guided-apf.v1.json'


def settings(path=DEFAULT):
    """读取有版本的仿真引导配置；参数缺失1004、非法1001，不替代共享运动限值。"""
    try:c=json.loads(Path(path).read_text())
    except OSError as exc:raise ContractError(1004,'Missing guided APF configuration') from exc
    required=('version','usage','guide_fractions','guide_offsets_rad','maximum_candidates',
        'smoothing_passes','smoothing_weight','edge_maximum_subdivisions','compression_factor')
    if any(k not in c or c[k] is None for k in required):raise ContractError(1004,'Incomplete guided APF configuration')
    if c['version']!='1.0.0' or c['usage']!='simulation_only':raise ContractError(1008,'Unsupported guided APF configuration')
    for key in ('maximum_candidates','smoothing_passes','edge_maximum_subdivisions'):
        if type(c[key]) is not int or c[key]<=0:raise ContractError(1001,'Invalid integer guide budget')
    for key in ('smoothing_weight','compression_factor'):
        if type(c[key]) not in (float,int) or not math.isfinite(c[key]) or not 0<c[key]<1:raise ContractError(1001,'Guide weight/factor must be in (0,1)')
    if not isinstance(c['guide_fractions'],list) or not c['guide_fractions'] or any(type(x) not in (float,int) or not math.isfinite(x) or not 0<x<1 for x in c['guide_fractions']):raise ContractError(1001,'Invalid guide fractions')
    if not isinstance(c['guide_offsets_rad'],list) or not c['guide_offsets_rad']:raise ContractError(1001,'Missing guide candidates')
    c['guide_offsets_rad']=[numbers(x,6) for x in c['guide_offsets_rad']]
    return c


def edge_check(planner,scene,start,goal,maximum_subdivisions=4096):
    """检查关节直线的全臂/自碰撞和连续净距界。

    入参两个6维rad节点及细分预算。返回passed、原因、细分数和净距下界；
    用固定URDF运动半径覆盖任意中间点，预算不足也拒绝，不能视作无碰撞。
    """
    a=np.array(numbers(start,6));b=np.array(numbers(goal,6));delta=b-a
    lo=np.array(planner.profile['q_min_rad']);hi=np.array(planner.profile['q_max_rad'])
    if np.any(np.minimum(a,b)<lo) or np.any(np.maximum(a,b)>hi):return dict(passed=False,code=1005,reason='Joint range exceeded')
    count=max(2,math.ceil(float(np.max(abs(delta)))/planner.config['collision_max_joint_step_rad']))
    checks=0
    while count<=maximum_subdivisions:
        distances=[]
        for k in range(count+1):
            collision=scene.check((a+delta*k/count).tolist());checks+=1
            if not collision['passed']:return dict(passed=False,code=3003,reason='Whole-arm/self collision',fraction=k/count,collision=collision,checks=checks)
            distances.append(collision['minimum_distance_lower_bound_m'])
        # 最近端半间隔、自碰撞相对点运动的2倍系数相互抵消。
        bound=scene.motion_radius_bound_m*float(np.sum(abs(delta)))/count
        certified=min(distances)-bound
        if certified>scene.clearance:return dict(passed=True,code=0,subdivisions=count,checks=checks,
            sampled_minimum_distance_m=min(distances),certified_minimum_distance_m=certified,motion_bound_m=bound)
        count*=2
    return dict(passed=False,code=3001,reason='Continuous edge clearance proof budget exhausted',checks=checks)


def guided_apf(planner,start,goal,scene,config_path=DEFAULT):
    """先尝试原势场；失败后按固定顺序尝试中间子目标，拼接两段真实APF路径。

    入参与基线相同，额外为版本化引导配置。子目标来自起终插值加6轴偏移，
    每候选从原起点重启，不能续用失败状态；逐候选保存两段返回码/迭代和拒绝原因。
    成功只返回几何waypoints，尚不可执行；失败data/next_state=null，保留原起点。
    """
    attempts=[];initial=None
    try:
        planner.setup();c=settings(config_path);initial=list(numbers(start,6));end=list(numbers(goal,6))
        direct=static_apf(planner,initial,end,scene)
        attempts.append(dict(candidate_index=0,kind='direct',result=direct))
        if direct['code']==0:return dict(code=0,message='Direct APF path',data=direct['data'],next_state=None,
            details=dict(attempts=attempts,used_subgoal=False,configuration=c))
        if direct['code']!=3001:raise ContractError(direct['code'],direct['message'])
        lo=np.array(planner.profile['q_min_rad']);hi=np.array(planner.profile['q_max_rad']);index=0
        for fraction in c['guide_fractions']:
            for offset in c['guide_offsets_rad']:
                if index>=c['maximum_candidates']:raise ContractError(3001,'Subgoal candidate budget exhausted')
                index+=1;guide=(np.array(initial)*(1-fraction)+np.array(end)*fraction+offset).tolist()
                attempt=dict(candidate_index=index,kind='subgoal',fraction=fraction,subgoal_rad=guide,stages=[]);attempts.append(attempt)
                if np.any(np.array(guide)<lo) or np.any(np.array(guide)>hi):attempt.update(code=1005,reason='Subgoal outside joint range');continue
                collision=scene.check(guide)
                if not collision['passed']:attempt.update(code=3003,reason='Subgoal whole-arm/self collision',collision=collision);continue
                first=static_apf(planner,initial,guide,scene);attempt['stages'].append(first)
                if first['code']!=0:attempt.update(code=first['code'],reason='First subgoal stage: '+first['message']);continue
                second=static_apf(planner,guide,end,scene);attempt['stages'].append(second)
                if second['code']!=0:attempt.update(code=second['code'],reason='Second subgoal stage: '+second['message']);continue
                attempt.update(code=0,reason='Two subgoal stages reached')
                return dict(code=0,message='Subgoal-guided APF path',next_state=None,
                    data=dict(waypoints_rad=first['data']['waypoints_rad']+second['data']['waypoints_rad'][1:]),
                    details=dict(attempts=attempts,used_subgoal=True,selected_candidate=index,configuration=c))
        raise ContractError(3001,'No admissible subgoal route')
    except (ContractError,ValueError,TypeError,KeyError,ArithmeticError) as exc:
        return dict(code=getattr(exc,'code',1004 if isinstance(exc,KeyError) else 1001),message=str(exc),data=None,next_state=None,
            details=dict(attempts=attempts,retained_input_q_rad=initial,stop_segment=True,replan_required=True))


def smooth_waypoints(planner,waypoints,scene,config_path=DEFAULT):
    """以净距认证的拉普拉斯移动和确定性捷径减少折线弯折/冗余节点。

    入参N×6 rad几何途经点；端点不变，每次修改两相邻边及捷径都检查连续碰撞。
    返回几何路径和所有接受/拒绝记录；几何路径不是可执行轨迹，须随后时间参数化。
    """
    events=[];input_checks=[]
    try:
        planner.setup();c=settings(config_path);path=[list(numbers(q,6)) for q in waypoints]
        if len(path)<2:raise ContractError(1001,'At least two waypoints required')
        for i in range(len(path)-1):
            proof=edge_check(planner,scene,path[i],path[i+1],c['edge_maximum_subdivisions'])
            input_checks.append(dict(edge_index=i,proof=proof))
            if not proof['passed']:raise ContractError(proof['code'],'Input smoothing edge '+str(i)+': '+proof['reason'])
        for pass_index in range(c['smoothing_passes']):
            for i in range(1,len(path)-1):
                previous=np.array(path[i]);candidate=(previous*(1-c['smoothing_weight'])+
                    .5*c['smoothing_weight']*(np.array(path[i-1])+np.array(path[i+1]))).tolist()
                proofs=[edge_check(planner,scene,a,b,c['edge_maximum_subdivisions']) for a,b in ((path[i-1],candidate),(candidate,path[i+1]))]
                accepted=all(p['passed'] for p in proofs)
                events.append(dict(kind='laplacian',pass_index=pass_index,node_index=i,accepted=accepted,proofs=proofs))
                if accepted:path[i]=candidate
        output=[path[0]];i=0
        while i<len(path)-1:
            accepted=False
            for j in range(len(path)-1,i,-1):
                proof=edge_check(planner,scene,path[i],path[j],c['edge_maximum_subdivisions'])
                events.append(dict(kind='shortcut',start_index=i,end_index=j,accepted=proof['passed'],proof=proof))
                if proof['passed']:output.append(path[j]);i=j;accepted=True;break
            if not accepted:raise ContractError(3001,'No continuously certified smoothing edge')
        return dict(code=0,message='Geometric smoothing passed',data=dict(waypoints_rad=output),next_state=None,
            details=dict(input_node_count=len(waypoints),output_node_count=len(output),input_edge_checks=input_checks,events=events,configuration=c))
    except (ContractError,ValueError,TypeError,KeyError,ArithmeticError) as exc:
        return dict(code=getattr(exc,'code',1004 if isinstance(exc,KeyError) else 1001),message=str(exc),data=None,next_state=None,
            details=dict(input_edge_checks=input_checks,events=events,stop_segment=True,replan_required=True))


def retime_trajectory(planner,trajectory,scene,factor):
    """固定0.02s周期缩时并重新拟合/认证整条C2轨迹，不能只缩放时间戳。

    factor为(0,1]请求比例；实际总时长向上取周期整数倍。输入为已认证轨迹，
    重采样后重验关节位置/速度/加速度、TCP向量模长及整臂/自碰撞连续界。
    超限3002、碰撞3003、证明/预算不足3001；失败不给可执行数据，原轨迹不变。
    """
    try:
        planner.setup()
        if type(factor) not in (float,int) or not math.isfinite(factor) or not 0<factor<=1:raise ContractError(1001,'Retime factor must be in (0,1]')
        dt=planner.config['sample_period_s'];old_dt=trajectory['sample_period_s']
        co=np.array(trajectory['coefficients_rad'],float);old_T=len(co)*old_dt
        n=max(2,math.ceil(old_T*factor/dt));new_T=n*dt;actual=new_T/old_T
        if new_T>planner.config['maximum_duration_s'] or n+1>planner.config['maximum_nodes']:raise ContractError(3001,'Retime duration/node budget exhausted')
        q=[evaluate(co,old_dt,k*old_T/n)[0].tolist() for k in range(n+1)]
        data=planner.certify(q,scene);data.update(sample_period_s=dt,method='guided_apf_smoothed_retimed')
        return dict(code=0,message='Retimed trajectory fully certified',data=data,next_state=None,
            details=dict(requested_factor=factor,actual_factor=actual,input_duration_s=old_T,actual_duration_s=new_T,
                sampled_and_refitted=True,revalidated=['joint_position','joint_velocity','joint_acceleration','tcp_linear_angular_rates','whole_arm','self_collision','continuous_clearance']))
    except (ContractError,ValueError,TypeError,KeyError,ArithmeticError) as exc:
        return dict(code=getattr(exc,'code',1004 if isinstance(exc,KeyError) else 1001),message=str(exc),data=None,next_state=None,
            details=dict(requested_factor=factor,stop_segment=True,replan_required=True))


def parameterize_smoothed(planner,waypoints,scene,compression_factor):
    """为平滑几何路径生成静止五次段，并为随后缩时预留共享TCP/关节限值裕度。

    入参N×6 rad途经点、场景和预期缩时比例。用DH保守半径推导时长下界，
    预留10%速度裕度；实际C2拟合仍必须certify。时长/节点预算不足3001，无输出。
    """
    try:
        planner.setup();dt=planner.config['sample_period_s'];limits=planner.limits
        if type(compression_factor) not in (float,int) or not math.isfinite(compression_factor) or not 0<compression_factor<=1:raise ContractError(1001,'Invalid planned compression factor')
        if len(waypoints)<2:raise ContractError(1001,'At least two smoothed waypoints required')
        radii=np.cumsum(np.hypot(planner.profile['a_m'],planner.profile['d_m'])[::-1])[::-1]
        for key in ('T_dh6_flange','T_flange_tool'):radii+=np.linalg.norm(np.array(planner.profile[key])[:3,3])
        samples=[];durations=[]
        for start,goal in zip(waypoints[:-1],waypoints[1:]):
            start=np.array(numbers(start,6));goal=np.array(numbers(goal,6));delta=abs(goal-start)
            V=1.875*delta;A=10/math.sqrt(3)*delta;linear=float(radii@V);angular=float(sum(V))
            minimum=max(.5,float(np.max(V/limits.joint_velocity_rad_s)),
                float(np.max(np.sqrt(A/limits.joint_acceleration_rad_s2))),
                linear/limits.linear_velocity_m_s,angular/limits.angular_velocity_rad_s,
                math.sqrt((float(radii@A)+2*linear*angular)/limits.linear_acceleration_m_s2),
                math.sqrt((float(sum(A))+angular**2)/limits.angular_acceleration_rad_s2))
            n=math.ceil(minimum*1.1/compression_factor/dt);durations.append(n*dt)
            if sum(durations)>planner.config['maximum_duration_s'] or len(samples)+n+1>planner.config['maximum_nodes']:raise ContractError(3001,'Smoothed trajectory duration/node budget exhausted')
            for k in range(n):
                u=k/n;s=u**3*(10-15*u+6*u*u);samples.append((start+(goal-start)*s).tolist())
        samples.append(list(waypoints[-1]));data=planner.certify(samples,scene)
        data.update(sample_period_s=dt,method='guided_apf_smoothed_before_compression')
        return dict(code=0,message='Smoothed path timed and certified',data=data,next_state=None,
            details=dict(rest_segment_durations_s=durations,planned_compression_factor=compression_factor,derivative_margin=1.1))
    except (ContractError,ValueError,TypeError,KeyError,ArithmeticError) as exc:
        return dict(code=getattr(exc,'code',1004 if isinstance(exc,KeyError) else 1001),message=str(exc),data=None,next_state=None,
            details=dict(stop_segment=True,replan_required=True))


def improved_plan(planner,start,goal,scene,config_path=DEFAULT):
    """依次引导、平滑、原速认证、缩时认证；成功才交付可执行轨迹。

    返回Result；details包含各阶段原始结果（含候选失败），失败不静默退回未经认证路径。
    """
    stages={}
    try:
        c=settings(config_path)
        stages['guided']=guided_apf(planner,start,goal,scene,config_path)
        if stages['guided']['code']:raise ContractError(stages['guided']['code'],stages['guided']['message'])
        stages['smoothing']=smooth_waypoints(planner,stages['guided']['data']['waypoints_rad'],scene,config_path)
        if stages['smoothing']['code']:raise ContractError(stages['smoothing']['code'],stages['smoothing']['message'])
        stages['before_compression']=parameterize_smoothed(planner,stages['smoothing']['data']['waypoints_rad'],scene,c['compression_factor'])
        if stages['before_compression']['code']:raise ContractError(stages['before_compression']['code'],stages['before_compression']['message'])
        stages['compression']=retime_trajectory(planner,stages['before_compression']['data'],scene,c['compression_factor'])
        if stages['compression']['code']:raise ContractError(stages['compression']['code'],stages['compression']['message'])
        return dict(code=0,message='Guided smoothed retimed path certified',data=stages['compression']['data'],next_state=None,details=dict(stages=stages))
    except (ContractError,ValueError,TypeError,KeyError,ArithmeticError) as exc:
        return dict(code=getattr(exc,'code',1004 if isinstance(exc,KeyError) else 1001),message=str(exc),data=None,next_state=None,
            details=dict(stages=stages,retained_input_q_rad=copy.deepcopy(start),stop_segment=True,replan_required=True))
