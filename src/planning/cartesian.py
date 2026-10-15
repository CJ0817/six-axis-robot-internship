"""C几何插值、SO(3)姿态、固定周期连续IK及整段关节/离散TCP碰撞验收。"""
import copy
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from adapters.c_kinematics import ContractError, numbers, pack_model
from adapters.ik_selection import StatefulIKSelector, SelectionConfig, pose_error
from planning.limits import load_planning_limits, DEFAULT_CONFIG, DEFAULT_MODEL
ROOT=Path(__file__).resolve().parents[2]


def configuration(path=ROOT/'config/cartesian-planning.v1.json'):
    """读取版本化算法/检查预算；缺项1004、非法值1001，运动限值不在此重复定义。"""
    try:c=json.loads(Path(path).read_text())
    except OSError as exc:raise ContractError(1004,'Missing Cartesian configuration') from exc
    required=('sample_period_s','maximum_duration_s','maximum_nodes','collision_subdivisions',
        'collision_max_joint_step_rad','position_tolerance_m','orientation_tolerance_rad',
        'derivative_step_s','ik_position_tolerance_m','ik_orientation_tolerance_rad','ik_timeout_s')
    if any(k not in c or c[k] is None for k in required) or 'apf' not in c:raise ContractError(1004,'Incomplete Cartesian configuration')
    if c.get('version')!='1.0.0' or c.get('usage')!='simulation_only':raise ContractError(1008,'Unsupported configuration')
    if any(type(c[k]) not in (int,float) or not math.isfinite(c[k]) or c[k]<=0 for k in required):raise ContractError(1001,'Positive Cartesian settings required')
    if any(type(c[k]) is not int for k in ('maximum_nodes','collision_subdivisions')):raise ContractError(1001,'Integer budget required')
    return c


def rigid(T):
    """入参4x4有限base→tool0变换；返回NumPy矩阵，非法刚体变换1001。"""
    if isinstance(T,np.ndarray):T=T.tolist()
    T=np.array([numbers(row,4) for row in T])
    if T.shape!=(4,4) or not np.allclose(T[3],[0,0,0,1],atol=1e-10,rtol=0) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-9,rtol=0) or abs(np.linalg.det(T[:3,:3])-1)>1e-9:
        raise ContractError(1001,'Invalid rigid transform')
    return T


def quaternion(R):
    """合法旋转矩阵转单位四元数[w,x,y,z]；按最大对角项避免pi附近除零。"""
    trace=np.trace(R)
    if trace>0:
        s=2*math.sqrt(1+trace);q=np.array([s/4,(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s])
    else:
        i=int(np.argmax(np.diag(R)));j=(i+1)%3;k=(i+2)%3
        s=2*math.sqrt(max(0,1+R[i,i]-R[j,j]-R[k,k]));q=np.zeros(4)
        q[0]=(R[k,j]-R[j,k])/s;q[i+1]=s/4;q[j+1]=(R[j,i]+R[i,j])/s;q[k+1]=(R[k,i]+R[i,k])/s
    q/=np.linalg.norm(q)
    # pi时符号唯一，确定性最短路径；四元数正负等价。
    nz=next((x for x in q if abs(x)>1e-12),1)
    return q if nz>0 else -q


def rotation(q):
    """单位四元数[w,x,y,z]转旋转矩阵；内部SLERP输出保证单位长度。"""
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

class CPath(C.Structure):
    """独立C规划库的几何参数结构，不属于冻结运动学ABI。"""
    _fields_=[('method',C.c_int)]+[(k,C.c_double*3) for k in ('origin','u','v')]+[(k,C.c_double) for k in ('radius','sweep')]

class PosePath:
    def __init__(self,library,start,goal,method='line',via=None,orientation='slerp_shortest'):
        """入参base→tool0起终位姿；arc另需途经点m(3,)。姿态fixed或shortest SLERP。

        圆弧采用三点唯一圆和平面，选择包含途经点的带符号弧；共线/重合拒绝1001。
        full-circle未支持；fixed要求起终旋转相同，否则1007。
        """
        self.start=rigid(start);self.goal=rigid(goal);self.method=method;self.orientation=orientation
        self.q0=quaternion(self.start[:3,:3]);self.q1=quaternion(self.goal[:3,:3])
        if np.dot(self.q0,self.q1)<0:self.q1=-self.q1
        self.dot=float(np.clip(np.dot(self.q0,self.q1),-1,1));self.angle=2*math.acos(self.dot)
        if orientation not in ('fixed','slerp_shortest'):raise ContractError(1008,'Unsupported orientation rule')
        if orientation=='fixed' and self.angle>1e-8:raise ContractError(1007,'Fixed orientation endpoints differ')
        p0,p1=self.start[:3,3],self.goal[:3,3]
        self.geometry=CPath();self.geometry.method={'line':1,'arc':2}.get(method,0)
        if method=='line':
            self.geometry.origin[:]=p0;self.geometry.u[:]=p1-p0;self.length=float(np.linalg.norm(p1-p0))
        elif method=='arc':
            b=np.array(numbers(via,3))-p0;c=p1-p0;n=np.cross(b,c);den=float(np.dot(n,n))
            if den<1e-16 or min(np.linalg.norm(b),np.linalg.norm(c),np.linalg.norm(b-c))<1e-6:raise ContractError(1001,'Degenerate three-point arc')
            center=p0+(np.dot(b,b)*np.cross(c,n)+np.dot(c,c)*np.cross(n,b))/(2*den)
            radius=float(np.linalg.norm(p0-center));u=(p0-center)/radius;v=np.cross(n/np.linalg.norm(n),u)
            angle=lambda p:math.atan2(np.dot(p-center,v),np.dot(p-center,u))%(2*math.pi)
            end=angle(p1);mid=angle(p0+b);sweep=end if mid<end else end-2*math.pi
            if radius>10 or abs(sweep)<1e-8:raise ContractError(1001,'Invalid arc radius/sweep')
            self.geometry.origin[:]=center;self.geometry.u[:]=u;self.geometry.v[:]=v
            self.geometry.radius=radius;self.geometry.sweep=sweep;self.length=radius*abs(sweep)
            self.via_progress=mid/sweep if sweep>0 else (mid-2*math.pi)/sweep
        else:raise ContractError(1008,'Unknown path method')
        self.lib=C.CDLL(str(Path(library).resolve()));self.fn=self.lib.robot_path_position
        self.fn.argtypes=[C.POINTER(CPath),C.c_double]+[C.POINTER(C.c_double)]*3+[C.c_size_t];self.fn.restype=C.c_int

    def at(self,s):
        """s为[0,1]几何进度，返回4x4位姿；位置由C求值，姿态按同一s最短SLERP。"""
        values=[(C.c_double*3)() for _ in range(3)];code=self.fn(C.byref(self.geometry),s,*values,3)
        if code:raise ContractError(code,'C path interpolation rejected')
        if self.orientation=='fixed':q=self.q0
        elif self.dot>1-1e-10:q=(1-s)*self.q0+s*self.q1;q=q/np.linalg.norm(q)
        else:
            alpha=math.acos(self.dot);q=(math.sin((1-s)*alpha)*self.q0+math.sin(s*alpha)*self.q1)/math.sin(alpha)
        T=np.eye(4);T[:3,:3]=rotation(q);T[:3,3]=list(values[0]);return T

    def metadata(self):
        """返回路径几何/姿态规范，以base米和弧度记录。"""
        return dict(method=self.method,orientation=self.orientation,orientation_angle_rad=self.angle,length_m=self.length,
            origin_m=list(self.geometry.origin),plane_u=list(self.geometry.u),plane_v=list(self.geometry.v),
            radius_m=self.geometry.radius,sweep_rad=self.geometry.sweep,via_progress=getattr(self,'via_progress',None))


def progress(t,T):
    """静止端点五次时间标度；t秒，T总秒，返回无量纲进度。"""
    u=min(1,max(0,t/T));return u**3*(10-15*u+6*u*u)


def coefficients(q,dt):
    """节点q(N,6)→归一化局部五次系数(M,6,6)，内节点中心差分导数，端导数零。"""
    q=np.asarray(q,float);v=np.gradient(q,dt,axis=0,edge_order=2);a=np.gradient(v,dt,axis=0,edge_order=2)
    v[[0,-1]]=0;a[[0,-1]]=0;segments=[]
    for i in range(len(q)-1):
        c0=q[i];c1=v[i]*dt;c2=.5*a[i]*dt*dt
        d=q[i+1]-c0-c1-c2;dv=v[i+1]*dt-c1-2*c2;da=a[i+1]*dt*dt-2*c2
        segments.append(np.array([c0,c1,c2,10*d-4*dv+.5*da,-15*d+7*dv-da,6*d-3*dv+.5*da]).T)
    return np.array(segments)


def evaluate(co,dt,t):
    """归一化五次轨迹任意时刻求q/qd/qdd，不外推，输出三组6维NumPy数组。"""
    T=len(co)*dt
    if t<0 or t>T+1e-12:raise ContractError(1006,'Trajectory time out of range')
    i=min(int(t/dt),len(co)-1);u=(min(t,T)-i*dt)/dt
    return tuple(np.array([np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(c,m=o))/dt**o for c in co[i]]) for o in range(3))


def joint_extrema(co,dt,profile,limits):
    """对每段多项式全部实根检查位置/速度/加速度极值；越限1005/3002。"""
    peaks=np.zeros((2,6));minimum=np.full(6,np.inf);maximum=np.full(6,-np.inf)
    for index,segment in enumerate(co):
        duration = float(dt if np.isscalar(dt) else dt[index])
        for i,c in enumerate(segment):
            for order in range(3):
                derivative=np.polynomial.polynomial.polyder(c,m=order)
                roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(derivative))
                points=[0.,1.]+[float(x.real) for x in roots if abs(x.imag)<1e-7 and 0<x.real<1]
                values=np.array([np.polynomial.polynomial.polyval(x,derivative)/duration**order for x in points])
                if order==0:minimum[i]=min(minimum[i],min(values));maximum[i]=max(maximum[i],max(values))
                else:peaks[order-1,i]=max(peaks[order-1,i],max(abs(values)))
    if np.any(minimum<np.array(profile['q_min_rad'])-1e-9) or np.any(maximum>np.array(profile['q_max_rad'])+1e-9):raise ContractError(3002,'Continuous polynomial position exceeded')
    if np.any(peaks[0]>np.array(limits.joint_velocity_rad_s)+1e-8) or np.any(peaks[1]>np.array(limits.joint_acceleration_rad_s2)+1e-7):raise ContractError(3002,'Continuous polynomial joint rates exceeded')
    return dict(position_min_rad=minimum.tolist(),position_max_rad=maximum.tolist(),peak_velocity_rad_s=peaks[0].tolist(),peak_acceleration_rad_s2=peaks[1].tolist(),proof='all real polynomial extrema')

class CartesianPlanner:
    def __init__(self,kinematics_library,planning_library,config_path=ROOT/'config/cartesian-planning.v1.json',limits_path=DEFAULT_CONFIG,model_path=DEFAULT_MODEL):
        """入参两独立C库和算法/限值配置路径；路径几何C11，编排与连续选解Python。"""
        self.kinematics_library=Path(kinematics_library);self.planning_library=Path(planning_library)
        self.config_path=config_path;self.limits_path=limits_path
        self.model_path=Path(model_path)
        self.profile=json.loads(self.model_path.read_text())

    def setup(self):
        """重新读取共享限值；选解临时profile采用有效速度，模型磁盘内容不修改。"""
        self.config=configuration(self.config_path);self.limits=load_planning_limits(self.limits_path,self.model_path)
        self.profile=json.loads(self.model_path.read_text())
        self.selection_profile=copy.deepcopy(self.profile);self.selection_profile['qd_max_rad_s']=list(self.limits.joint_velocity_rad_s)
        c=self.config
        self.selector=StatefulIKSelector(self.kinematics_library,SelectionConfig(dt_s=c['sample_period_s'],
            position_tol_m=c['ik_position_tolerance_m'],orientation_tol_rad=c['ik_orientation_tolerance_rad'],timeout_s=c['ik_timeout_s']))
        self.model=pack_model(self.profile)

    def certify(self,q,scene,target_at=None):
        """拟合C2节点段，验连续关节极值、细分全机器人碰撞及FK/TCP数值导数。

        q为固定周期节点，scene为StaticScene；target_at(t)可选目标位姿。
        TCP用标准DH半径及解析关节极值的保守导数界；路径误差按细分验收；碰撞用网格运动界覆盖间隔。
        """
        dt=self.config['sample_period_s']
        co=coefficients(q,dt)
        data=self.certify_polynomial(co, np.arange(len(co)+1)*dt, scene, target_at)
        data['continuity']='C2'
        return data

    def certify_polynomial(self, co, times, scene, target_at=None):
        """Run complete certification, retaining partial validation evidence on error."""
        self.certification_log=[]
        try:
            return self._certify_polynomial(co,times,scene,target_at)
        except (ContractError,OSError,KeyError,ValueError,TypeError,ArithmeticError,ImportError) as exc:
            self.certification_log.append(dict(stage='rejected',code=getattr(exc,'code',1001),message=str(exc)))
            exc.certification_log=copy.deepcopy(self.certification_log)
            raise

    def _certify_polynomial(self, co, times, scene, target_at=None):
        """Certify the supplied normalized polynomials without refitting them.

        Inputs: (M,6,6) coefficients, M+1 segment times in seconds and a
        StaticScene. Returns full evidence or raises; never returns partial data.
        """
        from planning.execution import seal
        c=self.config
        co=np.asarray(co,float);times=np.asarray(times,float)
        if co.ndim!=3 or co.shape[1:]!=(6,6) or len(co)<1 or times.shape!=(len(co)+1,) or not np.all(np.isfinite(co)) or not np.all(np.isfinite(times)) or times[0]!=0 or np.any(np.diff(times)<=0):
            raise ContractError(1001,'Invalid polynomial trajectory')
        if scene is None:
            raise ContractError(1004,'Whole-arm/self collision scene required')
        from planning.collision import StaticScene
        if not isinstance(scene,StaticScene):
            raise ContractError(1004,'Frozen StaticScene checker required')
        if scene.profile_sha256!=hashlib.sha256(json.dumps(self.profile,sort_keys=True).encode()).hexdigest():
            raise ContractError(1004,'Collision/kinematics model binding differs')
        durations=np.diff(times)
        if times[-1]>c['maximum_duration_s'] or len(co)>c['maximum_nodes']:
            raise ContractError(3001,'Certification time/node budget exhausted')
        # C1 joins are mandatory; acceleration may jump for cubic/trapezoidal.
        for i in range(1,len(co)):
            for order,tolerance in ((0,1e-9),(1,1e-8)):
                left=np.array([np.polynomial.polynomial.polyval(1,np.polynomial.polynomial.polyder(axis,m=order))/durations[i-1]**order for axis in co[i-1]])
                right=np.array([np.polynomial.polynomial.polyval(0,np.polynomial.polynomial.polyder(axis,m=order))/durations[i]**order for axis in co[i]])
                if np.max(abs(left-right))>tolerance:
                    raise ContractError(3001,'Discontinuous polynomial position/velocity')
        def at(t):
            i=min(int(np.searchsorted(times,t,side='right'))-1,len(co)-1)
            u=(min(t,times[-1])-times[i])/durations[i]
            return tuple(np.array([np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(axis,m=o))/durations[i]**o for axis in co[i]]) for o in range(3))
        extrema=joint_extrema(co,durations,self.profile,self.limits)
        self.certification_log.append(dict(stage='joint_extrema',passed=True,evidence=extrema))
        # 标准DH全转动关节：TCP到第i轴距离不超过剩余连杆平移长度之和。
        radii=np.cumsum(np.hypot(self.profile['a_m'],self.profile['d_m'])[::-1])[::-1]
        for key in ('T_dh6_flange','T_flange_tool'):radii+=np.linalg.norm(np.array(self.profile[key])[:3,3])
        v=np.array(extrema['peak_velocity_rad_s'])*(1+1e-6)+1e-8
        a=np.array(extrema['peak_acceleration_rad_s2'])*(1+1e-6)+1e-7
        angular_v=float(sum(v));linear_v=float(radii@v)
        # Jdot的保守界：线性列<=2*R_i*sum|qd|；角列<=sum|qd|。
        linear_a=float(radii@a+2*linear_v*angular_v);angular_a=float(sum(a)+angular_v**2)
        tcp_bounds=[linear_v,angular_v,linear_a,angular_a]
        self.certification_log.append(dict(stage='tcp_bounds',bounds=tcp_bounds))
        self.limits.check_tcp_rates(*([x,0.,0.] for x in tcp_bounds))
        T=float(times[-1]);nodes=[];minimum=math.inf;maxp=maxr=0.;tcppeaks=np.zeros(4);previous=None;max_motion_bound=0.;min_clearance_certificate=math.inf
        clearance_gap=None
        for i,segment in enumerate(co):
            dt=float(durations[i])
            # 基于解析速度峰值上界保证各子步关节变化有界，同时至少固定细分数。
            vmax=0.;axis_peaks=[]
            for axis in segment:
                d=np.polynomial.polynomial.polyder(axis);roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(d))
                us=[0,1]+[x.real for x in roots if abs(x.imag)<1e-7 and 0<x.real<1]
                peak=max(abs(np.polynomial.polynomial.polyval(u,d))/dt for u in us);axis_peaks.append(peak);vmax=max(vmax,peak)
            count=max(c['collision_subdivisions'],math.ceil(dt/c['sample_period_s']),math.ceil(vmax*dt/c['collision_max_joint_step_rad']))
            if count>100000:raise ContractError(3001,'Certification subdivision budget exhausted')
            for k in range(count+(i==len(co)-1)):
                t=float(times[i]+k/count*dt);angles,vel,acc=at(t);actual=self.selector._fk(self.model,angles)
                collision=scene.check(angles.tolist());minimum=min(minimum,collision['minimum_distance_lower_bound_m'])
                self.certification_log.append(dict(stage='collision',segment_index=i,time_s=t,q_rad=angles.tolist(),result=collision))
                if not collision['passed']:raise ContractError(3003,'Collision/clearance failed at t='+str(t))
                speed_sum=sum(axis_peaks)*(1+1e-6)+1e-8
                if previous is not None:
                    previous_t,previous_distance,previous_speed=previous
                    # 两端覆盖中间任意t最近端的半间隔；自碰撞距离最多2倍点位移。
                    bound=scene.motion_radius_bound_m*max(previous_speed,speed_sum)*(t-previous_t)
                    certified=min(previous_distance,collision['minimum_distance_lower_bound_m'])-bound
                    max_motion_bound=max(max_motion_bound,bound);min_clearance_certificate=min(min_clearance_certificate,certified)
                    # Finish collision queries first so an actual colliding node
                    # is reported as 3003 rather than an earlier proof gap.
                    if certified<=scene.clearance and clearance_gap is None:
                        clearance_gap=dict(start_time_s=previous_t,end_time_s=t,clearance_lower_bound_m=certified)
                previous=(t,collision['minimum_distance_lower_bound_m'],speed_sum)
                pe=re=None
                if target_at is not None:
                    pe,re=pose_error(actual,target_at(t));maxp=max(maxp,pe);maxr=max(maxr,re)
                    if pe>c['position_tolerance_m'] or re>c['orientation_tolerance_rad']:raise ContractError(3001,'Cartesian interpolation tracking error')
                # 由C FK的数值雅可比和沿轨迹J变化估计TCP空间导数，不采用Euler角差分。
                J=self.selector.jacobian(self.model,angles);twist=J@vel
                h=min(c['derivative_step_s'],dt/10);ql=at(max(0,t-h))[0];qr=at(min(T,t+h))[0]
                interval=min(T,t+h)-max(0,t-h)
                Jdot=(self.selector.jacobian(self.model,qr)-self.selector.jacobian(self.model,ql))/interval
                derivative=J@acc+Jdot@vel
                self.limits.check_tcp_rates(twist[:3].tolist(),twist[3:].tolist(),derivative[:3].tolist(),derivative[3:].tolist())
                tcppeaks=np.maximum(tcppeaks,[np.linalg.norm(twist[:3]),np.linalg.norm(twist[3:]),np.linalg.norm(derivative[:3]),np.linalg.norm(derivative[3:])])
                nodes.append(dict(time_s=t,q_rad=angles.tolist(),position_error_m=pe,orientation_error_rad=re,
                    minimum_distance_lower_bound_m=collision['minimum_distance_lower_bound_m']))
        if clearance_gap is not None:
            self.certification_log.append(dict(stage='continuous_clearance',passed=False,**clearance_gap))
            raise ContractError(3001,'Insufficient clearance for continuous motion bound')
        sample_times=sorted(set([float(t) for t in times]+[k*c['sample_period_s'] for k in range(math.ceil(T/c['sample_period_s'])) if k*c['sample_period_s']<T]))
        qv=[at(t) for t in sample_times]
        data=dict(time_s=sample_times,q_rad=[x[0].tolist() for x in qv],
            qd_rad_s=[x[1].tolist() for x in qv],qdd_rad_s2=[x[2].tolist() for x in qv],coefficients_rad=co.tolist(),
            segment_times_s=times.tolist(),continuity='C1',joint_check=extrema,
            collision_checked=True,cartesian_limits_checked=True,continuous_tcp_proof=True,tcp_continuous_bounds=tcp_bounds,tcp_joint_radius_bounds_m=radii.tolist(),
            continuous_collision_proof=True,
            collision_motion_bound_m=max_motion_bound,certified_minimum_distance_lower_bound_m=min_clearance_certificate,
            validation_nodes=nodes,minimum_distance_lower_bound_m=minimum,maximum_position_error_m=maxp if target_at is not None else None,
            maximum_orientation_error_rad=maxr if target_at is not None else None,tcp_sampled_peaks=tcppeaks.tolist(),scene=scene.metadata(),effective_limits=self.limits.as_dict())
        return seal(data, self, scene)

    def plan(self,start,goal,q_seed,scene,method='line',via=None,orientation='slerp_shortest',duration_s=4.0):
        """固定周期生成直线/圆弧节点，逐节点连续选解后整体验收。

        入参合法base→tool0位姿、种子rad(6,)、StaticScene、几何/姿态规则和请求时长秒。
        时长向上取周期整数倍；失败data/next_state=None，details保留逐节点记录及最后状态，
        不输出部分可执行轨迹。不得在失败后继续旧段。
        """
        records=[];state=None
        try:
            self.setup();c=self.config;dt=c['sample_period_s']
            path=PosePath(self.planning_library,start,goal,method,via,orientation)
            if type(duration_s) not in (int,float) or not math.isfinite(duration_s) or duration_s<2*dt:raise ContractError(1006,'Invalid duration')
            n=math.ceil(duration_s/dt);T=n*dt
            if T>c['maximum_duration_s'] or n+1>c['maximum_nodes']:raise ContractError(3001,'Time/node budget exhausted')
            state=dict(q_rad=list(numbers(q_seed,6)),step_index=0)
            initial=self.selector._fk(self.model,state['q_rad']);pe,re=pose_error(initial,path.start)
            if pe>c['ik_position_tolerance_m'] or re>c['ik_orientation_tolerance_rad']:raise ContractError(1001,'Seed does not match start')
            if not scene.check(state['q_rad'])['passed']:raise ContractError(3003,'Initial configuration collides')
            q=[state['q_rad'].copy()]
            for index in range(1,n+1):
                t=index*dt;target=path.at(progress(t,T));before=copy.deepcopy(state)
                result=self.selector.select(self.selection_profile,target.tolist(),state)
                record=dict(node_index=index,time_s=t,target_T_base_tool=target.tolist(),code=result['code'],
                    state_advanced=result['code']==0,input_state_unchanged=state==before,
                    position_error_m=None,orientation_error_rad=None,sigma_min=None)
                records.append(record)
                if result['code']:
                    # 保存实际调用前后状态与求解器输出，失败记录不能假定状态未变。
                    record.update(state_before=before,state_after=copy.deepcopy(state),
                        solver_next_state_is_null=result.get('next_state') is None,
                        solver_data_is_null=result.get('data') is None,
                        state_advanced=state!=before)
                    raise ContractError(result['code'],'IK failed at node '+str(index))
                state=copy.deepcopy(result['next_state']);actual=self.selector._fk(self.model,state['q_rad']);pe,re=pose_error(actual,target)
                record.update(q_rad=state['q_rad'].copy(),position_error_m=pe,orientation_error_rad=re,
                    sigma_min=self.selector.sigma_min(self.model,state['q_rad']),mode=result['data']['mode'])
                q.append(state['q_rad'].copy())
            data=self.certify(q,scene,lambda t:path.at(progress(t,T)))
            data.update(path=path.metadata(),sample_period_s=dt,actual_duration_s=T,ik_nodes=records)
            return dict(code=0,message='OK',data=data,next_state=state,details=dict(requested_duration_s=duration_s))
        except (ContractError,OSError,ImportError,ValueError,TypeError,KeyError,ArithmeticError,np.linalg.LinAlgError) as exc:
            return dict(code=getattr(exc,'code',1004 if isinstance(exc,(KeyError,OSError,ImportError)) else 1001),message=str(exc),data=None,next_state=None,
                details=dict(ik_nodes=records,last_ik_candidate_state=state,retained_input_q_rad=list(q_seed),stop_segment=True,replan_required=True,
                    failure_log=getattr(exc,'certification_log',[dict(message=str(exc))])))
