"""同场景基线/引导平滑缩时对比：固定输入、逐次耗时、路径与失败原因，不跳过失败。"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import platform
import sys
import time
import numpy as np
import pybullet as bullet
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'scripts'))
from planning.cartesian import CartesianPlanner,configuration,evaluate,coefficients
from planning.collision import StaticScene
from planning.apf import static_apf,time_parameterize
from planning.guided_apf import improved_plan,guided_apf,smooth_waypoints,retime_trajectory,parameterize_smoothed,edge_check,settings
from verify_joint_planning import save_curves


def statistics(values):
    """有限逐次秒值汇总；p95/中位数采用h=(n-1)p线性分位，不删除失败耗时。"""
    return dict(count=len(values),mean_s=float(np.mean(values)),median_s=float(np.median(values)),
        p95_s=float(np.quantile(values,.95,method='linear')),maximum_s=max(values))


def baseline(planner,start,goal,scene):
    """运行原势场及原静止段定时，返回全部阶段；失败不给部分可执行轨迹。"""
    geometric=static_apf(planner,start,goal,scene)
    if geometric['code']:
        return dict(code=geometric['code'],message=geometric['message'],data=None,next_state=None,details=dict(geometric=geometric))
    timed=time_parameterize(planner,geometric['data']['waypoints_rad'],scene)
    return dict(timed,next_state=None,details=dict(geometric=geometric,timing=timed.get('details',{})))


def measures(planner,result):
    """计算可执行轨迹时长、关节折线长及固定周期TCP路径长；失败长度/耗时不填零。"""
    if result['code']!=0:return dict(duration_s=None,joint_path_length_rad=None,tcp_path_length_m=None)
    data=result['data'];q=np.array(data['q_rad']);tcp=np.array([planner.selector._fk(planner.model,x)[:3,3] for x in q])
    return dict(duration_s=data['time_s'][-1],joint_path_length_rad=float(np.sum(np.linalg.norm(np.diff(q,axis=0),axis=1))),
        tcp_path_length_m=float(np.sum(np.linalg.norm(np.diff(tcp,axis=0),axis=1))),
        joint_check=data['joint_check'],tcp_continuous_bounds=data['tcp_continuous_bounds'],
        certified_minimum_distance_m=data['certified_minimum_distance_lower_bound_m'],
        node_count=len(q),validation_node_count=len(data['validation_nodes']))


def requested_retime_peaks(data,factor,dt):
    """独立复建缩时多项式并取导数实根极值，仅作失败诊断，不交付轨迹。"""
    old=np.array(data['coefficients_rad']);T=len(old)*data['sample_period_s'];n=max(2,int(np.ceil(T*factor/dt)))
    q=[evaluate(old,data['sample_period_s'],k*T/n)[0] for k in range(n+1)]
    co=coefficients(q,dt);peaks=np.zeros((2,6))
    for segment in co:
        for axis,c in enumerate(segment):
            for order in (1,2):
                d=np.polynomial.polynomial.polyder(c,m=order)
                roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(d))
                us=[0,1]+[float(r.real) for r in roots if abs(r.imag)<1e-7 and 0<r.real<1]
                peaks[order-1,axis]=max(peaks[order-1,axis],max(abs(np.polynomial.polynomial.polyval(u,d))/dt**order for u in us))
    return dict(actual_duration_s=n*dt,peak_velocity_rad_s=peaks[0].tolist(),peak_acceleration_rad_s2=peaks[1].tolist())


def main():
    """CLI指定两库/输出，预期错误也须验证；任何非预期结果或证据失败返回1。"""
    process_start=time.perf_counter()
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so')
    ap.add_argument('--planning-library',type=Path,default=ROOT/'build/librobot_planning.so')
    ap.add_argument('--output',type=Path,default=ROOT/'results/guided-apf')
    args=ap.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    checks=[];comparisons=[];artifacts=[];plot_data=[]
    def save(name,data):
        p=name+'.json';(out/p).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n');artifacts.append(p);return p
    def check(name,passed,details):checks.append(dict(id=name,passed=bool(passed),details=details))
    p=CartesianPlanner(args.library,args.planning_library);p.setup()
    scene_config=json.loads((ROOT/'scenes/static-apf.v1.json').read_text())
    start=[.3,-1.2,1.1,-1.1,1.,.2];goal=[.36,-1.17,1.06,-1.08,1.01,.2]
    self_goal=[-.3742565963470028,-2.3843366984624392,-.8970081111253569,1.1288944496614386,-.2917323291194913,-1.869156334633904]
    budget=configuration();budget['apf']['maximum_iterations']=4
    save('short-stage-algorithm-config',budget)
    short=CartesianPlanner(args.library,args.planning_library,config_path=out/'short-stage-algorithm-config.json')
    s=StaticScene(scene_config,p.profile)
    try:
        s.check(start);bounds=bullet.getAABB(s.robot,2,physicsClientId=s.client)
        center=((np.array(bounds[0])+np.array(bounds[1]))/2).tolist()
    finally:s.close()
    arm_config=copy.deepcopy(scene_config);arm_config['scene_id']='guided-upperarm-blocked'
    arm_config['obstacles'].append(dict(id='upperarm-blocker',kind='sphere',position_m=center,radius_m=.04))
    save('upperarm-blocked-scene',arm_config)
    scenarios=[('nominal',p,scene_config,goal,{'baseline':0,'improved':0}),
        ('short-stage-budget',short,scene_config,goal,{'baseline':3001,'improved':0}),
        ('self-blocked',p,scene_config,self_goal,{'baseline':3003,'improved':3003}),
        ('upperarm-blocked',p,arm_config,goal,{'baseline':3003,'improved':3003})]
    primary={}
    for name,planner,scene_definition,end,expected in scenarios:
        scene=StaticScene(scene_definition,planner.profile)
        try:
            trials={k:[] for k in expected};times={k:[] for k in expected}
            # 预热两条流水线不纳入统计；正式3次交替顺序，全部失败/异常也计数。
            for function in (baseline,improved_plan):function(planner,start,end,scene)
            for repetition in range(3):
                for method in (('baseline','improved') if repetition%2==0 else ('improved','baseline')):
                    fn=baseline if method=='baseline' else improved_plan
                    before=copy.deepcopy(start);t=time.perf_counter()
                    try:result=fn(planner,start,end,scene)
                    except Exception as exc:result=dict(code=9000,message=type(exc).__name__+': '+str(exc),data=None,next_state=None,details={})
                    elapsed=time.perf_counter()-t;times[method].append(elapsed)
                    measurement=measures(planner,result)
                    trial=dict(repetition=repetition,code=result['code'],message=result['message'],pipeline_wall_s=elapsed,
                        input_state_unchanged=start==before,**measurement)
                    if result['code']!=0:trial['failure_result']=result
                    else:
                        data=result['data'];trial['joint_path_sha256']=hashlib.sha256(json.dumps(data['q_rad']).encode()).hexdigest()
                    trials[method].append(trial)
                    check(name+'-'+method+'-'+str(repetition),result['code']==expected[method] and start==before and
                        (result['data'] is not None if expected[method]==0 else result['data'] is None and result.get('next_state') is None),trial)
                    if repetition==0:
                        artifact=save(name+'-'+method,result);primary[(name,method)]=result
                        if result['code']==0:
                            display=dict(result['data']);display['segment_kind']=['quintic']*len(display['coefficients_rad']);display['segment_times_s']=[0,display['time_s'][-1]]
                            artifacts.extend(save_curves(out,name+'-'+method,display))
                            curve=out/(name+'-'+method+'.svg');curve.write_text(curve.read_text().replace('No TCP/collision acceptance.','Full limits and whole-arm/self collision recertified; see report.'))
                            plot_data.append((name+'-'+method,result['data'],planner))
                    print(name,method,repetition,result['code'],round(elapsed,4),result['message'],flush=True)
            rows={}
            for method in trials:
                successes=[x for x in trials[method] if x['code']==0]
                rows[method]=dict(trials=trials[method],success_count=len(successes),executed_count=3,
                    success_rate=len(successes)/3,expected_result_rate=sum(x['code']==expected[method] for x in trials[method])/3,
                    failure_count=3-len(successes),timeout_count=0,pipeline_timing=statistics(times[method]),
                    trajectory_duration_statistics=statistics([x['duration_s'] for x in successes]) if successes else None)
                check(name+'-'+method+'-repeatability',not successes or len({x['joint_path_sha256'] for x in successes})==1,dict(successful_path_hashes=[x['joint_path_sha256'] for x in successes]))
            comparisons.append(dict(case_id=name,start_q_rad=start,goal_q_rad=end,scene=scene_definition,
                scene_sha256=scene.metadata()['scene_sha256'],apf_configuration=planner.config['apf'],methods=rows))
            if name=='self-blocked':
                collision=scene.check(end);check('explicit-non-parent-self-collision',not collision['passed'] and 'self_link_pair' in collision['nearest'],collision)
            if name=='upperarm-blocked':
                tcp=(np.array(planner.profile['T_world_base'])@np.r_[planner.selector._fk(planner.model,start)[:3,3],1])[:3]
                collision=scene.check(start);check('upperarm-blocker-away-from-tool',np.linalg.norm(tcp-center)>.1 and not collision['passed'] and collision['nearest'].get('obstacle')=='upperarm-blocker',dict(tcp_to_blocker_m=float(np.linalg.norm(tcp-center)),collision=collision))
        finally:scene.close()
    scene=StaticScene(scene_config,p.profile)
    try:
        improved=primary[('nominal','improved')];before=improved['details']['stages']['before_compression']['data']
        check('safe-compression-shortens-duration',improved['data']['time_s'][-1]<before['time_s'][-1] and improved['data']['continuous_tcp_proof'] and improved['data']['continuous_collision_proof'],improved['details']['stages']['compression']['details'])
        check('actual-subgoal-selected',primary[('short-stage-budget','improved')]['details']['stages']['guided']['details']['used_subgoal'],dict(selected_candidate=primary[('short-stage-budget','improved')]['details']['stages']['guided']['details'].get('selected_candidate')))
        for name,factor in [('excessive-speed-compression',.02)]:
            snapshot=copy.deepcopy(before);result=retime_trajectory(p,before,scene,factor)
            diagnostic=requested_retime_peaks(before,factor,p.config['sample_period_s']);result['details']['rejected_derivative_peaks']=diagnostic;save(name,result)
            check(name,result['code']==3002 and result['data'] is None and result['next_state'] is None and before==snapshot and
                np.any(np.array(diagnostic['peak_velocity_rad_s'])>p.limits.joint_velocity_rad_s),result)
        tiny=np.array(start);tiny[0]+=.001
        original=parameterize_smoothed(p,[start,tiny.tolist()],scene,.8)
        result=retime_trajectory(p,original['data'],scene,.1) if original['code']==0 else original
        diagnostic=requested_retime_peaks(original['data'],.1,p.config['sample_period_s']);result['details']['rejected_derivative_peaks']=diagnostic
        save('excessive-acceleration-compression',result)
        check('excessive-acceleration-compression',result['code']==3002 and result['data'] is None and
            np.all(np.array(diagnostic['peak_velocity_rad_s'])<=p.limits.joint_velocity_rad_s) and
            np.any(np.array(diagnostic['peak_acceleration_rad_s2'])>p.limits.joint_acceleration_rad_s2),result)
        for label,factor in [('zero',0),('nan',float('nan')),('expansion',1.1)]:
            result=retime_trajectory(p,before,scene,factor);result['details']['requested_factor']=label
            save('invalid-factor-'+label,result);check('invalid-factor-'+label,result['code']==1001 and result['data'] is None,result)
        proof=edge_check(p,scene,start,goal,maximum_subdivisions=2)
        check('edge-proof-budget-rejects',not proof['passed'] and proof['code']==3001,proof)
        blocked=smooth_waypoints(p,[start,self_goal],scene);save('unsafe-smoothing-rejected',blocked)
        check('unsafe-smoothing-rejected',blocked['code']==3003 and blocked['data'] is None,blocked)
        a=np.array(self_goal);b=a.copy();a[3]-=2.;b[3]+=2.
        endpoints_safe=scene.check(a.tolist())['passed'] and scene.check(b.tolist())['passed']
        rejected=smooth_waypoints(p,[a.tolist(),b.tolist()],scene);save('safe-endpoints-colliding-interior',rejected)
        check('safe-endpoints-colliding-interior',endpoints_safe and rejected['code']==3003 and rejected['data'] is None,
            dict(endpoints_safe=endpoints_safe,start_q_rad=a.tolist(),goal_q_rad=b.tolist(),failure=rejected))
        missing=out/'incomplete-guide-config.json';missing.write_text('{}')
        result=improved_plan(p,start,goal,scene,missing);save('missing-guide-config-rejected',result);missing.unlink()
        check('missing-guide-config-rejected',result['code']==1004 and result['data'] is None,result)
        # 真实默认160迭代预算用完，保存完整失败迭代及子目标两阶段恢复。
        long_goal=[-.8754747782692309,-2.6727021827979156,2.1755784066603465,-.22273107895773991,-.796302706150867,2.3246457254720685]
        t=time.perf_counter();direct=static_apf(p,start,long_goal,scene);direct_time=time.perf_counter()-t
        t=time.perf_counter();guided=guided_apf(p,start,long_goal,scene);guide_time=time.perf_counter()-t
        save('long-route-geometric-baseline',dict(result=direct,pipeline_wall_s=direct_time))
        save('long-route-geometric-guided',dict(result=guided,pipeline_wall_s=guide_time))
        check('default-budget-subgoal-geometric-recovery',direct['code']==3001 and guided['code']==0 and guided['details']['used_subgoal'],dict(baseline_code=direct['code'],guided_code=guided['code'],baseline_wall_s=direct_time,guided_wall_s=guide_time,executable=False))
        # 长路线仍要服从总时长预算，几何找到路线并不意味着可以执行。
        full=improved_plan(p,start,long_goal,scene);save('long-route-duration-budget-failure',full)
        check('long-route-no-partial-executable-output',full['code']==3001 and full['data'] is None and full['next_state'] is None,dict(code=full['code'],message=full['message'],retained_input_q_rad=full['details'].get('retained_input_q_rad')))
        for name,data,planner in plot_data:
            dt=data['sample_period_s'];co=np.array(data['coefficients_rad']);T=data['time_s'][-1]
            endpoints=[evaluate(co,dt,t) for t in (0,T)]
            jumps=[]
            for i in range(1,len(co)):
                for order in (0,1,2):
                    left=np.array([np.polynomial.polynomial.polyval(1,np.polynomial.polynomial.polyder(c,m=order))/dt**order for c in co[i-1]])
                    right=np.array([np.polynomial.polynomial.polyval(0,np.polynomial.polynomial.polyder(c,m=order))/dt**order for c in co[i]])
                    jumps.append(float(np.max(abs(left-right))))
            check(name+'-C2-endpoint-and-limits',data['continuity']=='C2' and data['continuous_tcp_proof'] and data['continuous_collision_proof'] and
                data['certified_minimum_distance_lower_bound_m']>scene.clearance and
                np.max(abs(np.diff(data['time_s'])-dt))<1e-12 and max(jumps)<1e-7 and
                all(np.max(abs(x[o]))<1e-7 for x in endpoints for o in (1,2)),dict(joint_check=data['joint_check'],tcp_bounds=data['tcp_continuous_bounds'],max_join_jump=max(jumps)))
    finally:scene.close()
    # 静态SVG/CSV便于对照，路径来自真实FK，不用投影声称碰撞证明。
    colors=['#2563eb','#16a34a','#9333ea'];svg=['<svg xmlns="http://www.w3.org/2000/svg" width="920" height="650"><rect width="920" height="650" fill="white"/>']
    svg.append('<text x="40" y="30" font-family="sans-serif" font-size="20">Baseline vs guided APF: same static scene / FK TCP path</text>')
    points=[np.array([pl.selector._fk(pl.model,q)[:3,3] for q in data['q_rad']]) for _,data,pl in plot_data]
    lower=np.min(np.concatenate(points),axis=0);upper=np.max(np.concatenate(points),axis=0);span=np.maximum(upper-lower,.005)
    for index,((name,data,pl),ps) in enumerate(zip(plot_data,points)):
        color=colors[index%len(colors)];xy=' '.join(f'{70+760*(v[0]-lower[0])/span[0]},{390-310*(v[1]-lower[1])/span[1]}' for v in ps)
        svg.append(f'<polyline points="{xy}" fill="none" stroke="{color}" stroke-width="2"/><text x="50" y="{435+25*index}" fill="{color}" font-family="sans-serif">{name}: T={data["time_s"][-1]:.2f}s</text>')
    svg.append('<text x="40" y="555" font-family="sans-serif">XY zoom only. Whole-arm/self clearance and continuous limits are in raw reports.</text></svg>')
    (out/'comparison.svg').write_text('\n'.join(svg));artifacts.append('comparison.svg')
    sources=['src/planning/guided_apf.py','scripts/verify_guided_apf.py','src/planning/apf.py','src/planning/cartesian.py','src/planning/collision.py','config/guided-apf.v1.json','config/planning-limits.v1.json','config/cartesian-planning.v1.json','scenes/static-apf.v1.json']
    report=dict(status='passed' if all(c['passed'] for c in checks) else 'failed',passed_count=sum(c['passed'] for c in checks),expected_count=len(checks),
        comparisons=comparisons,checks=checks,environment=dict(python=platform.python_version(),numpy=np.__version__,platform=platform.platform()),
        timing=dict(clock='perf_counter monotonic',start='before complete planning pipeline call',stop='after return including full certification',
            excludes='scene construction, measures/FK path statistics, serialization, plotting; 1 warmup per method',
            repeats=3,order='alternate baseline/improved',quantile='h=(n-1)p linear',c_function_ms=None,python_to_c_ms=None,
            function_timing_status='not separately measured; pipeline wall time is not C/FFI time',script_work_wall_s=time.perf_counter()-process_start,script_work_scope='main entry after imports through final report construction; excludes final report write/process startup; actual process wall is recorded by unified runner',timeout_count=0,
            timeout_scope='whole script limited to 120s by unified runner/CTest; no per-candidate preemptive timeout'),
        configuration=settings(),effective_limits=p.limits.as_dict(),
        limitations=['Simulation only; no global completeness or optimality guarantee','Subgoals have more APF calls than direct baseline; per-call settings equal, total work reported',
            'Rest quintic segments retain stops; no minimum-time claim','3 measured repetitions are observations, not formal latency acceptance',
            'Long-route geometric recovery cannot execute when total-duration/node budgets fail'],
        sources_sha256={x:hashlib.sha256((ROOT/x).read_bytes()).hexdigest() for x in sources},
        libraries_sha256={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in (args.library,args.planning_library)},
        artifacts=[dict(path=x,sha256=hashlib.sha256((out/x).read_bytes()).hexdigest()) for x in artifacts])
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(str(report['passed_count'])+'/'+str(report['expected_count'])+' guided APF checks passed')
    for c in checks:
        if not c['passed']:print(c)
    return 0 if report['status']=='passed' else 1

if __name__=='__main__':sys.exit(main())
