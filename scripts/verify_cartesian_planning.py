"""固定静态场景的直线/圆弧/连续IK/势场成功与失败验证，保留逐点数据。"""
import argparse
import copy
import ctypes as C
import pybullet as bullet
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
from importlib.metadata import version
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'scripts'))
from planning.cartesian import CartesianPlanner,PosePath,configuration,rotation,quaternion,progress,evaluate,joint_extrema
from planning.collision import StaticScene
from planning.apf import static_apf,time_parameterize
from adapters.c_kinematics import ContractError
from adapters.ik_selection import pose_error
from verify_joint_planning import save_curves


def main():
    """CLI指定两C库和输出；所有预期返回码/检查通过才返回0，失败结果也写报告。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so')
    parser.add_argument('--planning-library',type=Path,default=ROOT/'build/librobot_planning.so')
    parser.add_argument('--output',type=Path,default=ROOT/'results/cartesian-planning')
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    planner=CartesianPlanner(args.library,args.planning_library);planner.setup()
    scene_config=json.loads((ROOT/'scenes/static-apf.v1.json').read_text());scene=StaticScene(scene_config,planner.profile)
    q0=[.3,-1.2,1.1,-1.1,1.,.2];start=planner.selector._fk(planner.model,q0);goal=start.copy();goal[:3,3]+=[.02,0,.01]
    via=(start[:3,3]+goal[:3,3])/2+np.array([0,.004,0])
    cases=[];artifacts=[];saved=[]
    def check(name,passed,details):cases.append(dict(id=name,passed=bool(passed),details=details))
    def record(name,result,expected,success=False):
        passed=result['code']==expected and (result['data'] is not None if expected==0 else result['data'] is None)
        case=dict(id=name,passed=passed,expected_code=expected,actual_code=result['code'],message=result['message'])
        if success and result['data']:
            data=result['data'];dt=data['sample_period_s'];ts=np.array(data['time_s'])
            fixed=bool(np.max(abs(np.diff(ts)-dt))<1e-12)
            endpoint=float(np.max(abs(np.array(data['qd_rad_s'])[[0,-1]])))
            accel=float(np.max(abs(np.array(data['qdd_rad_s2'])[[0,-1]])))
            co=np.array(data['coefficients_rad']);joins=[0.,0.,0.]
            for index in range(1,len(co)):
                for order in range(3):
                    left=np.array([np.polynomial.polynomial.polyval(1,np.polynomial.polynomial.polyder(c,m=order))/dt**order for c in co[index-1]])
                    right=np.array([np.polynomial.polynomial.polyval(0,np.polynomial.polynomial.polyder(c,m=order))/dt**order for c in co[index]])
                    joins[order]=max(joins[order],float(np.max(abs(left-right))))
            case.update(fixed_period_passed=fixed,node_count=len(ts),validation_node_count=len(data['validation_nodes']),
                endpoint_speed_max_rad_s=endpoint,endpoint_acceleration_max_rad_s2=accel,maximum_join_jumps=joins,
                minimum_distance_lower_bound_m=data['minimum_distance_lower_bound_m'],tcp_sampled_peaks=data['tcp_sampled_peaks'],
                maximum_position_error_m=data['maximum_position_error_m'],maximum_orientation_error_rad=data['maximum_orientation_error_rad'])
            case['passed'] &= fixed and endpoint<=1e-8 and accel<=1e-7 and all(e<=tol for e,tol in zip(joins,(1e-9,1e-8,1e-7)))
            case['passed'] &= data['certified_minimum_distance_lower_bound_m']>scene.clearance and data['continuous_collision_proof']
            case['certified_minimum_distance_lower_bound_m']=data['certified_minimum_distance_lower_bound_m']
            case['collision_motion_bound_m']=data['collision_motion_bound_m']
            case['tcp_continuous_bounds']=data['tcp_continuous_bounds']
            case['passed'] &= data['continuous_tcp_proof']
            if 'ik_nodes' in data:
                case['passed'] &= all(n['code']==0 and n['state_advanced'] and n['input_state_unchanged'] and n['position_error_m']<=planner.config['ik_position_tolerance_m'] and n['orientation_error_rad']<=planner.config['ik_orientation_tolerance_rad'] for n in data['ik_nodes'])
            data['method']=data.get('method',data.get('path',{}).get('method','apf'));data['segment_kind']=['quintic']*len(co)
            display=dict(data);display['segment_times_s']=[data['time_s'][0],data['time_s'][-1]]
            artifacts.extend(save_curves(out,name,display))
            figure=out/(name+'.svg');figure.write_text(figure.read_text().replace('No TCP/collision acceptance.','TCP/collision bounds: see model-specific report.'))
            saved.append((name,data))
        else:
            case['details']=result.get('details',{})
            if expected!=0 and 'next_state' in result:case['passed'] &= result['next_state'] is None
        (out/(name+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n');artifacts.append(name+'.json');cases.append(case)
        print(name,result['code'],result['message'],flush=True)
    try:
        r=planner.plan(start.tolist(),goal.tolist(),q0,scene,duration_s=3.001,orientation='fixed')
        record('line-success',r,0,True)
        arc_goal=goal.copy();a=.04;R=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
        arc_goal[:3,:3]=R@goal[:3,:3]
        r=planner.plan(start.tolist(),arc_goal.tolist(),q0,scene,method='arc',via=via.tolist(),duration_s=3.)
        record('arc-success',r,0,True)
        # 机器人几何与运动学frame对齐，避免碰撞检查与TCP使用不同零位/坐标系。
        scene.check(q0)
        tool_index=next(i for i in range(bullet.getNumJoints(scene.robot,physicsClientId=scene.client)) if bullet.getJointInfo(scene.robot,i,physicsClientId=scene.client)[12].decode()=='tool0')
        link=bullet.getLinkState(scene.robot,tool_index,computeForwardKinematics=True,physicsClientId=scene.client)
        world=np.eye(4);world[:3,:3]=np.array(bullet.getMatrixFromQuaternion(link[5])).reshape(3,3);world[:3,3]=link[4]
        alignment=float(np.max(abs(world-np.array(planner.profile['T_world_base'])@start)))
        check('urdf-c-fk-frame-alignment',alignment<1e-6,dict(matrix_max_error=alignment))
        # 独立几何端点/途经点/恒半径、SLERP正交与四元数正负等价。
        path=PosePath(args.planning_library,start,arc_goal,'arc',via.tolist())
        ps=path.at(0);pe=path.at(1);pv=path.at(path.via_progress)
        check('arc-start-goal-via',max(np.max(abs(ps-start)),np.max(abs(pe-arc_goal)),np.linalg.norm(pv[:3,3]-via))<=1e-10,path.metadata())
        radius_errors=[];orthogonal=[]
        for s in np.linspace(0,1,41):
            T=path.at(float(s));radius_errors.append(abs(np.linalg.norm(T[:3,3]-np.array(path.geometry.origin))-path.geometry.radius));orthogonal.append(np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3))))
        check('arc-radius-slerp-orthogonality',max(radius_errors+orthogonal)<1e-10,dict(radius_max_error_m=max(radius_errors),rotation_matrix_max_error=max(orthogonal)))
        q=quaternion(arc_goal[:3,:3]);check('quaternion-sign-equivalence',np.max(abs(rotation(q)-rotation(-q)))<1e-12,{})
        for angle in (0.,1e-10,math.pi-1e-9,math.pi):
            goalR=start.copy();c,s=math.cos(angle),math.sin(angle);goalR[:3,:3]=np.array([[c,-s,0],[s,c,0],[0,0,1]])@start[:3,:3]
            pp=PosePath(args.planning_library,start,goalR)
            err=np.max(abs(pp.at(1)-goalR));check('orientation-angle-'+str(angle),err<1e-9,dict(endpoint_matrix_error=err,angle_rad=pp.angle))
        for via_synthetic,label in [([-.1,0,0],'major'),([.07071067811865475,.07071067811865475,0],'minor')]:
            A=np.eye(4);B=np.eye(4);A[:3,3]=[.1,0,0];B[:3,3]=[0,.1,0]
            synthetic=PosePath(args.planning_library,A,B,'arc',via_synthetic)
            error=np.linalg.norm(synthetic.at(synthetic.via_progress)[:3,3]-via_synthetic)
            check('arc-'+label+'-contains-via',error<1e-10 and ((abs(synthetic.geometry.sweep)>math.pi)==(label=='major')),synthetic.metadata())
        for name,s,count,expected in [('before',-.1,3,1001),('after',1.1,3,1001),('length',.5,2,1001)]:
            values=[(C.c_double*3)(99,99,99) for _ in range(3)];code=path.fn(C.byref(path.geometry),s,*values,count)
            check('C-position-'+name,code==expected and all(list(v)==[99]*3 for v in values),dict(code=code,output_untouched=all(list(v)==[99]*3 for v in values)))
        record('fixed-orientation-failure',planner.plan(start.tolist(),arc_goal.tolist(),q0,scene,orientation='fixed'),1007)
        record('collinear-arc-failure',planner.plan(start.tolist(),goal.tolist(),q0,scene,method='arc',via=((start[:3,3]+goal[:3,3])/2).tolist()),1001)
        far=goal.copy();far[:3,3]+=[.6,0,0]
        record('ik-step-failure',planner.plan(start.tolist(),far.tolist(),q0,scene,duration_s=.1),2002)
        # 强制时长过短，即使IK逐节点有解也不能接受超速/超加速度。
        record('rate-failure',planner.plan(start.tolist(),goal.tolist(),q0,scene,duration_s=.2),3002)
        apf_goal=[.36,-1.17,1.06,-1.08,1.01,.2]
        result=static_apf(planner,q0,apf_goal,scene);record('apf-waypoints',result,0)
        if result['data']:
            active=max(x['repulsion'] for x in result['data']['iterations'])
            check('apf-repulsion-active',active>0,dict(maximum_repulsion=active))
            timed=time_parameterize(planner,result['data']['waypoints_rad'],scene);record('apf-success',timed,0,True)
        blocked=copy.deepcopy(scene_config);blocked['scene_id']='blocked-start';blocked['obstacles'].append(dict(id='blocking-sphere',kind='sphere',position_m=(np.array(planner.profile['T_world_base'])@np.r_[start[:3,3],1])[:3].tolist(),radius_m=.08))
        # 单独封堵上臂，TCP本身远离球体，证明不是只检查工具点。
        scene.check(q0);bounds=bullet.getAABB(scene.robot,2,physicsClientId=scene.client);center=((np.array(bounds[0])+np.array(bounds[1]))/2).tolist()
        arm_scene=copy.deepcopy(scene_config);arm_scene['scene_id']='blocked-upperarm';arm_scene['obstacles'].append(dict(id='upperarm-blocker',kind='sphere',position_m=center,radius_m=.04))
        arm=StaticScene(arm_scene,planner.profile)
        try:
            record('blocked-upperarm-failure',planner.plan(start.tolist(),goal.tolist(),q0,arm),3003)
            check('upperarm-blocker-not-tcp',np.linalg.norm(np.array(center)-(np.array(planner.profile['T_world_base'])@np.r_[start[:3,3],1])[:3])>.1,dict(blocker_world_m=center))
        finally:arm.close()
        obstruction=StaticScene(blocked,planner.profile)
        try:
            record('blocked-line-failure',planner.plan(start.tolist(),goal.tolist(),q0,obstruction),3003)
            record('blocked-apf-failure',static_apf(planner,q0,apf_goal,obstruction),3003)
        finally:obstruction.close()
        # 只改变算法迭代预算，不改变限值：保留显式预算耗尽失败。
        conf=configuration();conf['apf']['maximum_iterations']=1;negative=out/'budget-config.json';negative.write_text(json.dumps(conf))
        budget=CartesianPlanner(args.library,args.planning_library,config_path=negative)
        record('apf-budget-failure',static_apf(budget,q0,apf_goal,scene),3001);negative.unlink()
        # 共享限值闸门也应用于笛卡尔与势场。
        missing=CartesianPlanner(args.library,args.planning_library,limits_path=out/'absent.json')
        record('missing-limits-failure',missing.plan(start.tolist(),goal.tolist(),q0,scene),1004)
        record('missing-apf-limits-failure',static_apf(missing,q0,apf_goal,scene),1004)
        check('seed-not-mutated',q0==[.3,-1.2,1.1,-1.1,1.,.2],{})
        for name,data in saved:
            if data['maximum_position_error_m'] is not None:
                check(name+'-path-error',data['maximum_position_error_m']<=planner.config['position_tolerance_m'] and data['maximum_orientation_error_rad']<=planner.config['orientation_tolerance_rad'],{})
            else:
                pe,re=pose_error(planner.selector._fk(planner.model,data['q_rad'][-1]),planner.selector._fk(planner.model,apf_goal))
                check(name+'-endpoint-fk-goal',pe<=1e-9 and re<=1e-8,dict(position_error_m=pe,orientation_error_rad=re,path_error_status='not_applicable; APF has no prescribed Cartesian path'))
        # 可视化base系TCP路径和静态球/盒投影；原始3D数据在结果JSON。
        svg=['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="790"><rect width="900" height="790" fill="white"/>',
            '<text x="45" y="30" font-family="sans-serif" font-size="18">UR5 static scene: TCP XY projection / simulation</text>']
        low=np.array([-.9,-.7]);high=np.array([.3,.75]);xy=lambda p:(60+760*(p[0]-low[0])/(high[0]-low[0]),450-370*(p[1]-low[1])/(high[1]-low[1]))
        for index in range(5):
            xv=low[0]+(high[0]-low[0])*index/4;yv=low[1]+(high[1]-low[1])*index/4
            x,y=xy([xv,yv]);svg.append(f'<path d="M{x},80 V450 M60,{y} H820" stroke="#e2e8f0"/><text x="{x}" y="465" font-size="11">{xv:.2f}m</text><text x="10" y="{y}" font-size="11">{yv:.2f}m</text>')
        world=np.array(planner.profile['T_world_base']);inverse=np.linalg.inv(world)
        for obstacle in scene_config['obstacles']:
            p=(inverse@np.r_[obstacle['position_m'],1])[:3];x,y=xy(p)
            if obstacle['kind']=='sphere':svg.append(f'<ellipse cx="{x}" cy="{y}" rx="{obstacle["radius_m"]*760/1.2}" ry="{obstacle["radius_m"]*370/(high[1]-low[1])}" fill="#fee2e2" stroke="#dc2626"/>')
            else:svg.append(f'<rect x="{x-obstacle["half_extents_m"][0]*760/1.2}" y="{y-obstacle["half_extents_m"][1]*370/(high[1]-low[1])}" width="{2*obstacle["half_extents_m"][0]*760/1.2}" height="{2*obstacle["half_extents_m"][1]*370/(high[1]-low[1])}" fill="#fee2e2" stroke="#dc2626"/>')
        for k,(name,data) in enumerate(saved):
            color=['#2563eb','#16a34a','#9333ea'][k];points=[]
            for q in data['q_rad']:
                pose=planner.selector._fk(planner.model,q);x,y=xy(pose[:3,3]);points.append(f'{x},{y}')
            svg.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="2"/><text x="{60+250*k}" y="480" fill="{color}" font-family="sans-serif">{name}</text>')
        svg.append('<text x="60" y="510" font-family="sans-serif" font-size="12">Projection only. Collision evidence uses URDF meshes plus excluded near-assembly pairs.</text>')
        all_points=[planner.selector._fk(planner.model,q)[:3,3] for _,data in saved for q in data['q_rad']]
        lower=np.min(all_points,axis=0)[:2]-.003;upper=np.max(all_points,axis=0)[:2]+.003
        zoom=lambda p:(60+760*(p[0]-lower[0])/(upper[0]-lower[0]),735-170*(p[1]-lower[1])/(upper[1]-lower[1]))
        svg.append('<text x="60" y="550" font-family="sans-serif" font-size="16">TCP XY zoom (same base coordinates)</text>')
        for k,(name,data) in enumerate(saved):
            color=['#2563eb','#16a34a','#9333ea'][k];points=[]
            for q in data['q_rad']:
                pose=planner.selector._fk(planner.model,q);x,y=zoom(pose[:3,3]);points.append(f'{x},{y}')
            svg.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="2"/>')
        svg.append('</svg>')
        (out/'paths.svg').write_text('\n'.join(svg));artifacts.append('paths.svg')
    finally:scene.close()
    passed=sum(c['passed'] for c in cases)
    sources=['src/planning/cartesian.c','src/planning/cartesian.py','src/planning/collision.py','src/planning/apf.py','include/robot_planning.h','scripts/verify_cartesian_planning.py','config/cartesian-planning.v1.json','scenes/static-apf.v1.json','requirements.lock','scripts/verify_joint_planning.py']
    report=dict(status='passed' if passed==len(cases) else 'failed',passed_count=passed,expected_count=len(cases),
        groups=dict(valid_trajectories=dict(total=3,passed=sum(c['passed'] for c in cases if c['id'] in ('line-success','arc-success','apf-success'))),
            expected_negative_requests=dict(total=sum(c.get('expected_code',0)!=0 for c in cases),passed=sum(c['passed'] and c.get('expected_code',0)!=0 for c in cases)),
            note='33 is expected-check pass count; negative requests are not successful trajectories'),
        environment=dict(python=platform.python_version(),numpy=np.__version__,pybullet=version('pybullet'),platform=platform.platform()),
        sources_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources},
        libraries_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.library,args.planning_library)},
        effective_limits=planner.limits.as_dict(),algorithm_configuration=configuration(),scene=scene_config,
        validation_scope='continuous joint polynomial extrema, conservative DH TCP derivative bounds and modeled-geometry collision motion bounds; path error sampled at reported resolution; no Windows GUI',
        artifacts=[dict(path=p,sha256=hashlib.sha256((out/p).read_bytes()).hexdigest()) for p in artifacts],cases=cases)
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(f'{passed}/{len(cases)} Cartesian/APF checks passed')
    for c in cases:
        if not c['passed']:print(c)
    return 0 if passed==len(cases) else 1
if __name__=='__main__':sys.exit(main())
