"""Matched APF business-task comparison on the actual fixed-transfer physics plant."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import platform
import os
from importlib.metadata import version
import sys
import time
import traceback
import numpy as np
import pybullet as p
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from adapters.c_kinematics import ContractError
from planning.cartesian import CartesianPlanner, quaternion
from planning.apf import static_apf, time_parameterize
from planning.guided_apf import improved_plan
from control.six_axis import SixAxisExecutor,load_execution_config
from verify_fixed_transfer import PayloadScene,TransferPlant,SegmentClock
from verify_six_axis import write_trace


def save(path,value):
    """Write finite, complete evidence; errors must never reuse a prior report."""
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def stats(values):
    """All supplied samples, linear h=(n-1)p quantiles; empty groups stay null."""
    if not values:return None
    return dict(n=len(values),mean=float(np.mean(values)),median=float(np.median(values)),
                p95=float(np.quantile(values,.95,method='linear')),maximum=max(values),minimum=min(values))


def traditional(planner,start,goal,scene):
    """Original APF and rest-at-each-waypoint timing, with complete certification."""
    geometric=static_apf(planner,start,goal,scene)
    if geometric['code']:return dict(geometric,next_state=None)
    timed=time_parameterize(planner,geometric['data']['waypoints_rad'],scene)
    return dict(timed,next_state=None,details=dict(geometric=geometric,timing=timed.get('details')))


def setup_recipe(args,out,experiment,task):
    """Shared offline IK endpoints and scene; no method gets a different via point."""
    planner=CartesianPlanner(args.library,args.planning_library);planner.setup()
    home=planner.selector._fk(planner.model,task['task_origin_q_rad'])
    world=np.array(planner.profile['T_world_base']);pick=world@home
    pick[:3,3]+=task['pick_world_delta_m']
    via=pick.copy();via[:3,3]+=task['via_world_delta_from_pick_m']
    place=pick.copy();place[:3,3]+=task['place_world_delta_from_pick_m']
    offset=np.array(task['T_tool_payload_translation_m'])
    initial=pick[:3,3]+pick[:3,:3]@offset;target=place[:3,3]+place[:3,:3]@offset
    q=quaternion(pick[:3,:3]);orientation=[*q[1:],q[0]]
    definition=json.loads((ROOT/'scenes/static-apf.v1.json').read_text())
    definition.update(scene_id='fixed-transfer-apf-paired-v1',obstacles=[dict(id='payload-route-barrier',kind='box',
        position_m=((initial+target)/2).tolist(),half_extents_m=task['barrier_half_extents_m'])])
    save(out/'scene.json',definition)
    scene=PayloadScene(definition,planner.profile,task,initial.tolist(),orientation)
    endpoints=[];start=list(task['task_origin_q_rad']);began=time.perf_counter()
    try:
        for i,(name,T) in enumerate([('pick',pick),('avoidance_via',via),('place',place),('return',world@home)]):
            if name=='return':end=list(task['task_origin_q_rad'])
            else:
                result=planner.plan(planner.selector._fk(planner.model,start).tolist(),
                    (np.linalg.inv(world)@T).tolist(),start,scene,
                    duration_s=task['segment_durations_s'][i],orientation='slerp_shortest')
                save(out/('recipe-'+name+'.json'),result)
                if result['code']:raise ContractError(result['code'],'Shared offline IK recipe: '+result['message'])
                end=result['data']['q_rad'][-1]
            endpoints.append((name,end));start=end
            if name=='pick':scene.attached=True
            if name=='place':
                scene.attached=False
                oq=quaternion(place[:3,:3]);scene.fixed_pose=(target.tolist(),[*oq[1:],oq[0]])
    finally:scene.close()
    planning=json.loads((ROOT/'config/cartesian-planning.v1.json').read_text())
    planning.update(maximum_duration_s=experiment['maximum_stage_duration_s'],maximum_nodes=experiment['maximum_stage_nodes'])
    save(out/'matched-planning.json',planning)
    limits=json.loads((ROOT/'config/planning-limits.v1.json').read_text())
    limits['joint_limits']['velocity_rad_s']=[experiment['planning_velocity_rad_s']/limits['scales']['velocity']]*6
    limits['joint_limits']['acceleration_rad_s2']=[experiment['planning_acceleration_rad_s2']/limits['scales']['acceleration']]*6
    limits['sources']['joint_velocity']='Matched fixed-transfer APF experiment; 0.04 rad/s effective, below unchanged 0.05 rad/s controller limit'
    limits['sources']['joint_acceleration']='Matched fixed-transfer APF experiment; 0.4 rad/s^2 effective, below unchanged 0.5 rad/s^2 controller limit'
    save(out/'matched-limits.json',limits)
    recipe=dict(endpoints=endpoints,initial_payload_m=initial.tolist(),target_payload_m=target.tolist(),
                orientation_xyzw=orientation,definition=definition,offline_recipe_wall_s=time.perf_counter()-began,
                task=task,shared_via=True)
    save(out/'recipe.json',recipe)
    return recipe


def trial(args,out,experiment,recipe,method,repetition):
    """One fresh identical plant and full pick/via/place/home closed-loop attempt."""
    folder=out/(method+'-'+str(repetition));folder.mkdir()
    task=recipe['task'];report=dict(method=method,repetition=repetition,success=False,stages=[],failure=None,
        planning_wall_s=0.,simulation_motion_s=0.,planned_joint_path_rad=0.,planned_tcp_path_m=0.,
        intermediate_rest_knots=0,placement_error_m=None,retention_error_m=None,return_error_rad=None,
        tracking_rmse_rad=None,minimum_actual_clearance_m=None,maximum_attachment_error_m=None)
    plant=scene=None;tracking=[]
    try:
        planner=CartesianPlanner(args.library,args.planning_library,config_path=out/'matched-planning.json',limits_path=out/'matched-limits.json')
        planner.setup()
        config=load_execution_config(ROOT/'config'/task['execution_config']);config['hold_s']=task['settle_s']
        config['initial_q_rad']=task['task_origin_q_rad']
        scene=PayloadScene(recipe['definition'],planner.profile,task,recipe['initial_payload_m'],recipe['orientation_xyzw'])
        plant=TransferPlant(config,planner.profile,task,recipe['definition'],
            (recipe['initial_payload_m'],recipe['orientation_xyzw']),folder)
        for name,goal in recipe['endpoints']:
            plant.stage=name;plant.origin_tick=plant.global_clock.tick;plant.stopped=False
            start=plant.read()['q_rad'];before=copy.deepcopy(start);t=time.perf_counter()
            fn=traditional if method=='traditional' else improved_plan
            planned=fn(planner,start,goal,scene) if method=='traditional' else fn(planner,start,goal,scene,ROOT/experiment['algorithm_config'])
            elapsed=time.perf_counter()-t;report['planning_wall_s']+=elapsed
            save(folder/(name+'-planning.json'),planned)
            stage=dict(name=name,code=planned['code'],planning_wall_s=elapsed,input_unchanged=start==before,
                start_global_tick=plant.global_clock.tick,execution_code=None)
            report['stages'].append(stage)
            if start!=before:raise AssertionError('Planner changed input state')
            if planned['code']:
                if planned.get('data') is not None:raise AssertionError('Failed planner exposed executable data')
                raise ContractError(planned['code'],name+': '+planned['message'])
            data=planned['data'];q=np.array(data['q_rad'])
            tcp=np.array([planner.selector._fk(planner.model,x)[:3,3] for x in q])
            report['planned_joint_path_rad']+=float(np.linalg.norm(np.diff(q,axis=0),axis=1).sum())
            report['planned_tcp_path_m']+=float(np.linalg.norm(np.diff(tcp,axis=0),axis=1).sum())
            if method=='traditional':knots=len(planned['details']['geometric']['data']['waypoints_rad'])-2
            else:knots=len(planned['details']['stages']['smoothing']['data']['waypoints_rad'])-2
            report['intermediate_rest_knots']+=max(0,knots)
            stage.update(trajectory_duration_s=data['time_s'][-1],certified=data['trajectory_status'],
                certified_minimum_clearance_m=data['certified_minimum_distance_lower_bound_m'],
                continuous_tcp_proof=data['continuous_tcp_proof'],continuous_collision_proof=data['continuous_collision_proof'])
            executor=SixAxisExecutor(args.control_library,config,plant,scene,planner.profile,SegmentClock(plant.global_clock))
            result=executor.run(planned);stage['execution_code']=result['code']
            save(folder/(name+'-execution.json'),result)
            for row in executor.rows:
                row.update(global_tick=stage['start_global_tick']+row['tick'],stage=name)
                tracking.extend((np.array(row['reference_rad'])-row['q_rad']).tolist())
            write_trace(folder/(name+'-control.csv'),executor.rows)
            save(folder/(name+'-events.json'),executor.events)
            if result['code']:raise ContractError(result['code'],name+': '+result['message'])
            if name=='pick':stage['capture_error_m']=plant.grasp();scene.attached=True
            if name=='place':
                pose=plant.release();scene.attached=False;scene.fixed_pose=pose
                report['placement_error_m']=float(np.linalg.norm(np.array(pose[0])-recipe['target_payload_m']))
                if report['placement_error_m']>task['placement_tolerance_m']:raise AssertionError('Placement tolerance exceeded')
            stage['end_global_tick']=plant.global_clock.tick
            print(method,repetition,name,'passed tick',plant.global_clock.tick,flush=True)
        report['return_error_rad']=float(np.max(abs(np.array(plant.read()['q_rad'])-task['task_origin_q_rad'])))
        actual=p.getBasePositionAndOrientation(plant.payload,physicsClientId=plant.client)[0]
        report['retention_error_m']=float(np.linalg.norm(np.array(actual)-recipe['target_payload_m']))
        if report['return_error_rad']>task['return_tolerance_rad'] or report['retention_error_m']>task['placement_tolerance_m']:
            raise AssertionError('Final return or object retention failed')
        if [x['global_tick'] for x in plant.writes]!=list(range(plant.global_clock.tick)):raise AssertionError('Clock gap or duplicate')
        if len(plant.zero_targets)!=4:raise AssertionError('Missing stage zero targets')
        report['success']=True
    except Exception as exc:
        report['failure']=dict(code=getattr(exc,'code',9000),message=str(exc),traceback=traceback.format_exc())
        print(method,repetition,'FAILED',report['failure']['code'],str(exc),flush=True)
    finally:
        if plant:
            report['simulation_motion_s']=plant.global_clock.tick*plant.dt_s
            report['minimum_actual_clearance_m']=min((x['physical_payload_clearance_m'] for x in plant.feedback),default=None)
            report['maximum_attachment_error_m']=max((x['attachment_error_m'] or 0 for x in plant.feedback),default=None)
            report['zero_target_count']=len(plant.zero_targets);report['global_ticks']=plant.global_clock.tick
            write_trace(folder/'global-commands.csv',plant.writes);write_trace(folder/'object-feedback.csv',plant.feedback)
            write_trace(folder/'zero-targets.csv',plant.zero_targets)
            try:plant.close()
            except Exception as exc:report['success']=False;report['failure']=dict(code=9000,message=str(exc))
        if scene:scene.close()
        report['tracking_rmse_rad']=float(np.sqrt(np.mean(np.square(tracking)))) if tracking else None
        report['serial_attempt_s']=report['simulation_motion_s']+report['planning_wall_s']
        save(folder/'report.json',report)
    return report


def summarize(trials):
    """Count failures in denominator and elapsed cost; successful-only metrics labelled."""
    good=[x for x in trials if x['success']]
    total=sum(x['serial_attempt_s'] for x in trials)
    return dict(attempts=len(trials),successes=len(good),failures=len(trials)-len(good),
        success_rate=len(good)/len(trials),failure_codes=[x['failure']['code'] for x in trials if not x['success']],
        attempt_time_s=total,observed_attempt_goodput_per_h=3600*len(good)/total if total else None,
        all_attempt_planning_wall_s=stats([x['planning_wall_s'] for x in trials]),
        successful_only={key:stats([x[key] for x in good]) for key in ('simulation_motion_s','serial_attempt_s',
            'planned_joint_path_rad','planned_tcp_path_m','intermediate_rest_knots','placement_error_m',
            'retention_error_m','return_error_rad','tracking_rmse_rad','minimum_actual_clearance_m','maximum_attachment_error_m')})


def main():
    """Fresh outputs, alternating paired order, complete evidence on every failure."""
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('library','planning-library','control-library'):
        parser.add_argument('--'+key,type=Path,default=ROOT/('build-core/librobot_'+{'library':'contract','planning-library':'planning','control-library':'control'}[key]+'.so'))
    parser.add_argument('--output',type=Path,default=ROOT/'results/fixed-transfer-apf')
    args=parser.parse_args();out=args.output.resolve()
    if out.exists():raise FileExistsError('Fresh output required: '+str(out))
    out.mkdir(parents=True)
    experiment=json.loads((ROOT/'config/fixed-transfer-apf.v1.json').read_text())
    task=json.loads((ROOT/experiment['task_config']).read_text())
    report=dict(passed=False,experiment=experiment,trials=[],environment=dict(python=platform.python_version(),
        numpy=np.__version__,pybullet=version('pybullet'),platform=platform.platform()),source_commit=os.environ.get('GITHUB_SHA'),
        libraries_sha256={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in (args.library,args.planning_library,args.control_library)},limitations=[
        'Three deterministic pairs are observations, not statistical generalization or formal latency acceptance.',
        'Both methods receive the prescribed business via point. This does not measure autonomous via discovery.',
        'The traditional reference already includes whole-arm collision and goal-distance-weighted repulsion; not every APF variant.',
        'Recipe IK is shared offline preparation, excluded from online planning time; scene creation, FK path measurement and video excluded.',
        'Planning time includes all APF retries, smoothing, timing and certification. Failures remain in counts and costs.',
        'Serial attempt cost = simulated motion/hold time + online planning wall time, not script wall time.',
        'Goodput is a simulation proxy without recovery, loading, communication, calibration or real grasp costs.',
        'No energy or hardware throughput claim. Ideal grasp, static nests and Linux x86_64 ABI scope retained.'])
    try:
        recipe=setup_recipe(args,out,experiment,task)
        report['recipe']=recipe
        # Warm each complete planning method once on the common first phase, no physics.
        warm=CartesianPlanner(args.library,args.planning_library,config_path=out/'matched-planning.json',limits_path=out/'matched-limits.json');warm.setup()
        scene=PayloadScene(recipe['definition'],warm.profile,task,recipe['initial_payload_m'],recipe['orientation_xyzw'])
        try:
            for method,fn in [('traditional',traditional),('improved',improved_plan)]:
                save(out/('warmup-'+method+'.json'),fn(warm,task['task_origin_q_rad'],recipe['endpoints'][0][1],scene) if method=='traditional' else fn(warm,task['task_origin_q_rad'],recipe['endpoints'][0][1],scene,ROOT/experiment['algorithm_config']))
        finally:scene.close()
        for i in range(experiment['repetitions']):
            order=experiment['order'] if i%2==0 else list(reversed(experiment['order']))
            for method in order:report['trials'].append(trial(args,out,experiment,recipe,method,i+1))
        report['summary']={method:summarize([x for x in report['trials'] if x['method']==method]) for method in experiment['order']}
        pairs=[]
        for i in range(1,experiment['repetitions']+1):
            pair={x['method']:x for x in report['trials'] if x['repetition']==i}
            if all(x['success'] for x in pair.values()):
                a,b=pair['traditional'],pair['improved']
                pairs.append(dict(repetition=i,motion_saved_s=a['simulation_motion_s']-b['simulation_motion_s'],
                    motion_reduction_percent=100*(1-b['simulation_motion_s']/a['simulation_motion_s']),
                    serial_saved_s=a['serial_attempt_s']-b['serial_attempt_s'],
                    serial_reduction_percent=100*(1-b['serial_attempt_s']/a['serial_attempt_s'])))
        report['successful_pairs']=pairs
        report['passed']=len(report['trials'])==2*experiment['repetitions'] and any(x['success'] for x in report['trials']) and not any(
            x['failure'] and x['failure']['code']==9000 for x in report['trials'])
    except Exception as exc:report['failure']=dict(message=str(exc),traceback=traceback.format_exc());print(report['failure']['traceback'],flush=True)
    finally:
        paths=['scripts/verify_transfer_apf.py','scripts/verify_fixed_transfer.py','src/planning/apf.py','src/planning/guided_apf.py',
               'config/fixed-transfer-apf.v1.json','config/fixed-transfer.v1.json','config/guided-apf.v1.json',
               'config/cartesian-planning.v1.json','config/planning-limits.v1.json','config/six-axis-execution.v1.json']
        report['source_sha256']={x:hashlib.sha256((ROOT/x).read_bytes()).hexdigest() for x in paths}
        save(out/'report.json',report)
        print('APF_BUSINESS_SUMMARY '+json.dumps({k:report.get(k) for k in ('passed','summary','successful_pairs','failure')},ensure_ascii=False,allow_nan=False),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
