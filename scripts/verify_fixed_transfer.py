"""Fixed origin/pick/via/place/return simulation, certified per segment, with video."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import traceback
import numpy as np
import pybullet as p
from PIL import Image, ImageDraw
import imageio_ffmpeg
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from adapters.c_kinematics import ContractError
from planning.cartesian import CartesianPlanner
from planning.collision import StaticScene
from control.six_axis import TickClock, SixAxisExecutor, load_execution_config
from verify_six_axis import SixAxisPlant, write_trace


def body_pose(robot, link, client):
    """Return world pose of a URDF link frame, not its inertial COM frame."""
    state=p.getLinkState(robot,link,computeForwardKinematics=True,physicsClientId=client)
    return state[4],state[5]


def payload_pose(robot, link, client, offset):
    """Rigid tool-relative payload pose used by collision certification."""
    return p.multiplyTransforms(*body_pose(robot,link,client),offset,[0,0,0,1])


class PayloadScene(StaticScene):
    """Include the carried box in whole-arm continuous motion-envelope checks."""
    def __init__(self, definition, profile, task, position, orientation):
        super().__init__(definition,profile)
        self.task=task;self.attached=False
        self.fixed_pose=(list(position),list(orientation))
        self.tool=next(i for i in range(p.getNumJoints(self.robot,physicsClientId=self.client))
                      if p.getJointInfo(self.robot,i,physicsClientId=self.client)[12].decode()=='tool0')
        self.ignored=[i for i in range(p.getNumJoints(self.robot,physicsClientId=self.client))
                      if p.getJointInfo(self.robot,i,physicsClientId=self.client)[12].decode() in
                      ('wrist_3_link','ee_link','tool0')]
        shape=p.createCollisionShape(p.GEOM_BOX,halfExtents=task['payload_half_extents_m'],physicsClientId=self.client)
        self.payload=p.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,basePosition=position,
                                      baseOrientation=orientation,physicsClientId=self.client)
        # Existing proof uses this bound for every moving point. Enlarge it for
        # the tool offset and all payload vertices; static payloads need no motion.
        self.motion_radius_bound_m+=float(np.linalg.norm(task['T_tool_payload_translation_m'])+
                                          np.linalg.norm(task['payload_half_extents_m']))

    def check(self,q):
        result=super().check(q)
        pose=payload_pose(self.robot,self.tool,self.client,self.task['T_tool_payload_translation_m']) if self.attached else self.fixed_pose
        p.resetBasePositionAndOrientation(self.payload,*pose,physicsClientId=self.client)
        minimum=result['minimum_distance_lower_bound_m'];nearest=result['nearest']
        for name,body in self.obstacles:
            for point in p.getClosestPoints(self.payload,body,self.query,physicsClientId=self.client):
                if point[8]<minimum: minimum=point[8];nearest=dict(payload_obstacle=name)
        for point in p.getClosestPoints(self.robot,self.payload,self.query,physicsClientId=self.client):
            if self.attached and point[3] in self.ignored: continue
            if point[8]<minimum: minimum=point[8];nearest=dict(payload_robot_link=point[3])
        return dict(passed=minimum>self.clearance,minimum_distance_lower_bound_m=minimum,nearest=nearest)

    def metadata(self):
        metadata=super().metadata()
        metadata['payload']=dict(attached=self.attached,mass_kg=self.task['payload_mass_kg'],
            half_extents_m=self.task['payload_half_extents_m'],tool_translation_m=self.task['T_tool_payload_translation_m'],
            fixed_world_pose=None if self.attached else self.fixed_pose,
            attached_contact_exclusions=self.ignored)
        return metadata


class SegmentClock:
    """A local trajectory-time view of the one scenario clock; no second clock."""
    def __init__(self,parent): self.parent=parent;self.origin=parent.tick;self.dt=parent.dt
    @property
    def tick(self): return self.parent.tick-self.origin
    def stamp(self): return self.tick,self.tick*self.dt
    def advance(self): self.parent.advance()


class TransferPlant(SixAxisPlant):
    """Six physical velocity servos plus an explicit ideal grasp constraint."""
    def __init__(self,config,profile,task,definition,pose,out):
        super().__init__(config,profile)
        self.task=task;self.out=out;self.global_clock=TickClock(self.dt_s)
        self.origin_tick=0;self.stage='origin';self.feedback=[];self.writes=[];self.constraint=None
        self.attached=False;self.last_command=np.zeros(6);self.frames=0;self.video=None
        self.tool=next(i for i in range(p.getNumJoints(self.robot,physicsClientId=self.client))
                      if p.getJointInfo(self.robot,i,physicsClientId=self.client)[12].decode()=='tool0')
        self.ignored=[i for i in range(p.getNumJoints(self.robot,physicsClientId=self.client))
                      if p.getJointInfo(self.robot,i,physicsClientId=self.client)[12].decode() in
                      ('wrist_3_link','ee_link','tool0')]
        self.obstacles=[]
        for obstacle in definition['obstacles']:
            shape=p.createCollisionShape(p.GEOM_BOX,halfExtents=obstacle['half_extents_m'],physicsClientId=self.client)
            visual=p.createVisualShape(p.GEOM_BOX,halfExtents=obstacle['half_extents_m'],rgbaColor=[.85,.12,.1,1],physicsClientId=self.client)
            self.obstacles.append(p.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,baseVisualShapeIndex=visual,
                basePosition=obstacle['position_m'],physicsClientId=self.client))
        shape=p.createCollisionShape(p.GEOM_BOX,halfExtents=task['payload_half_extents_m'],physicsClientId=self.client)
        visual=p.createVisualShape(p.GEOM_BOX,halfExtents=task['payload_half_extents_m'],rgbaColor=[1,.65,.05,1],physicsClientId=self.client)
        self.payload=p.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,baseVisualShapeIndex=visual,
            basePosition=pose[0],baseOrientation=pose[1],physicsClientId=self.client)
        self.focus=np.array(pose[0])+[.04,0,.025]
        self.render_stride=round(1/(task['video_fps']*self.dt_s))
        if abs(self.render_stride*self.dt_s*task['video_fps']-1)>1e-10:
            raise ContractError(1006,'Video frames must align with scenario ticks')
        width,height=task['video_width'],task['video_height']
        command=[imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','error','-f','rawvideo','-vcodec','rawvideo',
            '-s',str(width)+'x'+str(height),'-pix_fmt','rgb24','-r',str(task['video_fps']),'-i','-',
            '-an','-c:v','libx264','-crf','27','-pix_fmt','yuv420p',str(out/'demo.mp4')]
        self.video=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=(out/'video-encoder.log').open('wb'))

    def read(self):
        feedback=super().read()
        feedback.update(tick=self.global_clock.tick-self.origin_tick,
                        time_s=(self.global_clock.tick-self.origin_tick)*self.dt_s)
        position,orientation=p.getBasePositionAndOrientation(self.payload,physicsClientId=self.client)
        expected=payload_pose(self.robot,self.tool,self.client,self.task['T_tool_payload_translation_m'])
        error=float(np.linalg.norm(np.array(position)-expected[0])) if self.attached else None
        minimum=.25
        for body in self.obstacles:
            for point in p.getClosestPoints(self.payload,body,.25,physicsClientId=self.client): minimum=min(minimum,point[8])
        for point in p.getClosestPoints(self.robot,self.payload,.25,physicsClientId=self.client):
            if self.attached and point[3] in self.ignored: continue
            minimum=min(minimum,point[8])
        self.feedback.append(dict(stage=self.stage,global_tick=self.global_clock.tick,
            global_time_s=self.global_clock.tick*self.dt_s,segment_tick=feedback['tick'],
            object_position_m=list(position),object_orientation_xyzw=list(orientation),attached=self.attached,
            attachment_error_m=error,physical_payload_clearance_m=minimum))
        if minimum<=.003: raise ContractError(3003,'Actual payload collision/clearance violation')
        if error is not None and error>self.task['attachment_tolerance_m']:
            raise ContractError(4001,'Payload detached from ideal grasp')
        return feedback

    def advance(self,command):
        if np.max(abs(np.array(command)-self.last_command))>self.config['acceleration_limit_rad_s2']*self.dt_s+1e-8:
            raise ContractError(4001,'Cross-stage velocity command jump')
        self.writes.append(dict(stage=self.stage,global_tick=self.global_clock.tick,
            global_time_s=self.global_clock.tick*self.dt_s,command_rad_s=list(command)))
        super().advance(command)
        self.last_command=np.array(command)
        if (self.global_clock.tick+1)%self.render_stride==0:
            self.render(time_s=(self.global_clock.tick+1)*self.dt_s)

    def stop(self):
        super().stop()
        self.last_command=np.zeros(6)

    def grasp(self):
        actual=p.getBasePositionAndOrientation(self.payload,physicsClientId=self.client)
        desired=payload_pose(self.robot,self.tool,self.client,self.task['T_tool_payload_translation_m'])
        error=float(np.linalg.norm(np.array(actual[0])-desired[0]))
        if error>self.task['grasp_tolerance_m']: raise ContractError(4001,'Outside grasp capture tolerance')
        mass=self.task['payload_mass_kg'];h=np.array(self.task['payload_half_extents_m'])
        inertia=(mass/3*np.array([h[1]**2+h[2]**2,h[0]**2+h[2]**2,h[0]**2+h[1]**2])).tolist()
        p.changeDynamics(self.payload,-1,mass=mass,localInertiaDiagonal=inertia,physicsClientId=self.client)
        state=p.getLinkState(self.robot,self.tool,computeForwardKinematics=True,physicsClientId=self.client)
        local=p.multiplyTransforms(*p.invertTransform(state[0],state[1]),*desired)
        self.constraint=p.createConstraint(self.robot,self.tool,self.payload,-1,p.JOINT_FIXED,[0,0,0],
            local[0],[0,0,0],parentFrameOrientation=local[1],physicsClientId=self.client)
        p.changeConstraint(self.constraint,maxForce=30,erp=.8,physicsClientId=self.client)
        self.attached=True
        return error

    def release(self):
        p.removeConstraint(self.constraint,physicsClientId=self.client);self.constraint=None
        p.changeDynamics(self.payload,-1,mass=0,physicsClientId=self.client)
        p.resetBaseVelocity(self.payload,[0,0,0],[0,0,0],physicsClientId=self.client)
        self.attached=False
        return p.getBasePositionAndOrientation(self.payload,physicsClientId=self.client)

    def render(self,time_s=None,key=None):
        width,height=self.task['video_width'],self.task['video_height']
        def view(target,distance,yaw,pitch,w,h):
            camera=p.computeViewMatrixFromYawPitchRoll(list(target),distance,yaw,pitch,0,2)
            projection=p.computeProjectionMatrixFOV(48,w/h,.02,4)
            data=p.getCameraImage(w,h,camera,projection,renderer=p.ER_TINY_RENDERER,physicsClientId=self.client)
            return Image.fromarray(np.array(data[2],dtype=np.uint8).reshape(h,w,4)[:,:,:3])
        image=view([.25,.15,.35],1.55,55,-25,width,height)
        image.paste(view(self.focus,.32,80,-25,300,230),(width-300,height-230))
        draw=ImageDraw.Draw(image);draw.rectangle((0,0,width,52),fill='white')
        t=self.global_clock.tick*self.dt_s if time_s is None else time_s
        draw.text((10,8),'UR5 SIMULATION | '+self.stage+' | t='+format(t,'.2f')+' s',fill='black')
        draw.text((10,28),'IDEAL GRASP | orange: payload  red: barrier | cycle 20 ms',fill='black')
        if key: image.save(self.out/(key+'.png'))
        if self.video and time_s is not None:
            self.video.stdin.write(np.asarray(image).tobytes());self.frames+=1

    def close(self):
        if getattr(self,'video',None):
            self.video.stdin.close();code=self.video.wait(timeout=30);self.video=None
            if code: raise RuntimeError('Video encoder failed: '+str(code))
        super().close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,default=ROOT/'build-core/librobot_contract.so')
    parser.add_argument('--planning-library',type=Path,default=ROOT/'build-core/librobot_planning.so')
    parser.add_argument('--control-library',type=Path,default=ROOT/'build-core/librobot_control.so')
    parser.add_argument('--output',type=Path,default=ROOT/'results/fixed-transfer')
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    report=dict(passed=False,usage='simulation_only',gripper='ideal fixed constraint; virtual static pickup/place nests',
                stages=[],events=[])
    plant=scene=None
    try:
        task=json.loads((ROOT/'config/fixed-transfer.v1.json').read_text())
        config=load_execution_config(ROOT/'config'/task['execution_config']);config['hold_s']=task['settle_s']
        config['initial_q_rad']=task['task_origin_q_rad']
        planner=CartesianPlanner(args.library,args.planning_library,config_path=config['planning_config_path']);planner.setup()
        home=planner.selector._fk(planner.model,task['task_origin_q_rad'])
        world=np.array(planner.profile['T_world_base']);pick_world=world@home
        pick_world[:3,3]+=task['pick_world_delta_m']
        via_world=pick_world.copy();via_world[:3,3]+=task['via_world_delta_from_pick_m']
        place_world=pick_world.copy();place_world[:3,3]+=task['place_world_delta_from_pick_m']
        offset=np.array(task['T_tool_payload_translation_m'])
        object_pick=pick_world[:3,3]+pick_world[:3,:3]@offset
        object_place=place_world[:3,3]+place_world[:3,:3]@offset
        orientation=p.getQuaternionFromEuler([0,0,0])
        from planning.cartesian import quaternion
        wxyz=quaternion(pick_world[:3,:3]);orientation=[*wxyz[1:],wxyz[0]]
        definition=json.loads((ROOT/'scenes/static-apf.v1.json').read_text())
        definition.update(scene_id='ur5-fixed-transfer-v1',obstacles=[dict(id='payload-route-barrier',kind='box',
            position_m=((object_pick+object_place)/2).tolist(),half_extents_m=task['barrier_half_extents_m'])])
        (out/'scene.json').write_text(json.dumps(definition,indent=2)+'\n')
        scene=PayloadScene(definition,planner.profile,task,object_pick.tolist(),orientation)
        plant=TransferPlant(config,planner.profile,task,definition,(object_pick.tolist(),orientation),out)
        report.update(parameters=task,period_s=config['dt_s'],initial_payload_world_m=object_pick.tolist(),
                      target_payload_world_m=object_place.tolist(),origin_q_rad=task['task_origin_q_rad'])
        plant.render(key='00-origin')
        report['events'].append(dict(event='origin',global_tick=0,time_s=0))
        targets=[('pick',np.linalg.inv(world)@pick_world),('avoidance_via',np.linalg.inv(world)@via_world),
                 ('place',np.linalg.inv(world)@place_world),('return',home)]
        for index,((name,target),duration) in enumerate(zip(targets,task['segment_durations_s'])):
            plant.stage=name;plant.origin_tick=plant.global_clock.tick;plant.stopped=False
            feedback=plant.read();q_start=feedback['q_rad']
            start=planner.selector._fk(planner.model,q_start)
            if name=='avoidance_via':
                direct=planner.plan(start.tolist(),(np.linalg.inv(world)@place_world).tolist(),q_start,scene,
                                    duration_s=duration,orientation='slerp_shortest')
                report['direct_route_counterexample']=direct
                if direct['code']!=3003 or direct['data'] is not None:
                    raise AssertionError('Straight carried-payload route must collide and return 3003')
            planned=planner.plan(start.tolist(),target.tolist(),q_start,scene,duration_s=duration,orientation='slerp_shortest')
            (out/(name+'-planning.json')).write_text(json.dumps(planned,indent=2,allow_nan=False)+'\n')
            if planned['code']: raise ContractError(planned['code'],name+': '+planned['message'])
            executor=SixAxisExecutor(args.control_library,config,plant,scene,planner.profile,SegmentClock(plant.global_clock))
            origin_tick=plant.global_clock.tick
            result=executor.run(planned)
            for row in executor.rows:
                row.update(stage=name,global_tick=origin_tick+row['tick'],global_time_s=(origin_tick+row['tick'])*config['dt_s'])
            write_trace(out/(name+'-control.csv'),executor.rows)
            (out/(name+'-events.json')).write_text(json.dumps(executor.events,indent=2,allow_nan=False)+'\n')
            entry=dict(stage=name,code=result['code'],start_global_tick=origin_tick,end_global_tick=plant.global_clock.tick,
                control_samples=len(executor.rows),events=executor.events,ik_nodes=len(planned['data']['ik_nodes']),
                continuous_collision_proof=planned['data']['continuous_collision_proof'],
                minimum_certified_clearance_m=planned['data']['certified_minimum_distance_lower_bound_m'])
            report['stages'].append(entry)
            if result['code']: raise ContractError(result['code'],result['message'])
            report['events'].append(dict(event=name+'_reached',global_tick=plant.global_clock.tick,
                                        time_s=plant.global_clock.tick*config['dt_s']))
            if name=='pick':
                entry['capture_error_m']=plant.grasp();scene.attached=True
                report['events'].append(dict(event='grasp_attached',global_tick=plant.global_clock.tick,
                    mass_kg=task['payload_mass_kg'],capture_error_m=entry['capture_error_m']))
            if name=='place':
                pose=plant.release();scene.attached=False;scene.fixed_pose=pose
                entry['placement_error_m']=float(np.linalg.norm(np.array(pose[0])-object_place))
                if entry['placement_error_m']>task['placement_tolerance_m']: raise AssertionError('Placement error')
                report['events'].append(dict(event='payload_released',global_tick=plant.global_clock.tick,
                    actual_position_m=list(pose[0]),placement_error_m=entry['placement_error_m']))
            plant.render(key=str(index+1).zfill(2)+'-'+name)
            print(name,entry['code'],'tick',plant.global_clock.tick,flush=True)
        final=plant.read()
        report['return_max_error_rad']=float(np.max(abs(np.array(final['q_rad'])-task['task_origin_q_rad'])))
        if report['return_max_error_rad']>task['return_tolerance_rad']: raise AssertionError('Return error')
        report['actual_payload_final_world_m']=list(p.getBasePositionAndOrientation(plant.payload,physicsClientId=plant.client)[0])
        report['actual_payload_retention_error_m']=float(np.linalg.norm(np.array(report['actual_payload_final_world_m'])-object_place))
        if report['actual_payload_retention_error_m']>task['placement_tolerance_m']: raise AssertionError('Payload moved after release')
        ticks=[r['global_tick'] for r in plant.writes]
        if ticks!=list(range(plant.global_clock.tick)): raise AssertionError('Global clock sequence mismatch')
        report.update(passed=True,global_ticks=plant.global_clock.tick,duration_s=plant.global_clock.tick*config['dt_s'],
            timing_error_count=0,control_error_count=0,command_jump_count=0,video_frames=plant.frames,
            maximum_attachment_error_m=max(r['attachment_error_m'] or 0 for r in plant.feedback),
            minimum_actual_payload_clearance_m=min(r['physical_payload_clearance_m'] for r in plant.feedback))
    except Exception as exc:
        report['failure']=dict(code=getattr(exc,'code',9000),message=str(exc),traceback=traceback.format_exc())
        print(report['failure']['traceback'],flush=True)
    finally:
        if plant:
            write_trace(out/'object-feedback.csv',plant.feedback);write_trace(out/'global-commands.csv',plant.writes)
            try: plant.close()
            except Exception as exc: report.update(passed=False,video_error=str(exc))
        if scene: scene.close()
        report['source_sha256']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [Path(__file__).resolve(),ROOT/'config/fixed-transfer.v1.json',ROOT/'src/control/six_axis.py',
                         ROOT/'config/cartesian-planning.v1.json',ROOT/'config/six-axis-execution.v1.json']}
        (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return 0 if report['passed'] else 1


if __name__=='__main__': sys.exit(main())
