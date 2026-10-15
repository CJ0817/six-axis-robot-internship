"""PyBullet DIRECT静态场景，全机器人连杆距离/自碰撞检查，仿真用途。"""
import hashlib
from importlib.metadata import version
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
import pybullet as p
from adapters.c_kinematics import ContractError, numbers

ROOT=Path(__file__).resolve().parents[2]

class StaticScene:
    """独立DIRECT连接，场景关闭后不保留句柄；不连接用户桌面。"""
    def __init__(self, scene, profile):
        """入参：版本化场景字典和工程模型；world=URDF base_link、障碍物为静态球/盒。

        验证必填字段与刚体基座变换，失败抛统一码，不允许空配置冒充碰撞已检查。
        """
        if version('pybullet')!='3.2.7':raise ContractError(1008,'Collision engine version differs from frozen baseline')
        if not isinstance(scene,dict) or any(k not in scene for k in ('scene_id','revision','T_world_base','clearance_m','query_distance_m','obstacles')):
            raise ContractError(1004,'Missing collision scene')
        import numpy as np
        if not np.array_equal(scene['T_world_base'],profile['T_world_base']):
            raise ContractError(1007,'Scene base transform mismatch')
        self.profile_sha256=hashlib.sha256(json.dumps(profile,sort_keys=True).encode()).hexdigest()
        self.scene=scene;self.clearance=scene['clearance_m'];self.query=scene['query_distance_m']
        if any(type(x) not in (int,float) or not math.isfinite(x) for x in (self.clearance,self.query)) or not 0<=self.clearance<self.query:
            raise ContractError(1001,'Invalid collision clearance')
        self.client=p.connect(p.DIRECT);self.obstacles=[]
        try:
            self.robot=p.loadURDF(str(ROOT/'models/ur5/ur5.urdf'),useFixedBase=True,
                flags=p.URDF_USE_SELF_COLLISION,physicsClientId=self.client)
            infos=[p.getJointInfo(self.robot,i,physicsClientId=self.client) for i in range(p.getNumJoints(self.robot,physicsClientId=self.client))]
            mapping={info[1].decode():info[0] for info in infos}
            self.joints=[mapping[n] for n in profile['urdf_joint_names']]
            self.links=[i for i in range(-1,len(infos)) if p.getCollisionShapeData(self.robot,i,physicsClientId=self.client)]
            parent={i:info[16] for i,info in enumerate(infos)}
            def distance(a,b):
                def ancestors(x):
                    out=[x]
                    while x!=-1:x=parent[x];out.append(x)
                    return out
                aa,bb=ancestors(a),ancestors(b)
                return min(aa.index(x)+bb.index(x) for x in set(aa)&set(bb))
            # URDF近邻装配允许接触；精确列出排除对，不能忽略所有自碰撞。
            self.excluded=[(a,b) for k,a in enumerate(self.links) for b in self.links[k+1:] if distance(a,b)<=1]
            self.pairs=[(a,b) for k,a in enumerate(self.links) for b in self.links[k+1:] if distance(a,b)>1]
            for obstacle in scene['obstacles']:
                if not isinstance(obstacle,dict) or any(k not in obstacle for k in ('id','kind','position_m')):
                    raise ContractError(1004,'Obstacle fields missing')
                pos=numbers(obstacle['position_m'],3)
                if obstacle['kind']=='sphere':
                    radius=obstacle.get('radius_m')
                    if type(radius) not in (int,float) or not math.isfinite(radius) or radius<=0:raise ContractError(1001,'Invalid radius')
                    shape=p.createCollisionShape(p.GEOM_SPHERE,radius=radius,physicsClientId=self.client)
                elif obstacle['kind']=='box':
                    ext=numbers(obstacle.get('half_extents_m'),3)
                    if min(ext)<=0:raise ContractError(1001,'Invalid half extents')
                    shape=p.createCollisionShape(p.GEOM_BOX,halfExtents=ext,physicsClientId=self.client)
                else:raise ContractError(1008,'Unsupported obstacle')
                body=p.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,basePosition=pos,physicsClientId=self.client)
                self.obstacles.append((obstacle['id'],body))
            # 保守运动半径：全部关节原点平移长度之和 + 最大碰撞网格点半径/原点偏置。
            # 任一连杆点到任一关节轴的距离均不超过该上界；含0.01m引擎裕度。
            tree=ET.parse(ROOT/'models/ur5/ur5.urdf').getroot()
            translations=sum(float(np.linalg.norm([float(x) for x in joint.find('origin').get('xyz','0 0 0').split()]))
                for joint in tree.findall('joint') if joint.find('origin') is not None)
            extent=0.0
            for collision in tree.findall('link/collision'):
                mesh=collision.find('geometry/mesh')
                if mesh is None:raise ContractError(1008,'Motion bound requires mesh-only URDF')
                data=(ROOT/'models/ur5'/mesh.get('filename')).read_bytes()
                count=struct.unpack_from('<I',data,80)[0]
                if len(data)!=84+50*count:raise ContractError(1008,'Motion bound requires binary STL')
                scale=np.array([float(x) for x in mesh.get('scale','1 1 1').split()])
                radius=max(float(np.linalg.norm(np.array(struct.unpack_from('<3f',data,84+50*i+12+12*j))*scale))
                           for i in range(count) for j in range(3))
                origin=collision.find('origin')
                offset=0.0 if origin is None else float(np.linalg.norm([float(x) for x in origin.get('xyz','0 0 0').split()]))
                extent=max(extent,radius+offset)
            self.motion_radius_bound_m=translations+extent+0.01
            self.calls=0
        except Exception:
            self.close();raise

    def close(self):
        """释放当前DIRECT连接；返回None，可重复调用。"""
        if p.isConnected(self.client):p.disconnect(self.client)

    def check(self,q):
        """q为6关节rad。返回最小距离下界、最近对象和通过状态；无近邻返回query上界。

        检查所有具碰撞几何的连杆对静态物体、指定非邻接自碰撞对；距离<=clearance拒绝。
        此方法是离散配置检查，不宣称连续无碰撞证明。
        """
        q=numbers(q,6)
        for joint,value in zip(self.joints,q):p.resetJointState(self.robot,joint,value,physicsClientId=self.client)
        p.performCollisionDetection(physicsClientId=self.client)
        minimum=self.query;nearest=None
        for name,body in self.obstacles:
            points=p.getClosestPoints(self.robot,body,self.query,physicsClientId=self.client)
            for point in points:
                if point[8]<minimum:minimum=point[8];nearest={'obstacle':name,'robot_link':point[3]}
        for a,b in self.pairs:
            for point in p.getClosestPoints(self.robot,self.robot,self.query,linkIndexA=a,linkIndexB=b,physicsClientId=self.client):
                if point[8]<minimum:minimum=point[8];nearest={'self_link_pair':[a,b]}
        self.calls+=1
        return {'passed':minimum>self.clearance,'minimum_distance_lower_bound_m':minimum,'nearest':nearest}

    def metadata(self):
        """返回场景/URDF/碰撞网格哈希、检查协议和排除对，用于复现。"""
        if not p.isConnected(self.client):raise ContractError(1004,'Collision scene is closed')
        return {'scene_id':self.scene['scene_id'],'revision':self.scene['revision'],
            'model_profile_sha256':self.profile_sha256,
            'scene_sha256':hashlib.sha256(json.dumps(self.scene,sort_keys=True).encode()).hexdigest(),
            'geometry_sha256':{str(x.relative_to(ROOT)):hashlib.sha256(x.read_bytes()).hexdigest()
                for x in [ROOT/'models/ur5/ur5.urdf']+sorted((ROOT/'models/ur5/meshes').glob('*.stl'))},
            'checker':'PyBullet 3.2.7 DIRECT closestPoints, URDF collision meshes',
            'clearance_m':self.clearance,'query_distance_m':self.query,
            'excluded_near_assembly_pairs':self.excluded,'checked_self_pairs':self.pairs,
            'calls':self.calls,'motion_radius_bound_m':self.motion_radius_bound_m,
            'motion_bound_source':'sum URDF joint translations + max scaled binary-STL vertex/collision offset + 0.01m padding'}
