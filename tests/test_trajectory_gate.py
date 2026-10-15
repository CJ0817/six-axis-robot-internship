"""Certification/gate unit regressions with real C planning and FK.

The deterministic collision double exercises failure propagation and continuous
proof rejection. It provides NO evidence of physical whole-arm/self clearance;
that is covered separately by verify_trajectory_certification.py on PyBullet.
"""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
if importlib.util.find_spec('pybullet') is None:
    # Import-only stub; no simulation engine function is called by these tests.
    with patch.dict(sys.modules,{'pybullet':types.ModuleType('pybullet')}):
        import planning.collision
        collision_module=sys.modules['planning.collision']
else:
    import planning.collision
    collision_module=sys.modules['planning.collision']
from adapters.c_kinematics import ContractError
from planning.cartesian import CartesianPlanner
from planning.execution import execute_trajectory
from planning.joint import JointPlanner


class CollisionDouble:
    """Analytic test clearance; not a robot collision model."""
    def __init__(self,profile):
        self.profile_sha256=hashlib.sha256(json.dumps(profile,sort_keys=True).encode()).hexdigest()
        self.motion_radius_bound_m=1.;self.clearance=.003;self.distance=10.
        self.revision='unit';self.reject=False

    def check(self,q):
        return dict(passed=not self.reject,minimum_distance_lower_bound_m=self.distance,
                    nearest={'test_double':True})

    def metadata(self):
        return dict(revision=self.revision,profile_sha256=self.profile_sha256)


class TrajectoryGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin=Path(os.environ.get('ROBOT_KINEMATICS_LIBRARY',ROOT/'build/librobot_contract.so'))
        cls.lib=Path(os.environ.get('ROBOT_PLANNING_LIBRARY',ROOT/'build/librobot_planning.so'))
        if not cls.kin.exists() or not cls.lib.exists():
            raise RuntimeError('Build real C FK/planning libraries before running these tests')

    def setUp(self):
        modules=patch.dict(sys.modules,{'planning.collision':collision_module});modules.start()
        self.addCleanup(modules.stop)
        self.patcher=patch.object(collision_module,'StaticScene',CollisionDouble);self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.profile=json.loads((ROOT/'models/ur5/kinematics.json').read_text())
        self.scene=CollisionDouble(self.profile);self.joint=JointPlanner(self.lib)
        self.q0=[.3,-1.2,1.1,-1.1,1.,.2];self.q1=[.36,-1.17,1.06,-1.08,1.01,.2]
        self.options=dict(method='quintic',sample_period_s=.02,duration_s=8.,duration_policy='strict',max_duration_s=30.)

    def plan(self,options=None,planner=None,scene='default',**kwargs):
        return (planner or self.joint).plan_joint(self.q0,self.q1,options or self.options,
            scene=self.scene if scene=='default' else scene,kinematics_library=self.kin,**kwargs)

    def reject(self,result,code):
        self.assertEqual(result['code'],code,result['message'])
        self.assertIsNone(result['data']);self.assertIsNone(result.get('next_state'))
        self.assertTrue(result['details']['failure_log'])
        calls=[];self.assertNotEqual(execute_trajectory(result,self.scene,calls.append)['code'],0)
        self.assertFalse(calls)

    def test_candidate_rejected_and_flags_cannot_forge(self):
        candidate=self.joint.plan_candidate(self.q0,self.q1,self.options)
        self.assertEqual(candidate['data']['trajectory_status'],'candidate')
        self.assertFalse(candidate['data']['executable'])
        self.reject(execute_trajectory(candidate,self.scene,lambda _:self.fail()),3001)
        candidate['data'].update(trajectory_status='certified',executable=True)
        self.reject(execute_trajectory(candidate,self.scene,lambda _:self.fail()),3001)

    def test_all_methods_preserve_original_polynomials_and_dispatch(self):
        for method in ('cubic','quintic','trapezoidal'):
            with self.subTest(method=method):
                options=dict(self.options,method=method)
                result=self.plan(options);self.assertEqual(result['code'],0,result['message'])
                candidate=self.joint.plan_candidate(self.q0,self.q1,options)
                for key in ('coefficients_rad','segment_times_s'):
                    self.assertEqual(result['data'][key],candidate['data'][key])
                calls=[];gate=execute_trajectory(result,self.scene,calls.append)
                self.assertEqual(gate['code'],0,gate['message']);self.assertEqual(len(calls),1)
                calls[0]['q_rad'][0][0]+=1
                self.assertNotEqual(calls[0]['q_rad'],result['data']['q_rad'])

    def test_tampering_each_executable_field_rejected(self):
        result=self.plan();self.assertEqual(result['code'],0,result['message'])
        for field in ('q_rad','qd_rad_s','qdd_rad_s2','coefficients_rad','segment_times_s','time_s','sample_period_s','actual_duration_s'):
            with self.subTest(field=field):
                altered=copy.deepcopy(result)
                if field=='coefficients_rad':altered['data'][field][0][0][0]+=.01
                elif field in ('segment_times_s','time_s'):altered['data'][field][-1]+=.1
                elif field in ('sample_period_s','actual_duration_s'):altered['data'][field]+=.1
                else:altered['data'][field][0][0]+=.01
                self.reject(execute_trajectory(altered,self.scene,lambda _:self.fail()),3001)

    def test_joint_rate_and_position_codes(self):
        self.reject(self.plan(dict(self.options,duration_s=.1)),3002)
        self.q1[0]=self.profile['q_max_rad'][0]+.1
        self.reject(self.plan(),3002)

    def test_all_four_tcp_limits(self):
        source=json.loads((ROOT/'config/planning-limits.v1.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            for key in source['tcp_limits']:
                with self.subTest(key=key):
                    config=copy.deepcopy(source);config['tcp_limits'][key]=1e-5
                    path=Path(directory)/'limits.json';path.write_text(json.dumps(config))
                    self.reject(self.plan(planner=JointPlanner(self.lib,path)),3002)

    def test_stretch_retries_complete_certification(self):
        config=json.loads((ROOT/'config/planning-limits.v1.json').read_text())
        config['tcp_limits']['linear_velocity_m_s']=.04
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'limits.json';path.write_text(json.dumps(config))
            joint=JointPlanner(self.lib,path)
            self.reject(self.plan(planner=joint),3002)
            result=self.plan(dict(self.options,duration_policy='stretch'),planner=joint)
            self.assertEqual(result['code'],0,result['message'])
            attempts=result['details']['attempts']
            self.assertTrue(any(a['stage']=='certification' and a['code']==3002 for a in attempts))
            self.assertEqual(attempts[-1]['code'],0)
            self.assertGreater(result['data']['actual_duration_s'],8.)
            self.assertEqual(execute_trajectory(result,self.scene,lambda _:None)['code'],0)
            path.write_text('{}')
            self.reject(execute_trajectory(result,self.scene,lambda _:self.fail()),3001)
            path.unlink()
            self.reject(execute_trajectory(result,self.scene,lambda _:self.fail()),1004)

    def test_stretch_budget_still_fails_closed(self):
        self.reject(self.plan(dict(self.options,duration_policy='stretch',max_duration_s=.1)),3001)

    def test_missing_configuration_and_scene(self):
        self.reject(self.plan(scene=None),1004)
        with tempfile.TemporaryDirectory() as directory:
            missing=Path(directory)/'missing.json'
            self.reject(self.plan(planner=JointPlanner(self.lib,missing)),1004)
            self.reject(self.plan(certification_config=missing),1004)

    def test_collision_failure_keeps_validation_node(self):
        self.scene.reject=True;result=self.plan();self.reject(result,3003)
        records=result['details']['attempts'][-1]['failure_log']
        self.assertTrue(any(r['stage']=='collision' and 'time_s' in r for r in records))

    def test_continuous_clearance_failure(self):
        self.scene.distance=.003001
        self.reject(self.plan(),3001)

    def test_scene_changed_rejected(self):
        result=self.plan();self.assertEqual(result['code'],0)
        self.scene.revision='changed'
        self.reject(execute_trajectory(result,self.scene,lambda _:self.fail()),3001)

    def test_interior_extremum_position_rejected(self):
        verifier=CartesianPlanner(self.kin,self.lib);verifier.setup()
        # Safe endpoint values, interior peak exceeds the position range.
        co=[[[0.,30.,-30.,0.,0.,0.]]+[[x,0,0,0,0,0] for x in self.q0[1:]]]
        with self.assertRaises(ContractError) as error:
            verifier.certify_polynomial(co,[0.,30.],self.scene)
        self.assertEqual(error.exception.code,3002)


if __name__=='__main__':unittest.main()
