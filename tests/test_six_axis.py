"""Nonuniform polynomial timing and terminal hold contracts."""
from pathlib import Path
import copy
import json
import tempfile
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from control.six_axis import (TickClock, interpolate, check_cycle, load_execution_config,
                              SixAxisExecutor, snapshot)
from adapters.c_kinematics import ContractError


class ClockAndInterpolation(unittest.TestCase):
    def setUp(self):
        # q=t^2 across unequal intervals [0,.2,.7], normalized local u.
        self.data=dict(segment_times_s=[0.,.2,.7], coefficients_rad=[
            [[0.,0.,.04,0.,0.,0.]]*6,
            [[.04,.2,.25,0.,0.,0.]]*6])

    def test_nonuniform_segments_and_exact_endpoint(self):
        for t in (0.,.1,.2,.4,.7):
            q,v,a=interpolate(self.data,t)
            np.testing.assert_allclose(q,[t*t]*6,atol=1e-14)
            np.testing.assert_allclose(v,[2*t]*6,atol=1e-14)
            np.testing.assert_allclose(a,[2.]*6,atol=1e-14)

    def test_terminal_hold_does_not_extrapolate(self):
        q,v,a=interpolate(self.data,1.)
        np.testing.assert_allclose(q,[.49]*6)
        np.testing.assert_array_equal(v,np.zeros(6))
        np.testing.assert_array_equal(a,np.zeros(6))

    def test_invalid_time_rejected(self):
        for t in (-.01,float('nan'),float('inf')):
            with self.assertRaises(ContractError): interpolate(self.data,t)

    def test_integer_ticks_have_no_accumulating_drift(self):
        clock=TickClock(.01)
        for tick in range(10001):
            self.assertEqual(clock.stamp(),(tick,tick*.01))
            clock.advance()

    def test_invalid_period(self):
        for dt in (0.,-.01,float('nan'),float('inf')):
            with self.assertRaises(ContractError): TickClock(dt)


class CycleAndSceneRecords(unittest.TestCase):
    def setUp(self):
        self.config=load_execution_config()
        self.dt=self.config['dt_s']
        self.data=dict(time_s=[i*self.dt for i in range(4)],
            segment_times_s=[i*self.dt for i in range(4)],
            coefficients_rad=[[[0.]*6 for _ in range(6)] for _ in range(3)],
            sample_period_s=self.dt,certification=dict(sample_period_s=self.dt),
            ik_nodes=[dict(node_index=i,time_s=i*self.dt) for i in range(1,4)])

    def test_single_source_resolves_all_periods(self):
        self.assertEqual(self.dt,self.config['pid']['dt_s'])
        self.assertEqual(self.dt,json.loads((ROOT/'config/cartesian-planning.v1.json').read_text())['sample_period_s'])
        check_cycle(self.data,self.dt)

    def test_conflicting_config_is_rejected(self):
        raw=json.loads((ROOT/'config/six-axis-execution.v1.json').read_text())
        for location in ('execution','pid'):
            value=copy.deepcopy(raw)
            (value if location=='execution' else value['pid'])['dt_s']=self.dt/2
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'execution.json';path.write_text(json.dumps(value))
                with self.assertRaises(ContractError) as failure:
                    load_execution_config(path,ROOT/'config/cartesian-planning.v1.json')
                self.assertEqual(failure.exception.code,1006)

    def test_missing_cycle_file_is_rejected(self):
        with self.assertRaises(ContractError) as failure:
            load_execution_config(planning_path=ROOT/'config/absent-cycle.json')
        self.assertEqual(failure.exception.code,1004)

    def test_each_mismatched_cycle_and_grid_is_rejected(self):
        for name in ('metadata','certificate','segments','samples','ik_time','ik_sequence'):
            data=copy.deepcopy(self.data)
            if name=='metadata': data['sample_period_s']/=2
            if name=='certificate': data['certification']['sample_period_s']/=2
            if name=='segments': data['segment_times_s'][2]+=.001
            if name=='samples': data['time_s'][2]+=.001
            if name=='ik_time': data['ik_nodes'][1]['time_s']+=.001
            if name=='ik_sequence': data['ik_nodes'][1]['node_index']=1
            with self.subTest(name=name),self.assertRaises(ContractError) as failure:
                check_cycle(data,self.dt)
            self.assertEqual(failure.exception.code,1006)

    def executor(self):
        class Plant:
            def __init__(self,dt): self.dt_s=dt;self.tick=0;self.commands=0;self.stopped=False
            def read(self): return dict(tick=self.tick,time_s=self.tick*self.dt_s,q_rad=[0.]*6,qd_rad_s=[0.]*6)
            def advance(self,command): self.tick+=1;self.commands+=1
            def stop(self): self.stopped=True
        class Controller:
            def __init__(self,library,config,*args): self.config=SimpleNamespace(dt_s=config['dt_s'])
            def step(self,*args): return dict(code=0,data=dict(output=0.,integral=0.,output_limited=0))
        scene=SimpleNamespace(check=lambda q:dict(passed=True,minimum_distance_lower_bound_m=.1))
        profile=dict(q_min_rad=[-6.]*6,q_max_rad=[6.]*6)
        with patch('control.six_axis.FeedforwardPID',Controller):
            return SixAxisExecutor(None,self.config,Plant(self.dt),scene,profile)

    def test_nan_output_retains_current_raw_and_separate_previous_sample(self):
        executor=self.executor();original=executor.controllers[0].step
        def broken(*args):
            result=original(*args)
            if executor.plant.tick==1: result['data']['output']=float('nan')
            return result
        executor.controllers[0].step=broken
        with self.assertRaises(ContractError): executor._consume(self.data)
        event=executor.events[0]
        self.assertEqual(event['tick'],1)
        self.assertEqual(event['observed_command'][0],{'nonfinite':'NaN'})
        self.assertEqual(event['raw_control_outputs'][0]['data']['output'],{'nonfinite':'NaN'})
        self.assertIsNone(event['validated_command'])
        self.assertEqual(event['previous_sample']['tick'],0)
        self.assertEqual(event['previous_sample']['observed_command'],[0.]*6)
        self.assertEqual(executor.plant.commands,1)
        self.assertTrue(executor.plant.stopped)
        json.dumps(event,allow_nan=False)

    def test_early_clock_failure_has_null_current_feedback_reference_command(self):
        executor=self.executor();clock=executor.clock
        clock.stamp=lambda:(0,0.) # valid first tick, duplicate at next tick
        with self.assertRaises(ContractError): executor._consume(self.data)
        event=executor.events[0]
        self.assertEqual(event['tick'],1)
        self.assertEqual(event['previous_sample']['tick'],0)
        for field in ('observed_feedback','observed_reference','raw_control_outputs',
                      'observed_command','validated_command'):
            self.assertIsNone(event[field],field)

    def test_period_mismatch_prevents_feedback_and_motion(self):
        executor=self.executor()
        data=copy.deepcopy(self.data);data['sample_period_s']/=2
        executor.plant.read=lambda: self.fail('feedback read before cycle rejection')
        with self.assertRaises(ContractError) as failure: executor._consume(data)
        self.assertEqual(failure.exception.code,1006)
        self.assertEqual(executor.plant.commands,0)
        self.assertIsNone(executor.events[0]['observed_command'])
        self.assertIsNone(executor.events[0]['previous_sample'])

    def test_nonfinite_encoding_keeps_missing_distinct(self):
        value=snapshot([None,float('nan'),float('inf'),float('-inf')])
        self.assertEqual(value,[None,{'nonfinite':'NaN'},{'nonfinite':'+Infinity'},{'nonfinite':'-Infinity'}])
        json.dumps(value,allow_nan=False)
