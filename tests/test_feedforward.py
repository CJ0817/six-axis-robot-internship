"""Independent C recurrence, total saturation and metric definition regressions."""
import ctypes as C
import math
import os
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from control.pid import PID,FeedforwardPID,FeedforwardSample


class FeedforwardContracts(unittest.TestCase):
    def setUp(self):
        """Use a real compiled library and explicit test limits."""
        self.lib=Path(os.environ.get('ROBOT_CONTROL_LIBRARY',ROOT/'build/librobot_control.so'))
        self.config=dict(dt_s=.01,kp=2.,ki=.5,kd=.01,integral_min=-3.,integral_max=3.,output_min=-100.,output_max=100.)

    def test_independent_positional_equivalence(self):
        p=FeedforwardPID(self.lib,self.config,.4,1.5);integral=old=0.
        for k in range(200):
            error=.05*math.sin(.13*k);v=.2*math.cos(.07*k);a=-.4*math.sin(.07*k)
            integral+=.5*.01*error
            expected=2*error+integral+.01/.01*(error-old)+.4*v+1.5*a
            result=p.step(error,0,v,a);self.assertEqual(result['code'],0)
            self.assertAlmostEqual(result['data']['output'],expected,places=11);old=error

    def test_zero_feedforward_is_identical_to_pid(self):
        p=PID(self.lib,self.config);f=FeedforwardPID(self.lib,self.config,0.,0.)
        for k in range(200):
            r=.1*math.sin(k);a=p.step(r,0);b=f.step(r,0,10.,10.)
            self.assertEqual(a['data']['output'],b['data']['output'])
            self.assertEqual(bytes(p.state),bytes(f.state))

    def test_no_feedforward_accumulation(self):
        f=FeedforwardPID(self.lib,self.config,.4,1.5)
        for _ in range(100):self.assertAlmostEqual(f.step(0,0,1,1)['data']['output'],1.9)
        self.assertAlmostEqual(f.step(0,0,0,0)['data']['output'],0.)

    def test_combined_saturation_and_antiwindup_both_signs(self):
        for sign in (1,-1):
            f=FeedforwardPID(self.lib,dict(self.config,kp=.2,ki=1.,kd=0.,output_min=-1.,output_max=1.),1.,1.)
            for _ in range(100):r=f.step(sign,0,sign,sign)['data']
            self.assertEqual(r['output'],sign);self.assertEqual(r['integral'],0.)
            self.assertTrue(r['output_limited']);self.assertTrue(r['integral_limited'])
            self.assertAlmostEqual(f.step(0,0,0,0)['data']['output'],0.)

    def test_opposing_feedforward_does_not_prematurely_clip_feedback(self):
        f=FeedforwardPID(self.lib,dict(self.config,kp=2.,ki=0.,kd=0.,output_min=-1.,output_max=1.),1.,0.)
        r=f.step(1,0,-1.5,0)['data']
        self.assertAlmostEqual(r['feedback_raw'],2.);self.assertAlmostEqual(r['output'],.5)
        self.assertFalse(r['output_limited'])

    def test_reset_repeats_sample(self):
        f=FeedforwardPID(self.lib,self.config,.4,1.5);first=f.step(.1,0,.2,.3)
        f.step(.4,.2,.3,.4);self.assertEqual(f.reset(),0)
        self.assertEqual(f.step(.1,0,.2,.3),first)

    def test_bad_inputs_do_not_advance(self):
        f=FeedforwardPID(self.lib,self.config,.4,1.5)
        for values in ((math.nan,0,0,0),(0,math.inf,0,0),(0,0,math.nan,0),(0,0,0,math.inf),(True,0,0,0),(0,0,'1',0),(1e308,-1e308,0,0)):
            before=bytes(f.state);self.assertEqual(f.step(*values),dict(code=1001,data=None));self.assertEqual(bytes(f.state),before)

    def test_direct_c_failure_preserves_both_objects(self):
        f=FeedforwardPID(self.lib,self.config,.4,1.5);out=FeedforwardSample();C.memset(C.byref(out),0x5a,C.sizeof(out))
        before=bytes(f.state);sentinel=bytes(out)
        for kv,ka,v,a in ((-1,1,0,0),(1,math.nan,0,0),(1e308,1,1e308,0),(1,1,math.inf,0)):
            self.assertEqual(f.lib.robot_pid_step_ff(C.byref(f.state),0,0,v,a,kv,ka,C.byref(out)),1001)
            self.assertEqual(bytes(f.state),before);self.assertEqual(bytes(out),sentinel)
        self.assertEqual(f.lib.robot_pid_step_ff(None,0,0,0,0,1,1,C.byref(out)),1001)
        self.assertEqual(f.lib.robot_pid_step_ff(C.byref(f.state),0,0,0,0,1,1,None),1001)
        f.state.initialized=0;before=bytes(f.state)
        self.assertEqual(f.lib.robot_pid_step_ff(C.byref(f.state),0,0,0,0,1,1,C.byref(out)),4001)
        self.assertEqual(bytes(f.state),before);self.assertEqual(bytes(out),sentinel)

    def test_bad_gain_binding(self):
        for kv in (-1,True,math.nan,math.inf,'1'):
            with self.assertRaises(ValueError):FeedforwardPID(self.lib,self.config,kv,1.)


class MetricContracts(unittest.TestCase):
    def test_settling_requires_persistent_band_and_dwell(self):
        from verify_pid_feedforward import settled
        rows=[dict(time_s=k*.1,error_rad=.1 if k in (2,7) else 0.) for k in range(20)]
        self.assertAlmostEqual(settled(rows,.5,.01,.5),.3)
        self.assertIsNone(settled(rows[:11],.5,.01,.5))

    def test_analytic_reference_derivatives_and_endpoints(self):
        import json
        from verify_pid_feedforward import reference
        c=json.loads((ROOT/'config/pid-feedforward.v1.json').read_text())
        for case in ('move','sine'):
            for t in (.8,1.3,2.1):
                h=1e-5;l=reference(case,t-h,c);m=reference(case,t,c);r=reference(case,t+h,c)
                self.assertAlmostEqual((r[0]-l[0])/(2*h),m[1],places=7)
                self.assertAlmostEqual((r[1]-l[1])/(2*h),m[2],places=7)
            stop=c['trajectory']['start_s']+c['trajectory']['move_duration_s' if case=='move' else 'sine_duration_s']
            self.assertEqual(reference(case,0.,c)[1:],(0.,0.))
            self.assertEqual(reference(case,stop,c)[1:],(0.,0.))

    def test_hold_overshoot_is_not_a_step_percentage(self):
        import json
        from verify_pid_feedforward import metrics
        c=json.loads((ROOT/'config/pid-feedforward.v1.json').read_text())
        rows=[dict(time_s=k*.01,error_rad=.01,output_limited=0,integral_limited=0) for k in range(1200)]
        m=metrics(rows,'hold_sustained',c)
        self.assertIsNone(m['overshoot_percent']);self.assertIsNone(m['settling_time_s'])
        self.assertAlmostEqual(m['tracking_rmse_rad'],.01)
        self.assertAlmostEqual(m['steady_state_bias_rad'],.01)


if __name__=='__main__':unittest.main()
