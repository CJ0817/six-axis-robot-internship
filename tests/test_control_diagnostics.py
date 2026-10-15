"""Synthetic signals check the criterion; physical trials remain separate."""
import math
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from control.diagnostics import oscillation,saturation


class OscillationCriterion(unittest.TestCase):
    def setUp(self):
        self.rule=dict(window_s=4.,minimum_peak_to_peak_rad=.01,hysteresis_rad=.0025,minimum_full_cycles=2,consecutive_windows=2)
    def rows(self,fn,duration=16.):
        return [dict(time_s=k*.01,error_rad=fn(k*.01)) for k in range(round(duration/.01))]
    def test_sustained_signal_triggers_after_two_full_windows(self):
        r=oscillation(self.rows(lambda t:.02*math.sin(2*math.pi*t)),0.,.01,self.rule)
        self.assertTrue(r['triggered']);self.assertAlmostEqual(r['trigger_time_s'],8.)
    def test_hysteresis_noise_is_not_oscillation(self):
        r=oscillation(self.rows(lambda t:.002*math.sin(2*math.pi*15*t)),0.,.01,self.rule)
        self.assertEqual(r['status'],'not_detected')
    def test_small_crossings_with_one_large_spike_do_not_count_cycles(self):
        r=oscillation(self.rows(lambda t:.1 if abs(t-1.)<.005 else .003*math.sin(2*math.pi*t)),0.,.01,self.rule)
        self.assertFalse(r['triggered'])
    def test_one_window_of_ringdown_is_not_persistent(self):
        r=oscillation(self.rows(lambda t:.02*math.sin(2*math.pi*t) if t<4 else 0.),0.,.01,self.rule)
        self.assertFalse(r['triggered'])
    def test_monotone_drift_is_not_oscillation(self):
        self.assertFalse(oscillation(self.rows(lambda t:.01*t),0.,.01,self.rule)['triggered'])
    def test_insufficient_duration_is_explicit(self):
        self.assertEqual(oscillation(self.rows(lambda t:0.,2.),0.,.01,self.rule)['status'],'insufficient_window')
    def test_missing_and_nonfinite_samples_rejected(self):
        r=self.rows(lambda t:0.);r.pop(4)
        with self.assertRaises(ValueError):oscillation(r,0.,.01,self.rule)
        r=self.rows(lambda t:math.nan)
        with self.assertRaises(ValueError):oscillation(r,0.,.01,self.rule)
    def test_saturation_intervals_and_release_time(self):
        c=dict(dt_s=.01,pid=dict(output_min=-1.,output_max=1.,integral_min=-3.,integral_max=3.))
        r=[dict(time_s=i*.01,total_raw=x,output=max(-1.,min(1.,x)),integral=0.,integral_limited=int(abs(x)>1),output_limited=int(abs(x)>1),error_rad=0.) for i,x in enumerate((0.,2.,2.,0.,-2.,0.))]
        d=saturation(r,c)
        self.assertEqual(d['request_exceeded_count'],3);self.assertEqual(len(d['intervals']),2)
        self.assertAlmostEqual(d['final_release_s'],.05);self.assertEqual(d['actual_limit_violation_count'],0)


if __name__=='__main__':unittest.main()
