"""Nonuniform polynomial timing and terminal hold contracts."""
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from control.six_axis import TickClock, interpolate
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
