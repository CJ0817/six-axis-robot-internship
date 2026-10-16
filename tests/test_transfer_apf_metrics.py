"""Guard failed-attempt accounting used for business-value claims."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from verify_transfer_apf import stats,summarize


class BusinessAccounting(unittest.TestCase):
    def test_failure_is_in_rate_and_cost_denominators(self):
        good=dict(success=True,failure=None,serial_attempt_s=20.,planning_wall_s=2.,
            simulation_motion_s=18.,planned_joint_path_rad=1.,planned_tcp_path_m=.2,
            intermediate_rest_knots=2,placement_error_m=.001,retention_error_m=.001,
            return_error_rad=.0001,tracking_rmse_rad=.0002,minimum_actual_clearance_m=.01,
            maximum_attachment_error_m=.00001)
        bad=dict(success=False,failure=dict(code=3001),serial_attempt_s=5.,planning_wall_s=5.)
        result=summarize([good,bad])
        self.assertEqual((result['attempts'],result['successes'],result['failures']),(2,1,1))
        self.assertEqual(result['success_rate'],.5)
        self.assertEqual(result['attempt_time_s'],25.)
        self.assertEqual(result['observed_attempt_goodput_per_h'],144.)
        self.assertEqual(result['all_attempt_planning_wall_s']['n'],2)
        self.assertEqual(result['successful_only']['simulation_motion_s']['n'],1)

    def test_no_success_has_null_success_metrics(self):
        result=summarize([dict(success=False,failure=dict(code=3003),
            serial_attempt_s=4.,planning_wall_s=4.)])
        self.assertEqual(result['successes'],0)
        self.assertEqual(result['observed_attempt_goodput_per_h'],0.)
        self.assertTrue(all(value is None for value in result['successful_only'].values()))
        self.assertEqual(result['failure_codes'],[3003])

    def test_quantile_keeps_large_sample(self):
        result=stats([1.,2.,100.])
        self.assertEqual(result['n'],3)
        self.assertEqual(result['maximum'],100.)
        self.assertAlmostEqual(result['p95'],90.2)


if __name__=='__main__':unittest.main()
