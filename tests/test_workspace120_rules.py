import copy
import json
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from prepare_workspace120 import EXPECTED_RULES,compare

class ExpectedRules(unittest.TestCase):
    def test_saved_rules_and_unrun_status(self):
        data=json.loads((ROOT/'tests/ik/workspace120.json').read_text())
        compare(data['expected_rules'],EXPECTED_RULES)
        self.assertEqual(data['sample_count'],len(data['samples']))
        self.assertEqual(data['sample_count'],120)
        self.assertTrue(data['expected_rules']['geometric_reachable'])
        self.assertEqual(data['expected_rules']['fk']['expected_code'],0)
        self.assertEqual(data['expected_rules']['ik']['execution_status'],'not_run')
        self.assertIsNone(data['expected_rules']['ik']['solver_success_rate'])
    def test_missing_or_weakened_rules_rejected(self):
        missing=copy.deepcopy(EXPECTED_RULES);del missing['ik']['singular_exception']
        with self.assertRaises(AssertionError):compare(missing,EXPECTED_RULES)
        weaker=copy.deepcopy(EXPECTED_RULES);weaker['ik']['regular_target']['every_retained_candidate_fk_position_error_m_max']=1e-3
        with self.assertRaises(AssertionError):compare(weaker,EXPECTED_RULES)
if __name__=='__main__':unittest.main()
