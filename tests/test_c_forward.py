"""Check Python boundary metadata and Result failure semantics for the C FK ABI."""
import copy
import json
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import Forward

class ForwardBoundary(unittest.TestCase):
    def test_metadata_and_failed_data(self):
        forward=Forward(ROOT/'build/librobot_contract.so')
        profile=json.loads((ROOT/'models/ur5/kinematics.json').read_text())
        result=forward(profile,[0]*6)
        self.assertEqual(result['code'],0)
        for observed,expected in zip([result['data'][i][3] for i in range(3)],[-.81725,-.19145,-.005491]):
            self.assertAlmostEqual(observed,expected,places=12)
        for field,value,code in [('angle_unit','deg',1002),('joint_names',['J6']*6,1003),
                                 ('tool_frame','flange',1007),('joint_sign',[0]*6,1001)]:
            model=copy.deepcopy(profile);model[field]=value
            with self.subTest(field=field):
                result=forward(model,[0]*6)
                self.assertEqual(result['code'],code)
                self.assertIsNone(result['data'])
        for q in [[0]*5,[float('nan')]*6,[100]*6]:
            result=forward(profile,q)
            self.assertNotEqual(result['code'],0);self.assertIsNone(result['data'])
        self.assertEqual(forward({},[0]*6)['code'],1004)

if __name__=='__main__':unittest.main()
