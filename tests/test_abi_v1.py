import ctypes as C
import json
from pathlib import Path
import re
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import FKModel,IKOptionsV1,IKResultV1

class FrozenABI(unittest.TestCase):
    def test_layout_symbols_codes_and_stub(self):
        abi=json.loads((ROOT/'src/common/abi-v1.json').read_text())
        for name,struct in [('robot_fk_model',FKModel),('robot_ik_options_v1',IKOptionsV1),('robot_ik_result_v1',IKResultV1)]:
            frozen=abi['structures'][name]
            self.assertEqual(C.sizeof(struct),frozen['size']);self.assertEqual(C.alignment(struct),frozen['alignment'])
            for field,offset in frozen['offsets'].items():self.assertEqual(getattr(struct,field).offset,offset)
        contract=json.loads((ROOT/'src/common/contract.json').read_text())
        codes={e['name']:e['code'] for e in contract['errors']}
        header={n:int(c) for n,c in re.findall(r'ROBOT_(\w+)\s*=\s*(\d+)',(ROOT/'include/robot_errors.h').read_text())}
        self.assertEqual(codes,abi['errors']);self.assertEqual(header,codes)
        lib=C.CDLL(str(ROOT/'build/librobot_contract.so'))
        for symbol in abi['exported_symbols']:self.assertTrue(hasattr(lib,symbol))
        lib.robot_abi_version.argtypes=[];lib.robot_abi_version.restype=C.c_uint32
        self.assertEqual(lib.robot_abi_version(),65536)
        fn=lib.robot_inverse_v1;fn.restype=C.c_int
        fn.argtypes=[C.POINTER(FKModel),C.POINTER(C.c_double),C.c_size_t,C.POINTER(C.c_double),C.c_size_t,C.POINTER(IKOptionsV1),C.POINTER(IKResultV1)]
        out=IKResultV1();C.memset(C.byref(out),0x5A,C.sizeof(out));before=bytes(out)
        self.assertEqual(fn(None,None,16,None,6,None,C.byref(out)),1008)
        self.assertEqual(bytes(out),before)
if __name__=='__main__':unittest.main()
