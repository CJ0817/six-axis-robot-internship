"""Load the audited UR5 model and call the C forward kinematics function."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import Forward

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so')
    parser.add_argument('--model',type=Path,default=ROOT/'models/ur5/kinematics.json')
    parser.add_argument('--q',nargs=6,type=float,default=[0]*6,metavar='RAD')
    args=parser.parse_args()
    try:
        result=Forward(args.library)(json.loads(args.model.read_text()),args.q)
    except (OSError,AttributeError,ValueError) as exc:
        result={'code':1004,'message':str(exc),'data':None,'details':{'hint':'Build library and check model path'}}
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0 if result['code']==0 else 1

if __name__=='__main__':sys.exit(main())
