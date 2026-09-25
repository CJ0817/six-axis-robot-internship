"""Print one state-aware IK selection. No connection to a physical robot."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from adapters.c_kinematics import Forward
from adapters.ik_selection import StatefulIKSelector

def main():
    p=argparse.ArgumentParser();p.add_argument('--library',type=Path,default=ROOT/'build/librobot_contract.so');p.add_argument('--target',type=Path,help='JSON 4x4 base-to-tool0 matrix');p.add_argument('--previous',type=float,nargs=6,default=[.3,-1,.9,-.7,.8,.2]);a=p.parse_args()
    profile=json.loads((ROOT/'models/ur5/kinematics.json').read_text())
    if a.target:target=json.loads(a.target.read_text())
    else:
        q=a.previous.copy();q[0]+=.004;fk=Forward(a.library)(profile,q)
        if fk['code']:print(json.dumps(fk));return 1
        target=fk['data']
    result=StatefulIKSelector(a.library).select(profile,target,dict(q_rad=a.previous,step_index=0))
    print(json.dumps(result,indent=2,allow_nan=False));return 0 if result['code']==0 else 1
if __name__=='__main__':raise SystemExit(main())
