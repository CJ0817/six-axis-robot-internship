"""Executable derivation check, not the production IK/ABI implementation."""
import hashlib
import json
import math
from pathlib import Path
import platform
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
M = json.loads((ROOT/'models/ur5/kinematics.json').read_text())

def A(i, t):
    a, d, al = M['a_m'][i], M['d_m'][i], M['alpha_rad'][i]
    c, s, ca, sa = math.cos(t), math.sin(t), math.cos(al), math.sin(al)
    return np.array([[c,-s*ca,s*sa,a*c],[s,c*ca,-c*sa,a*s],[0,sa,ca,d],[0,0,0,1.]])

def fk(q):
    T = np.eye(4)
    for i, t in enumerate(q): T = T @ A(i,t)
    return T

def lift(q, seed):
    out = []
    for t, ref, lo, hi in zip(q,seed,M['q_min_rad'],M['q_max_rad']):
        ks = range(math.ceil((lo-t)/(2*math.pi)),math.floor((hi-t)/(2*math.pi))+1)
        values = [t+2*math.pi*k for k in ks]
        if not values: return None
        out.append(min(values,key=lambda v: ((v-ref)**2,v)))
    return np.array(out)

def candidates(T, seed, singular_phi=None):
    p = T[:3,3]-M['d_m'][5]*T[:3,2]
    rho = math.hypot(p[0],p[1]); d4 = M['d_m'][3]
    if rho < d4-1e-12: return []
    b = math.asin(min(1.,d4/rho)); az = math.atan2(p[1],p[0])
    out = []
    for t1 in [az+b,az+math.pi-b]:
        h = np.array([math.sin(t1),-math.cos(t1),0.])
        u,v,c5 = h @ T[:3,:3]
        # atan2(hypot(u,v), c5) avoids acos loss near |c5|=1.
        mag = math.hypot(u,v)
        for w in [1,-1]:
            t5 = math.atan2(w*mag,c5)
            if mag <= 1e-12:
                if singular_phi is None: continue
                t6 = singular_phi
            else: t6 = math.atan2(-w*v,w*u)
            U = np.linalg.inv(A(0,t1)) @ T @ np.linalg.inv(A(5,t6)) @ np.linalg.inv(A(4,t5))
            x,y = U[:2,3]; a2,a3 = M['a_m'][1:3]
            c3 = (x*x+y*y-a2*a2-a3*a3)/(2*a2*a3)
            if abs(c3)>1+1e-12: continue
            for e in [1,-1]:
                t3 = e*math.acos(float(np.clip(c3,-1,1)))
                t2 = math.atan2(y,x)-math.atan2(a3*math.sin(t3),a2+a3*math.cos(t3))
                t4 = math.atan2(U[1,0],U[0,0])-t2-t3
                q = lift([t1,t2,t3,t4,t5,t6],seed)
                if q is not None and not any(np.max(np.abs(q-r))<=1e-9 for r in out): out.append(q)
    return sorted(out,key=lambda q:tuple(q))

def errors(T, q):
    D = fk(q)
    R = D[:3,:3].T @ T[:3,:3]
    skew = np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/2
    return float(np.linalg.norm(D[:3,3]-T[:3,3])),math.atan2(float(np.linalg.norm(skew)),float((np.trace(R)-1)/2))

def stats(v):
    return dict(valid_count=len(v),mean=float(np.mean(v)),rmse=float(np.sqrt(np.mean(np.square(v)))),p95=float(np.quantile(v,.95,method='linear')),maximum=max(v)) if v else dict(valid_count=0,mean=None,rmse=None,p95=None,maximum=None)

def main():
    # This proof harness intentionally supports only the audited nominal profile.
    assert M['model_id'] == 'ur5_cb_generic'
    assert M['a_m'] == [0,-.425,-.39225,0,0,0]
    assert M['d_m'] == [.089159,0,0,.10915,.09465,.0823]
    assert M['joint_sign'] == [1]*6 and M['theta_offset_rad'] == [0]*6
    assert np.allclose(M['alpha_rad'],[math.pi/2,0,0,math.pi/2,-math.pi/2,0],atol=1e-15,rtol=0)
    assert np.array_equal(np.array(M['T_base_dh0']).reshape(4,4),np.eye(4))
    assert np.array_equal(np.array(M['T_dh6_flange']).reshape(4,4) @ np.array(M['T_flange_tool']).reshape(4,4),np.eye(4))
    fixture = json.loads((ROOT/'tests/ik/targets.json').read_text())
    groups = {}; rows = []
    for group, cases in fixture['groups'].items():
        pe, re = [], []; passed = 0
        for case in cases:
            T = np.array(case['target_T_base_tool']); seed = case['q_seed_rad']
            cs = candidates(T,seed)
            residuals = [errors(T,q) for q in cs]
            ok = bool(cs) and all(p<=1e-5 and r<=1e-4 for p,r in residuals)
            passed += ok
            if residuals:
                pe.append(max(p for p,r in residuals)); re.append(max(r for p,r in residuals))
            rows.append(dict(id=case['sample_id'],group=group,candidate_count=len(cs),passed=ok,position_max_m=pe[-1] if residuals else None,orientation_max_rad=re[-1] if residuals else None))
        groups[group] = dict(expected=len(cases),passed=passed,failed=len(cases)-passed,success_rate=passed/len(cases),position=stats(pe),orientation=stats(re))
    special = []
    for t5 in [0.,math.pi]:
        for phi in [-1.,0.,1.]:
            q = np.array([.3,-.8,.9,.4,t5,phi]); T=fk(q)
            cs=candidates(T,q,singular_phi=phi)
            special.append(dict(kind='singular_fixed_phi_witness',theta5=t5,phi=phi,passed=any(max(errors(T,c))<1e-8 for c in cs)))
    # A definitely unreachable shoulder cylinder target.
    T=np.eye(4); T[:3,3]=[0,0,M['d_m'][5]]
    special.append(dict(kind='shoulder_unreachable',passed=not candidates(T,np.zeros(6))))
    q=np.array([.2,-.6,.8,-.5,.4,-.2]); lifted=q.copy();lifted[0]+=2*math.pi
    special.append(dict(kind='two_pi_fk_equivalence',passed=bool(np.max(np.abs(fk(q)-fk(lifted)))<1e-12)))
    report=dict(scope='formula_validation_only_not_C_IK_acceptance',python=platform.python_version(),numpy=np.__version__,model_sha256=hashlib.sha256((ROOT/'models/ur5/kinematics.json').read_bytes()).hexdigest(),fixture_sha256=hashlib.sha256((ROOT/'tests/ik/targets.json').read_bytes()).hexdigest(),groups=groups,special_cases=special,samples=rows)
    report['passed']=all(g['failed']==0 for g in groups.values()) and all(s['passed'] for s in special)
    path=ROOT/'results/ik-derivation/report.json';path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__': raise SystemExit(main())
