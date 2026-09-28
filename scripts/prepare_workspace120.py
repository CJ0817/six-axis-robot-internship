"""Fixed-seed, workspace-stratified reachable fixture. Default verifies; never overwrites."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import sys
ROOT=Path(__file__).resolve().parents[1]
SEED=20260928
EXPECTED_RULES = {'scope': 'nominal UR5 geometry and joint limits; not collision, dynamics or physical-robot safety',
 'geometric_reachable': True,
 'fk': {'expected_code': 0,
        'output_shape': [4, 4],
        'finite_output_required': True,
        'comparison_target': 'sample.target_T_base_tool',
        'position_error_m_max': 1e-05,
        'orientation_error_rad_max': 0.0001},
 'reference_data_validation': {'method': 'explicit project-parameter Robotics Toolbox FK',
                               'matrix_max_abs_error_max': 1e-12},
 'ik': {'execution_status': 'not_run',
        'solver_success_rate': None,
        'applies_when': 'valid finite model/target/options and a seed inside the configured joint limits; '
                        'analytic_ur5 with the frozen ABI semantics',
        'seed_policy': 'test harness must declare seed and branch_policy before execution; q_rad is a '
                       'reachability witness, not a hidden solver answer',
        'regular_target': {'expected_code': 0,
                           'minimum_valid_solution_count': 1,
                           'joint_limits_required': True,
                           'finite_output_required': True,
                           'every_retained_candidate_fk_position_error_m_max': 1e-05,
                           'every_retained_candidate_fk_orientation_error_rad_max': 0.0001,
                           'must_recover_witness_q': False},
        'singular_exception': {'condition': 'analytic wrist decomposition detects hypot(u,v)<=1e-12 on a '
                                            'shoulder branch; q5 near zero alone does not automatically '
                                            'exempt the sample',
                               'all_policy': '2003 for an unresolved continuous solution family; never claim '
                                             'finite candidates exhaust that family',
                               'nearest_seed_policy': '0 is allowed for a verified matching-seed '
                                                      'representative; otherwise unresolved singular '
                                                      'selection returns 2003',
                               'allowed_exception_code': 2003,
                               'classification': 'record as singular exception, not code-0 solver success; '
                                                 'keep the original 120 denominator and additionally report '
                                                 'the subgroup',
                               'geometric_reachable_remains': True},
        'other_failures': {'codes': [1001, 1004, 1005, 1008, 2001, 2002, 9000],
                           'classification': 'must be investigated and counted as failures, not excused by '
                                             'geometric reachability; 2001 contradicts the nominal-model '
                                             'witness',
                           'timeout_or_nonconvergence_is_success': False},
        'execution_plan_required': ['q_seed_rad or deterministic seed-generation rule',
                                    'branch_policy',
                                    'joint_margin_rad',
                                    'position_tol_m',
                                    'orientation_tol_rad',
                                    'timeout_s'],
        'constraint_change': 'a nonzero joint margin or modified model can exclude a witness; document '
                             'separately rather than silently changing this fixture expectation'}}
I=[[1.,0,0,0],[0,1.,0,0],[0,0,1.,0],[0,0,0,1.]]

def multiply(a,b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def forward(m,q):
    T=m['T_base_dh0']
    for i in range(6):
        t=m['joint_sign'][i]*q[i]+m['theta_offset_rad'][i];c,s=math.cos(t),math.sin(t)
        ca,sa=math.cos(m['alpha_rad'][i]),math.sin(m['alpha_rad'][i]);a,d=m['a_m'][i],m['d_m'][i]
        T=multiply(T,[[c,-s*ca,s*sa,a*c],[s,c*ca,-c*sa,a*s],[0,sa,ca,d],[0,0,0,1.]])
    return multiply(multiply(T,m['T_dh6_flange']),m['T_flange_tool'])

def region(m,T):
    x,y,z=T[0][3],T[1][3],T[2][3]-m['d_m'][0]
    r=math.sqrt(x*x+y*y+z*z)
    shell='near' if r<.35 else 'middle' if r<.65 else 'outer'
    return ''.join('+' if v>=0 else '-' for v in (x,y,z))+'/'+shell,r

def generate(m):
    rng=random.Random(SEED);counts={a+b+c+'/'+r:0 for a in '-+' for b in '-+' for c in '-+' for r in ['near','middle','outer']}
    rows=[];attempts=0
    while len(rows)<120:
        attempts+=1
        if attempts>100000:raise RuntimeError('Coverage quota not reached')
        q=[rng.uniform(lo+.05,hi-.05) for lo,hi in zip(m['q_min_rad'],m['q_max_rad'])]
        T=forward(m,q);cell,r=region(m,T)
        if counts[cell]==5:continue
        counts[cell]+=1
        rows.append(dict(target_id='workspace_%03d'%len(rows),region=cell,radius_from_shoulder_m=r,q_rad=q,target_T_base_tool=T))
    normal=dict(schema_version=1,purpose='reachable_input_fixture_not_IK_results',expected_rules=copy.deepcopy(EXPECTED_RULES),random_seed=SEED,rng='Python random.Random MT19937; uniform finite joint interval',model_sha256=hashlib.sha256((ROOT/'models/ur5/kinematics.json').read_bytes()).hexdigest(),base_frame='base',tool_frame='tool0',length_unit='m',angle_unit='rad',joint_order=m['joint_names'],generation=dict(method='independent standard-DH FK; accept first five FK samples in each spatial cell; no IK success filtering',joint_margin_rad=.05,attempt_count=attempts,workspace_origin_base_m=[0,0,m['d_m'][0]],shell_bounds_m=[0,.35,.65,None],octant_boundary='zero belongs to positive side',quota_per_cell=5),coverage=counts,sample_count=120,samples=rows)
    base=[.2,-.6,.8,-.5,.4,-.2];bounds=[]
    for i in range(6):
        for side,key in [('lower','q_min_rad'),('upper','q_max_rad')]:
            q=base.copy();q[i]=m[key][i];bounds.append(dict(target_id='boundary_J%d_%s'%(i+1,side),q_rad=q,target_T_base_tool=forward(m,q),expected_fk_code=0))
    for name,q in [('zero',[0]*6),('wrist_pi',base[:4]+[math.pi,base[5]]),('elbow_straight',base[:2]+[0]+base[3:]),('elbow_fold',base[:2]+[math.pi]+base[3:])]:
        bounds.append(dict(target_id='boundary_'+name,q_rad=q,target_T_base_tool=forward(m,q),expected_fk_code=0))
    exceptions=[]
    def add(name,operation,payload,expected):exceptions.append(dict(target_id='exception_'+name,operation=operation,payload=payload,expected_code=expected,execution_status='not_run'))
    add('q_length5','forward',dict(q_rad=base[:5]),1001);add('q_length7','forward',dict(q_rad=base+[0]),1001)
    for token in ['NaN','+Inf']:
        q=base.copy();q[0]=token;add('q_'+token,'forward',dict(q_rad=q),1001)
    for name,v in [('below_limit',m['q_min_rad'][0]-1e-6),('above_limit',m['q_max_rad'][0]+1e-6)]:
        q=base.copy();q[0]=v;add(name,'forward',dict(q_rad=q),1005)
    for name,index,value in [('bottom_row',(3,3),0),('nonrotation',(0,0),2),('target_nan',(0,0),'NaN')]:
        T=copy.deepcopy(I);T[index[0]][index[1]]=value;add(name,'inverse',dict(target_T_base_tool=T,q_seed_rad=base),1001)
    T=copy.deepcopy(I);T[0][3]=10;add('outside_reach','inverse',dict(target_T_base_tool=T,q_seed_rad=base),2001)
    T=copy.deepcopy(I);T[0][3]=m['d_m'][3]-.001;T[2][3]=m['d_m'][5];add('inside_shoulder','inverse',dict(target_T_base_tool=T,q_seed_rad=base),2001)
    special=dict(schema_version=1,purpose='separate_boundary_exception_inputs_not_solver_results',model_sha256=normal['model_sha256'],excluded_from_main_120=True,inverse_defaults=dict(method='analytic_ur5',branch_policy='all',position_tol_m=1e-5,orientation_tol_rad=1e-4,joint_margin_rad=0,timeout_s=.2),boundary_count=len(bounds),boundaries=bounds,exception_count=len(exceptions),exceptions=exceptions,nonfinite_encoding='Only NaN and +Inf string tokens in payload numeric positions decode to IEEE nonfinite values; JSON itself contains no NaN/Infinity literals. Null pointers need native ABI tests, not this fixture.',note='Boundary witnesses prove FK poses; do not require IK all to succeed at exact singularities; exception expected_code is specification, not execution evidence.')
    return normal,special

def compare(a,b,path='root'):
    if type(a)!=type(b):raise AssertionError(path+' type mismatch')
    if isinstance(a,dict):
        assert a.keys()==b.keys(),path+' keys'
        for k in a:compare(a[k],b[k],path+'.'+k)
    elif isinstance(a,list):
        assert len(a)==len(b),path+' length'
        for i,(x,y) in enumerate(zip(a,b)):compare(x,y,path+'[%d]'%i)
    elif isinstance(a,float):assert math.isfinite(a) and math.isfinite(b) and abs(a-b)<=1e-12,path+' float mismatch'
    else:assert a==b,path+' mismatch'

def main():
    p=argparse.ArgumentParser();p.add_argument('--write-fixture',action='store_true');p.add_argument('--output',type=Path,default=ROOT/'results/workspace120');p.add_argument('--fixture-dir',type=Path,default=ROOT/'tests/ik');args=p.parse_args()
    model=json.loads((ROOT/'models/ur5/kinematics.json').read_text());expected=generate(model)
    files=['workspace120.json','workspace120-special.json']
    if args.write_fixture:
        args.fixture_dir.mkdir(parents=True,exist_ok=True)
        for name,data in zip(files,expected):(args.fixture_dir/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    # Independent RTB check after regeneration equality. No C IK is run here.
    import numpy as np
    import roboticstoolbox as rtb
    from spatialmath import SE3
    report=dict(status='failed',seed=SEED,main_count=120,boundary_count=16,exception_count=11,checks=[],ik_execution_status='not_run',solver_success_rate=None)
    try:
        actual=[]
        for name,data in zip(files,expected):
            loaded=json.loads((args.fixture_dir/name).read_text());compare(loaded,data);actual.append(loaded)
        robot=rtb.DHRobot([rtb.RevoluteDH(a=model['a_m'][i],d=model['d_m'][i],alpha=model['alpha_rad'][i],offset=model['theta_offset_rad'][i]) for i in range(6)],base=SE3(np.array(model['T_base_dh0'])),tool=SE3(np.array(model['T_dh6_flange']) @ np.array(model['T_flange_tool'])))
        for row in actual[0]['samples']+actual[1]['boundaries']:
            q=row['q_rad'];assert all(lo<=v<=hi for v,lo,hi in zip(q,model['q_min_rad'],model['q_max_rad']))
            T=robot.fkine(np.array(q)*model['joint_sign']).A;error=float(np.max(np.abs(T-np.array(row['target_T_base_tool']))))
            report['checks'].append(dict(target_id=row['target_id'],rtb_matrix_max_abs_error=error,passed=error<=1e-12))
        assert all(r['passed'] for r in report['checks'])
        assert len({tuple(r['q_rad']) for r in expected[0]['samples']})==120
        poses=[np.array(r['target_T_base_tool']) for r in actual[0]['samples']]
        assert all(float(np.max(np.abs(a-b)))>1e-9 for i,a in enumerate(poses) for b in poses[:i]),'Duplicate target pose'
        xyz=np.array([T[:3,3] for T in poses])
        report.update(generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),position_min_base_m=xyz.min(axis=0).tolist(),position_max_base_m=xyz.max(axis=0).tolist(),status='passed',coverage=expected[0]['coverage'],attempt_count=expected[0]['generation']['attempt_count'],numpy=np.__version__,roboticstoolbox=rtb.__version__,python=sys.version,fixture_sha256={f:hashlib.sha256((args.fixture_dir/f).read_bytes()).hexdigest() for f in files})
    except (AssertionError,OSError,ValueError) as exc:report['reason']=str(exc)
    args.output.mkdir(parents=True,exist_ok=True);(args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='checks'},indent=2));return 0 if report['status']=='passed' else 1
if __name__=='__main__':raise SystemExit(main())
