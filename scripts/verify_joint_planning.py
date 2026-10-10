"""验证C关节曲线；保存逐例检查、代表CSV/SVG及源码/配置哈希。"""
import argparse
import copy
import csv
import ctypes as C
import hashlib
import json
import math
import random
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from planning.joint import JointPlanner, CPlan
from planning.limits import load_planning_limits
from adapters.c_kinematics import D6


def polynomial(coeff, t, order, duration):
    """独立按归一化局部升幂系数求导；order=0/1/2，除段时长的对应幂。"""
    return sum(coeff[k]*math.prod(range(k-order+1,k+1))*(t/duration)**(k-order)/duration**order
               for k in range(order,len(coeff)))


def verify(data, start, goal):
    """交叉核对端点、采样、连续段极值和左右拼接。返回阈值与实测误差。"""
    ts=data['time_s']; seg=data['segment_times_s']; co=data['coefficients_rad']
    maxima=[0.0]*3
    for index,t in enumerate(ts):
        j=min(sum(t>=b for b in seg[1:]),len(co)-1)
        for order,key in enumerate(('q_rad','qd_rad_s','qdd_rad_s2')):
            for axis in range(6):
                maxima[order]=max(maxima[order],abs(polynomial(co[j][axis],t-seg[j],order,seg[j+1]-seg[j])-data[key][index][axis]))
    endpoint_q=max(abs(data['q_rad'][0][i]-start[i]) for i in range(6))
    endpoint_q=max(endpoint_q,max(abs(data['q_rad'][-1][i]-goal[i]) for i in range(6)))
    endpoint_v=max(abs(x) for v in (data['qd_rad_s'][0],data['qd_rad_s'][-1]) for x in v)
    endpoint_a=max(abs(x) for a in (data['qdd_rad_s2'][0],data['qdd_rad_s2'][-1]) for x in a)
    joins=[]
    for j,t in enumerate(seg[1:-1],1):
        errors=[max(abs(polynomial(co[j-1][i],t-seg[j-1],o,seg[j]-seg[j-1])-polynomial(co[j][i],0,o,seg[j+1]-seg[j]))
                    for i in range(6)) for o in range(3)]
        joins.append({'time_s':t,'q_jump_rad':errors[0],'qd_jump_rad_s':errors[1],
                      'qdd_jump_rad_s2':errors[2], 'acceleration_jump_allowed':data['method']=='trapezoidal'})
    # 每一子段以独立多项式导数检查两端和所有已知极值点，包括切换左右两侧。
    lim=data['effective_limits']; pv=[0.0]*6; pa=[0.0]*6; pos_ok=True
    T=seg[-1]
    for j,c in enumerate(co):
        L=seg[j+1]-seg[j]
        points=[0.0,L]
        if data['method']=='cubic':points.append(T/2)
        if data['method']=='quintic':points.extend([T/2,T*(3-math.sqrt(3))/6,T*(3+math.sqrt(3))/6])
        for t in points:
            if 0<=t<=L:
                for i in range(6):
                    pv[i]=max(pv[i],abs(polynomial(c[i],t,1,L)))
                    pa[i]=max(pa[i],abs(polynomial(c[i],t,2,L)))
                    pos_ok &= min(start[i],goal[i])-1e-9<=polynomial(c[i],t,0,L)<=max(start[i],goal[i])+1e-9
    rate_ok=all(pv[i]<=lim['joint_velocity_rad_s'][i]+1e-8 and pa[i]<=lim['joint_acceleration_rad_s2'][i]+1e-7 for i in range(6))
    monotone=all(ts[i]>ts[i-1] for i in range(1,len(ts))) and ts[0]==0 and ts[-1]==T
    peak_error=max(abs(pv[i]-data['continuous_joint_check']['peak_velocity_rad_s'][i]) for i in range(6))
    peak_error=max(peak_error,max(abs(pa[i]-data['continuous_joint_check']['peak_acceleration_rad_s2'][i]) for i in range(6)))
    ok=endpoint_q<=1e-9 and endpoint_v<=1e-8 and maxima[0]<=1e-9 and maxima[1]<=1e-8 and maxima[2]<=1e-7
    ok &= rate_ok and pos_ok and monotone and peak_error<=1e-7
    ok &= all(x['q_jump_rad']<=1e-9 and x['qd_jump_rad_s']<=1e-8 for x in joins)
    if data['method']=='quintic':ok &= endpoint_a<=1e-7
    return {'passed':bool(ok),'endpoint_position_error_rad':endpoint_q,'endpoint_velocity_error_rad_s':endpoint_v,
        'endpoint_acceleration_magnitude_rad_s2':endpoint_a,'zero_acceleration_required':data['method']=='quintic',
        'coefficient_sample_max_errors':maxima,'peak_formula_max_difference':peak_error,
        'continuous_peak_velocity_rad_s':pv,'continuous_peak_acceleration_rad_s2':pa,
        'position_monotone':bool(pos_ok),'time_strictly_increasing':monotone,'joins':joins}


def save_curves(output, name, data):
    """保存原始采样CSV和三个六轴SVG面板；单位、方法和加速度采样侧明确标注。"""
    csv_path=output/(name+'.csv')
    with csv_path.open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['time_s']+[f'J{i}_{k}' for k in ('rad','rad_s','rad_s2') for i in range(1,7)])
        for i,t in enumerate(data['time_s']):
            writer.writerow([t]+data['q_rad'][i]+data['qd_rad_s'][i]+data['qdd_rad_s2'][i])
    colors=['#2563eb','#dc2626','#16a34a','#9333ea','#ea580c','#0891b2']
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="960" height="790" viewBox="0 0 960 790">',
         '<rect width="960" height="790" fill="white"/>',
         f'<text x="60" y="26" font-family="sans-serif" font-size="18">{name}: {data["method"]} / {data["continuity"]} / simulation</text>']
    for panel,(key,unit) in enumerate([('q_rad','Position (rad)'),('qd_rad_s','Velocity (rad/s)'),('qdd_rad_s2','Acceleration (rad/s^2)')]):
        top=60+panel*230; values=data[key]; low=min(x for row in values for x in row); high=max(x for row in values for x in row)
        if high==low:low-=.1;high+=.1
        pad=(high-low)*.12;low-=pad;high+=pad
        x=lambda t:80+800*t/data['time_s'][-1]
        y=lambda a:top+175-175*(a-low)/(high-low)
        svg.append(f'<text x="80" y="{top-10}" font-family="sans-serif" font-size="15">{unit}</text>')
        for k in range(5):
            value=low+(high-low)*k/4; yy=y(value)
            svg.append(f'<path d="M80,{yy:.3f} H880" stroke="#e2e8f0"/><text x="8" y="{yy+4:.3f}" font-size="12">{value:.3g}</text>')
        for axis,color in enumerate(colors):
            points=[]
            for i,t in enumerate(data['time_s']):
                # 梯形加速度分段常值，以竖线显示有限跳变；不画成斜坡。
                if key=='qdd_rad_s2' and data['method']=='trapezoidal' and i:
                    points.append(f'{x(t):.3f},{y(values[i-1][axis]):.3f}')
                points.append(f'{x(t):.3f},{y(values[i][axis]):.3f}')
            svg.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="1.7"/>')
        for t in data['segment_times_s']:
            svg.append(f'<path d="M{x(t):.3f},{top} v175" stroke="#94a3b8" stroke-dasharray="3 4"/>')
        for k in range(5):
            t=data['time_s'][-1]*k/4
            svg.append(f'<text x="{x(t):.3f}" y="{top+195}" font-size="12">{t:.3g}s</text>')
    for i,color in enumerate(colors):svg.append(f'<text x="{80+i*100}" y="753" fill="{color}" font-family="sans-serif">J{i+1}</text>')
    svg.append('<text x="80" y="777" font-family="sans-serif" font-size="12">Acceleration: right at switches; left at final endpoint. No TCP/collision acceptance.</text></svg>')
    (output/(name+'.svg')).write_text('\n'.join(svg))
    return [csv_path.name,name+'.svg']


def main():
    """CLI：指定库/输出目录；全部用例满足预期才返回0，失败也保存逐例记录。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,default=ROOT/'build/librobot_planning.so')
    parser.add_argument('--output',type=Path,default=ROOT/'results/joint-planning')
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    planner=JointPlanner(args.library);limits=load_planning_limits()
    model=json.loads((ROOT/'models/ur5/kinematics.json').read_text())
    cases=[];artifacts=[];elapsed=[]
    base=dict(sample_period_s=.1,duration_s=None,duration_policy='stretch',max_duration_s=60)
    def run(name,start,goal,options,expected=0,extra=None,curves=False,instance=planner):
        begin=time.perf_counter_ns();r=instance.plan_joint(start,goal,options);ms=(time.perf_counter_ns()-begin)/1e6
        record={'id':name,'q_start_rad':start,'q_goal_rad':goal,'options':options,'expected_code':expected,
                'actual_code':r['code'],'python_planning_wall_ms':ms,'passed':r['code']==expected,
                'details':r['details'],'message':r['message']}
        if r['code']==0:
            elapsed.append(ms);d=r['data'];record['checks']=verify(d,start,goal);record['passed'] &= record['checks']['passed']
            record.update(duration_s=d['time_s'][-1],segment_times_s=d['segment_times_s'],segment_kind=d['segment_kind'],triangular=d['triangular'])
            if extra:record['extra_check_passed']=bool(extra(d,r));record['passed'] &= record['extra_check_passed']
            if curves:artifacts.extend(save_curves(out,name,d))
        else:record['failure_data_is_null']=r['data'] is None;record['passed'] &= r['data'] is None
        cases.append(record);return r
    zero=[0.0]*6
    for method in ('cubic','quintic','trapezoidal'):
        opts=dict(base,method=method)
        mixed=run(method+'-mixed',zero,[1,-.7,.3,0,.02,-.1],opts,curves=method!='trapezoidal')
        run(method+'-strict-minimum',zero,[1,-.7,.3,0,.02,-.1],
            dict(opts,duration_s=mixed['details']['minimum_duration_s'],duration_policy='strict'))
        run(method+'-hold',zero,zero,opts,extra=lambda d,r:len(d['segment_kind'])==1)
        run(method+'-reverse',[1,-.7,.3,0,.02,-.1],zero,opts)
        run(method+'-short',zero,[.01,0,0,0,0,0],opts,
            extra=(lambda d,r:d['triangular']) if method=='trapezoidal' else None,curves=method=='trapezoidal')
        run(method+'-near-limits',model['q_min_rad'],model['q_max_rad'],opts,curves=False,
            extra=(lambda d,r:not d['triangular'] and 'cruise' in d['segment_kind']) if method=='trapezoidal' else None)
        run(method+'-strict-too-short',zero,[1]*6,dict(opts,duration_s=.1,duration_policy='strict'),3002)
        run(method+'-stretch',zero,[1]*6,dict(opts,duration_s=.1),extra=lambda d,r:r['details']['duration_stretched'])
        run(method+'-strict-long',zero,[1]*6,dict(opts,duration_s=10,duration_policy='strict'),extra=lambda d,r:d['time_s'][-1]==10)
        run(method+'-max-duration',zero,[1]*6,dict(opts,max_duration_s=.1),3001)
        run(method+'-scaled',zero,[1]*6,dict(opts,velocity_scale=.5,acceleration_scale=.25),
            extra=lambda d,r:d['effective_limits']['velocity_scale']==.25 and d['effective_limits']['acceleration_scale']==.125)
    run('trapezoidal-long',zero,[5.5,-3,1.5,0,.1,-.7],dict(base,method='trapezoidal'),
        curves=True,extra=lambda d,r:not d['triangular'])
    opts=dict(base,method='quintic')
    bad=[('nan-start',[math.nan]*6,zero,opts,1001),('bad-length',[0]*5,zero,opts,1001),
        ('position-over',zero,[model['q_max_rad'][0]+.001]+zero[1:],opts,1005),
        ('negative-dt',zero,[1]*6,dict(opts,sample_period_s=-.01),1006),
        ('underflow-dt',zero,zero,dict(opts,sample_period_s=1e-300),1006),
        ('zero-duration',zero,[1]*6,dict(opts,duration_s=0),1006),
        ('inf-duration',zero,[1]*6,dict(opts,duration_s=math.inf),1006),
        ('unknown-method',zero,zero,dict(opts,method='spline'),1008),
        ('invalid-policy',zero,zero,dict(opts,duration_policy='ignore'),1001),
        ('auto-strict',zero,zero,dict(opts,duration_policy='strict'),1001),
        ('scale-increase',zero,zero,dict(opts,velocity_scale=2),1001),
        ('nonzero-boundary-unsupported',zero,zero,dict(opts,qd_start_rad_s=[1]*6),1008),
        ('collision-required',zero,zero,dict(opts,collision_policy='required'),1008),
        ('sampling-budget',zero,[1]*6,dict(opts,sample_period_s=1e-9),3001),
        ('margin-over',zero,zero,dict(opts,joint_margin_rad=10),1001)]
    for args2 in bad:run(*args2)
    # 缺配置拒绝正式规划；并逐项删掉前置未完成参数，不借用无限制。
    run('missing-config',zero,[1]*6,opts,1004,instance=JointPlanner(args.library,out/'missing.json'))
    original=json.loads((ROOT/'config/planning-limits.v1.json').read_text())
    for section,key in [('joint_limits','acceleration_rad_s2')]+[('tcp_limits',k) for k in original['tcp_limits']]:
        broken=copy.deepcopy(original);del broken[section][key];path=out/'negative-config.json';path.write_text(json.dumps(broken))
        run('missing-'+key,zero,[1]*6,opts,1004,instance=JointPlanner(args.library,path))
    (out/'negative-config.json').unlink()
    # 梯形/三角形临界行程v²/a（J1），不生成零时长匀速子段。
    critical=limits.joint_velocity_rad_s[0]**2/limits.joint_acceleration_rad_s2[0]
    run('triangle-threshold',zero,[critical]+zero[1:],dict(base,method='trapezoidal'),extra=lambda d,r:d['triangular'])
    for factor in (.999, 1.001):
        run('triangle-threshold-'+str(factor),zero,[critical*factor]+zero[1:],dict(base,method='trapezoidal'),
            extra=lambda d,r,factor=factor:d['triangular']==(factor<1))
    rng=random.Random(20261010)
    for index in range(8):
        start=[rng.uniform(lo*.8,hi*.8) for lo,hi in zip(model['q_min_rad'],model['q_max_rad'])]
        goal=[rng.uniform(lo*.8,hi*.8) for lo,hi in zip(model['q_min_rad'],model['q_max_rad'])]
        for method in ('cubic','quintic','trapezoidal'):
            run('seed20261010-'+str(index)+'-'+method,start,goal,dict(base,method=method))
    # C边界：失败必须不改输出。直接调用同一个编译库，崩溃将导致入口非0。
    def ccase(name, expected, invoke):
        p=CPlan();C.memset(C.byref(p),0x5a,C.sizeof(p));before=bytes(p)
        code=invoke(p);cases.append({'id':name,'expected_code':expected,'actual_code':code,
            'output_untouched':bytes(p)==before,'passed':code==expected and bytes(p)==before})
    arrays=[D6(*zero),D6(*([1]*6)),D6(*model['q_min_rad']),D6(*model['q_max_rad']),D6(*limits.joint_velocity_rad_s),D6(*limits.joint_acceleration_rad_s2)]
    for j in range(6):
        x=list(arrays);x[j]=None
        ccase('C-null-input-'+str(j),1001,lambda p,x=x:planner.fn(1,*x,6,0,1,.1,60,C.byref(p)))
    for n in (0,5,7):ccase('C-length-'+str(n),1001,lambda p,n=n:planner.fn(1,*arrays,n,0,1,.1,60,C.byref(p)))
    ccase('C-null-output',1001,lambda p:planner.fn(1,*arrays,6,0,1,.1,60,None))
    for j in (4,5):
        x=list(arrays);x[j]=D6(*([-1]*6))
        ccase('C-negative-cap-'+str(j),1001,lambda p,x=x:planner.fn(1,*x,6,0,1,.1,60,C.byref(p)))
    good=CPlan();assert planner.fn(1,*arrays,6,0,1,.1,60,C.byref(good))==0
    for name,t,expected in [('before',-.1,1006),('after',good.duration_s+.1,1006),('nan',math.nan,1006)]:
        q,v,a=D6(*([99]*6)),D6(*([99]*6)),D6(*([99]*6));code=planner.evaluate(C.byref(good),t,q,v,a,6)
        cases.append({'id':'C-evaluate-'+name,'actual_code':code,'expected_code':expected,
                      'passed':code==expected and list(q)==list(v)==list(a)==[99]*6})
    passed=sum(c['passed'] for c in cases)
    timing=sorted(elapsed)
    p95index=(len(timing)-1)*.95;lower=math.floor(p95index);upper=math.ceil(p95index)
    sources=['include/robot_planning.h','src/planning/joint.c','src/planning/joint.py','scripts/verify_joint_planning.py']
    report={'status':'passed' if passed==len(cases) else 'failed','passed_count':passed,'expected_count':len(cases),
        'scope':'rest-to-rest joint planning; no TCP, collision or hardware acceptance',
        'thresholds':{'position_rad':1e-9,'velocity_rad_s':1e-8,'acceleration_rad_s2':1e-7},
        'effective_limits':limits.as_dict(),'source_sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources},
        'library_sha256':hashlib.sha256(args.library.read_bytes()).hexdigest(),
        'timing':{'scope':'Python plan_joint wall includes shared load, C planning/evaluation and full serialization; not C-only',
            'valid_count':len(timing),'mean_ms':sum(timing)/len(timing),
            'p95_ms':timing[lower]+(timing[upper]-timing[lower])*(p95index-lower),'maximum_ms':max(timing),
            'quantile':'linear interpolation (n-1)*p','timeout_scope':'unified runner whole process budget 120s; no per-call interruption'},
        'artifacts':[{ 'path':p,'sha256':hashlib.sha256((out/p).read_bytes()).hexdigest()} for p in artifacts], 'cases':cases}
    # 非法输入保留可读NaN/Inf字符串，输出严格JSON，不能伪填零。
    def clean(x):
        if isinstance(x,float) and not math.isfinite(x):return str(x)
        if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
        if isinstance(x,list):return [clean(v) for v in x]
        return x
    (out/'report.json').write_text(json.dumps(clean(report),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(f'{passed}/{len(cases)} joint planning checks passed')
    for c in cases:
        if not c['passed']:print(json.dumps(clean(c),ensure_ascii=False))
    return 0 if passed==len(cases) else 1

if __name__=='__main__':sys.exit(main())
