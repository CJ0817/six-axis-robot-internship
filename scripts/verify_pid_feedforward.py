"""Paired PID/velocity+acceleration feedforward trials and disturbance evidence."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import traceback
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from control.pid import PID,FeedforwardPID
from verify_single_joint_pid import Plant as BaselinePlant,sha

CASES=('move','sine','hold_pulse','hold_sustained','sine_pulse')


def reference(case,t,c):
    """Return analytic (rad,rad/s,rad/s^2), with C2 endpoints and no differencing."""
    c=c['trajectory'];x=t-c['start_s']
    if case.startswith('hold'):return c['hold_rad'],0.,0.
    if case=='move':
        T=c['move_duration_s'];u=min(1.,max(0.,x/T));A=c['move_target_rad']
        return A*u**3*(10-15*u+6*u*u),A*30*u*u*(1-u)**2/T,A*60*u*(1-u)*(1-2*u)/T**2
    T=c['sine_duration_s']
    if x<=0 or x>=T:return 0.,0.,0.
    A=c['sine_amplitude_rad'];w=2*math.pi*c['sine_frequency_hz'];b=math.pi/T
    s=math.sin(w*x);cs=math.cos(w*x);E=math.sin(b*x)**2
    Ed=b*math.sin(2*b*x);Edd=2*b*b*math.cos(2*b*x)
    return A*E*s,A*(Ed*s+E*w*cs),A*(Edd*s+2*Ed*w*cs-E*w*w*s)


def disturbance(case,t,c):
    """External J1 torque after actuator clipping; exact half-open pulse interval."""
    d=c['disturbance']
    if case.endswith('sustained') and t>=d['start_s']:return d['torque_nm']
    if case.endswith('pulse') and d['start_s']<=t<d['pulse_end_s']:return d['torque_nm']
    return 0.


class TrialPlant(BaselinePlant):
    """Real UR5 J1 or explicit second-order model; identical reset per paired run."""
    def __init__(self,kind,c,initial,load):
        self.kind=kind;self.config=c;self.q=initial;self.v=0.;self.client=None
        b=c['joint_viscous_damping_nm_s_rad'];radius=c['added_load_radius_m']
        self.inertia=c['second_order_model']['nominal_inertia_kg_m2']+load*c['second_order_model']['payload_radius_to_J1_m']**2+.4*load*radius**2
        self.substeps=c['pybullet']['physics_substeps']
        self.parameters=dict(added_load_kg=load,load_link=c['load_link'],viscous_damping_nm_s_rad=b)
        if kind=='pybullet':
            try:
                from importlib.metadata import version
                if version('pybullet')!='3.2.7':raise ValueError('Frozen PyBullet 3.2.7 required')
                super().__init__('pybullet',c,initial)
                pb=self.pb
                for link in range(-1,pb.getNumJoints(self.robot,self.client)):
                    pb.changeDynamics(self.robot,link,linearDamping=0.,angularDamping=0.,physicsClientId=self.client)
                    if link>=0:pb.changeDynamics(self.robot,link,jointDamping=0.,physicsClientId=self.client)
                pb.changeDynamics(self.robot,self.joint,jointDamping=b,physicsClientId=self.client)
                movable=[j for j in range(pb.getNumJoints(self.robot,self.client)) if pb.getJointInfo(self.robot,j,self.client)[2]==pb.JOINT_REVOLUTE]
                links={pb.getJointInfo(self.robot,j,self.client)[12].decode():j for j in range(pb.getNumJoints(self.robot,self.client))}
                link=links[c['load_link']];info=pb.getDynamicsInfo(self.robot,link,physicsClientId=self.client)
                inertia=[x+.4*load*radius**2 for x in info[2]]
                pb.changeDynamics(self.robot,link,mass=info[0]+load,localInertiaDiagonal=inertia,physicsClientId=self.client)
                q=[pb.getJointState(self.robot,j,self.client)[0] for j in movable]
                self.inertia=pb.calculateMassMatrix(self.robot,q,physicsClientId=self.client)[0][0]
                self.parameters.update(original_link_mass_kg=info[0],loaded_link_mass_kg=info[0]+load,
                    loaded_link_inertia_diagonal_kg_m2=inertia,load_COM_in_link_m=list(info[3]))
            except Exception:
                self.close();raise
        self.parameters.update(effective_inertia_kg_m2=self.inertia,
            kv_nm_s_rad=b,ka_nm_s2_rad=self.inertia*c['feedforward']['inertia_scale'])

    def advance(self,output,external):
        """Apply clipped actuator and independent disturbance for one fixed dt."""
        if self.kind=='pybullet':super().advance(output+external)
        else:
            h=self.config['dt_s']/self.substeps;b=self.parameters['viscous_damping_nm_s_rad']
            for _ in range(self.substeps):
                self.v+=h*(output+external-b*self.v)/self.inertia;self.q+=h*self.v


def settled(rows,anchor,band,dwell,key='error_rad'):
    """First sample after last out-of-band point, requiring >=dwell tail duration."""
    tail=[r for r in rows if r['time_s']>=anchor]
    if not tail:return None
    bad=[i for i,r in enumerate(tail) if abs(r[key])>band]
    index=bad[-1]+1 if bad else 0
    if index>=len(tail) or tail[-1]['time_s']-tail[index]['time_s']<dwell-1e-12:return None
    return tail[index]['time_s']-anchor


def metrics(rows,case,c):
    """All-sample tracking RMSE; final constant-reference bias; explicit anchors."""
    m=c['metrics'];tr=c['trajectory'];d=c['disturbance'];dt=c['dt_s']
    anchor=(tr['start_s']+tr['move_duration_s'] if case=='move' else
        tr['start_s']+tr['sine_duration_s'] if case.startswith('sine') else
        d['pulse_end_s'] if case.endswith('pulse') else d['start_s'])
    amplitude=abs(tr['move_target_rad']) if case=='move' else 0.
    band=max(m['absolute_settling_band_rad'],m['relative_settling_band']*amplitude)
    count=round(m['steady_window_s']/dt);tail=rows[-count:]
    active=[r for r in rows if r['time_s']>=anchor]
    overshoot=max(0.,max(-math.copysign(1.,tr['move_target_rad'])*r['error_rad'] for r in active)) if case=='move' else None
    disturbed=case.endswith(('pulse','sustained'))
    recovery_anchor=d['pulse_end_s'] if case.endswith('pulse') else d['start_s']
    disturbance_rows=[r for r in rows if r['time_s']>=d['start_s']] if disturbed else []
    tracking=[r for r in rows if tr['start_s']<=r['time_s']<anchor] if not case.startswith('hold') else rows
    result=dict(tracking_rmse_rad=math.sqrt(sum(r['error_rad']**2 for r in rows)/len(rows)),
        tracking_active_rmse_rad=math.sqrt(sum(r['error_rad']**2 for r in tracking)/len(tracking)),
        steady_state_bias_rad=sum(r['error_rad'] for r in tail)/len(tail),
        steady_state_mae_rad=sum(abs(r['error_rad']) for r in tail)/len(tail),
        final_error_rad=rows[-1]['error_rad'],maximum_abs_error_rad=max(abs(r['error_rad']) for r in rows),
        overshoot_rad=overshoot,overshoot_percent=100*overshoot/amplitude if amplitude else None,
        settling_time_s=settled(rows,anchor,band,m['minimum_settling_dwell_s']),settling_anchor_s=anchor,settling_band_rad=band,
        disturbance_recovery_time_s=settled(rows,recovery_anchor,band,m['minimum_settling_dwell_s']) if disturbed else None,
        disturbance_anchor_s=recovery_anchor if disturbed else None,
        disturbance_peak_abs_tracking_error_rad=max(abs(r['error_rad']) for r in disturbance_rows) if disturbed else None,
        output_limited_count=sum(r['output_limited'] for r in rows),integral_limited_count=sum(r['integral_limited'] for r in rows))
    if disturbed:
        for name,lo,hi in (('before',0.,d['start_s']),('during',d['start_s'],d['pulse_end_s'] if case.endswith('pulse') else c['duration_s']),('after',d['pulse_end_s'],c['duration_s'])):
            window=[r['error_rad'] for r in rows if lo<=r['time_s']<hi] if name!='after' or case.endswith('pulse') else []
            result['disturbance_'+name+'_rmse_rad']=math.sqrt(sum(e*e for e in window)/len(window)) if window else None
    return result


def comparison_svg(pid,ff,path):
    """Plot measured paired position/error/output curves, preserving original CSV."""
    parts=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="720"><rect width="100%" height="100%" fill="white"/>']
    for panel,(key,title) in enumerate((('actual_rad','Position (rad)'),('error_rad','Tracking error (rad)'),('output','Clipped actuator torque (N m)'))):
        series=[('PID',pid,key,'#2563eb'),('PID + FF',ff,key,'#dc2626')]
        if panel==0:series.append(('reference',pid,'reference_rad','#64748b'))
        values=[r[k] for _,rs,k,_ in series for r in rs];lo=min(values);hi=max(values);span=hi-lo or 1.;lo-=.1*span;hi+=.1*span;top=35+panel*230
        parts.append(f'<text x="90" y="{top-10}" font-size="15">{title}</text>')
        for j in range(5):
            y=top+170*j/4;v=hi-(hi-lo)*j/4
            parts.append(f'<path d="M90 {y} H970" stroke="#e2e8f0"/><text x="5" y="{y+4}" font-size="12">{v:.4g}</text>')
        for n,(label,rs,k,color) in enumerate(series):
            xy=' '.join(f"{90+880*r['time_s']/rs[-1]['time_s']:.2f},{top+170*(hi-r[k])/(hi-lo):.2f}" for r in rs)
            parts.append(f'<polyline points="{xy}" fill="none" stroke="{color}" stroke-width="1.4"/><text x="{600+n*120}" y="{top-10}" fill="{color}">{label}</text>')
    parts.append(f'<text x="280" y="710">Simulation time: 0 to {pid[-1]["time_s"]:.2f} seconds; disturbance in CSV</text></svg>')
    path.write_text(''.join(parts),encoding='utf-8')


def main():
    """Run all matched pairs and repeats; save raw traces even on later failure."""
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--library',type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'config/pid-feedforward.v1.json')
    p.add_argument('--plant',choices=('second_order','pybullet'),default='second_order')
    p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    report=dict(status='failed',plant=args.plant,trials=[],comparisons=[],environment=dict(python=platform.python_version(),platform=platform.platform()))
    try:
        c=json.loads(args.config.read_text(encoding='utf-8'));dt=c['dt_s'];count=round(c['duration_s']/dt)
        if c['version']!='1.0.0' or dt!=c['pid']['dt_s'] or c['repetitions']<2 or count<2:raise ValueError('Unsupported/inconsistent experiment settings')
        if c['trajectory']['sine_duration_s']*c['trajectory']['sine_frequency_hz']%1!=0:raise ValueError('Sine envelope requires whole periods for C2 endpoint')
        report.update(config=c,config_sha256=sha(args.config),library_sha256=sha(args.library),model_sha256=sha(ROOT/'models/ur5/ur5.urdf'),
            source_sha256={s:sha(ROOT/s) for s in ('src/control/pid.c','src/control/pid.py','scripts/verify_pid_feedforward.py')})
        all_passed=True
        for load in c['load_mass_kg']:
            for case in CASES:
                refs=[reference(case,k*dt,c) for k in range(count)];dist=[disturbance(case,k*dt,c) for k in range(count)]
                trace_hash=hashlib.sha256(json.dumps([refs,dist]).encode()).hexdigest();runs={};params={}
                for mode in ('pid','pid_ff'):
                    for repeat in range(c['repetitions']):
                        rows=[];plant=TrialPlant(args.plant,c,refs[0][0],load)
                        name=f'{case}-load{load:g}-{mode}-r{repeat+1}'
                        try:
                            params[mode]=plant.parameters
                            control=PID(args.library,c['pid']) if mode=='pid' else FeedforwardPID(args.library,c['pid'],plant.parameters['kv_nm_s_rad'],plant.parameters['ka_nm_s2_rad'])
                            for k,(ref,v,a) in enumerate(refs):
                                q,qd=plant.read();r=control.step(ref,q) if mode=='pid' else control.step(ref,q,v,a)
                                if r['code']:raise RuntimeError('Control failed '+str(r['code']))
                                d=r['data'];rows.append(dict(node=k,time_s=k*dt,reference_rad=ref,reference_velocity_rad_s=v,reference_acceleration_rad_s2=a,
                                    actual_rad=q,actual_velocity_rad_s=qd,error_rad=ref-q,feedback_raw=d.get('feedback_raw',d.get('raw_output')),
                                    velocity_feedforward=d.get('velocity_feedforward',0.),acceleration_feedforward=d.get('acceleration_feedforward',0.),
                                    total_raw=d.get('total_raw',d.get('raw_output')),output=d['output'],disturbance_nm=dist[k],net_joint_torque_nm=d['output']+dist[k],integral=d['integral'],
                                    output_limited=d['output_limited'],integral_limited=d['integral_limited'],code=0))
                                plant.advance(d['output'],dist[k])
                        finally:
                            plant.close()
                            if rows:
                                with (args.output/(name+'.csv')).open('w',newline='',encoding='utf-8') as f:
                                    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
                        stats=metrics(rows,case,c);g=c['functional_gates'];pid=c['pid']
                        passed=len(rows)==count and abs(stats['final_error_rad'])<=g['maximum_final_abs_error_rad'] and all(
                            all(math.isfinite(x) for x in row.values()) and abs(row['actual_rad'])<=g['maximum_abs_position_rad'] and
                            pid['output_min']<=row['output']<=pid['output_max'] and pid['integral_min']<=row['integral']<=pid['integral_max'] for row in rows)
                        report['trials'].append(dict(name=name,case=case,mode=mode,repeat=repeat+1,load_kg=load,status='passed' if passed else 'failed',
                            reference_disturbance_sha256=trace_hash,parameters=dict(pid=c['pid'],plant=plant.parameters),metrics=stats,csv_sha256=sha(args.output/(name+'.csv'))))
                        all_passed &= passed;runs[(mode,repeat)]=rows
                repeated=max(abs(a['actual_rad']-b['actual_rad']) for mode in ('pid','pid_ff') for repeat in range(1,c['repetitions']) for a,b in zip(runs[(mode,0)],runs[(mode,repeat)]))
                fair=params['pid']==params['pid_ff'];all_passed &= fair and repeated<=c['functional_gates']['repeatability_tolerance_rad']
                m0=metrics(runs[('pid',0)],case,c);m1=metrics(runs[('pid_ff',0)],case,c)
                report['comparisons'].append(dict(case=case,load_kg=load,matched_parameters=fair,repeatability_max_abs_delta_rad=repeated,
                    pid=m0,pid_ff=m1,rmse_reduction_percent=100*(m0['tracking_rmse_rad']-m1['tracking_rmse_rad'])/m0['tracking_rmse_rad'] if m0['tracking_rmse_rad'] else None))
                comparison_svg(runs[('pid',0)],runs[('pid_ff',0)],args.output/f'{case}-load{load:g}-comparison.svg')
        report['status']='passed' if all_passed else 'failed'
        report['sample_count']=sum(count for _ in report['trials'])
    except Exception as exc:
        report['error']=type(exc).__name__+': '+str(exc);traceback.print_exc()
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    # A portable numeric parameter table and compact comparison table are exported.
    if report['trials']:
        with (args.output/'parameters.csv').open('w',newline='',encoding='utf-8') as f:
            fields=['case','mode','load_kg','repeat','dt_s','kp','ki','kd','integral_min','integral_max','output_min','output_max','kv_nm_s_rad','ka_nm_s2_rad','effective_inertia_kg_m2'];w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
            for trial in report['trials']:
                values={k:trial[k] for k in fields[:4]};values.update(trial['parameters']['pid']);values.update({k:trial['parameters']['plant'][k] for k in fields[-3:]})
                if trial['mode']=='pid':values.update(kv_nm_s_rad=0.,ka_nm_s2_rad=0.)
                w.writerow(values)
        with (args.output/'comparison.csv').open('w',newline='',encoding='utf-8') as f:
            fields=['case','load_kg','mode','tracking_rmse_rad','tracking_active_rmse_rad','steady_state_bias_rad','steady_state_mae_rad','overshoot_percent','settling_time_s','disturbance_recovery_time_s','disturbance_peak_abs_tracking_error_rad'];w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
            for pair in report['comparisons']:
                for mode in ('pid','pid_ff'):w.writerow(dict(case=pair['case'],load_kg=pair['load_kg'],mode=mode,**{k:pair[mode][k] for k in fields[3:]}))
    print(json.dumps(dict(status=report['status'],plant=args.plant,trials=len(report['trials']),comparisons=len(report['comparisons']),error=report.get('error'))))
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':sys.exit(main())
