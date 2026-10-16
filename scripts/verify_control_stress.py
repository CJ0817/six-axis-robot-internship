"""Physical clipping/release records and explicit oscillation case tables."""
import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from control.pid import PID,FeedforwardPID
from control.diagnostics import saturation,oscillation
from verify_pid_feedforward import TrialPlant,reference,settled
from verify_single_joint_pid import sha


class StressPlant(TrialPlant):
    """Clamp-limited actuator and separate physical external torque in DIRECT."""
    def advance(self,output,external):
        """Send actuator command at every substep; report exact sent command bounds."""
        commands=[]
        if self.kind=='pybullet':
            p=self.pb
            for _ in range(self.substeps):
                p.setJointMotorControl2(self.robot,self.joint,p.TORQUE_CONTROL,force=output,physicsClientId=self.client)
                if external:p.applyExternalTorque(self.robot,self.joint,[0.,0.,external],p.WORLD_FRAME,physicsClientId=self.client)
                commands.append(output);p.stepSimulation(physicsClientId=self.client)
        else:
            super().advance(output,external);commands=[output]*self.substeps
        return min(commands),max(commands)


def save_csv(path,rows,fields=None):
    """Persist full tables, including a header for an empty exception list."""
    with path.open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=fields or list(rows[0]));w.writeheader();w.writerows(rows)


def plot_pair(pid,ff,path,limit):
    """Separate request, physically sent actuator, integral and recovery plots."""
    panels=[('error_rad','Tracking error (rad)'),('total_raw','Requested torque before clamp (N m)'),
            ('physics_actuator_max_nm','Actuator command sent each physics substep (N m)'),('integral','Integral term (N m)')]
    parts=['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="940"><rect width="100%" height="100%" fill="white"/>']
    for panel,(key,title) in enumerate(panels):
        top=35+panel*225;values=[r[key] for rows in (pid,ff) for r in rows]
        if panel==2:values += [-limit,limit]
        lo=min(values);hi=max(values);span=hi-lo or 1.;lo-=.1*span;hi+=.1*span
        y=lambda v:top+160*(hi-v)/(hi-lo)
        parts.append(f'<text x="100" y="{top-12}" font-size="15">{title}</text>')
        for j in range(5):
            value=hi-(hi-lo)*j/4;yy=y(value)
            parts.append(f'<path d="M100 {yy} H1060" stroke="#e2e8f0"/><text x="5" y="{yy+4}" font-size="12">{value:.4g}</text>')
        if panel==2:
            for v in (-limit,limit):parts.append(f'<path d="M100 {y(v)} H1060" stroke="#111827" stroke-dasharray="6 4"/>')
        for n,(label,rows,color) in enumerate((('PID',pid,'#2563eb'),('PID + FF',ff,'#dc2626'))):
            xy=' '.join(f"{100+960*r['time_s']/rows[-1]['time_s']:.2f},{y(r[key]):.2f}" for r in rows)
            parts.append(f'<polyline points="{xy}" fill="none" stroke="{color}" stroke-width="1.3"/><text x="{820+120*n}" y="{top-12}" fill="{color}">{label}</text>')
        for j in range(7):
            t=pid[-1]['time_s']*j/6
            parts.append(f'<text x="{100+960*j/6}" y="{top+180}" font-size="12">{t:.1f}</text>')
    parts.append('<text x="440" y="925">Simulation time (s); full raw nodes in CSV</text></svg>')
    path.write_text(''.join(parts),encoding='utf-8')


def main():
    """Execute matched dynamic profiles, then classify every old and new case."""
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--library',type=Path,required=True)
    p.add_argument('--plant',choices=('pybullet','second_order'),default='pybullet')
    p.add_argument('--config',type=Path,default=ROOT/'config/control-stress.v1.json')
    p.add_argument('--baseline-output',type=Path,default=ROOT/'results/pid-feedforward-pybullet')
    p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    report=dict(status='failed',plant=args.plant,trials=[],paired_checks=[],oscillation_cases=[],exceptions=[])
    try:
        settings=json.loads(args.config.read_text(encoding='utf-8'));base=json.loads((ROOT/settings['base_config']).read_text(encoding='utf-8'))
        if settings['version']!='1.0.0':raise ValueError('Unsupported stress configuration')
        dt=base['dt_s'];count=round(settings['duration_s']/dt);limit=settings['output_limit_nm']
        report.update(config=settings,base_config_sha256=sha(ROOT/settings['base_config']),config_sha256=sha(args.config),
            library_sha256=sha(args.library),model_sha256=sha(ROOT/'models/ur5/ur5.urdf'),
            source_sha256={s:sha(ROOT/s) for s in ('src/control/diagnostics.py','scripts/verify_control_stress.py','scripts/verify_pid_feedforward.py','src/control/pid.c')})
        passed=True
        def diagnose(name,rows,start,source,diagnostic):
            """Append explicit negative/positive decision plus all supporting windows."""
            d=oscillation(rows,start,dt,settings['oscillation'])
            report['oscillation_cases'].append(dict(trial=name,source=source,diagnostic_only=diagnostic,**d))
            if d['triggered']:report['exceptions'].append(dict(trial=name,category='oscillation',diagnostic_only=diagnostic,value=d['trigger_time_s']))
            return d
        for profile in settings['profiles']:
            c=copy.deepcopy(base);c['duration_s']=settings['duration_s']
            c['pid'].update(output_min=-limit,output_max=limit,**profile.get('pid_override',{}))
            c['trajectory'].update(move_target_rad=profile['target_rad'],move_duration_s=profile['move_duration_s'])
            refs=[reference('move',k*dt,c) for k in range(count)]
            forces=[profile['pulse_nm'] if profile['pulse_start_s']<=k*dt<profile['pulse_end_s'] else 0. for k in range(count)]
            signature=hashlib.sha256(json.dumps([refs,forces]).encode()).hexdigest()
            for load in c['load_mass_kg']:
                curves={};parameters={}
                for mode in ('pid','pid_ff'):
                    for repeat in range(c['repetitions']):
                        name=f"{profile['id']}-load{load:g}-{mode}-r{repeat+1}";rows=[]
                        plant=StressPlant(args.plant,c,0.,load)
                        try:
                            parameters[mode]=plant.parameters
                            controller=PID(args.library,c['pid']) if mode=='pid' else FeedforwardPID(args.library,c['pid'],plant.parameters['kv_nm_s_rad'],plant.parameters['ka_nm_s2_rad'])
                            for k,(ref,v,a) in enumerate(refs):
                                q,qd=plant.read();r=controller.step(ref,q) if mode=='pid' else controller.step(ref,q,v,a)
                                if r['code']:raise RuntimeError('Controller error '+str(r['code']))
                                d=r['data'];sentlo,senthi=plant.advance(d['output'],forces[k])
                                rows.append(dict(node=k,time_s=k*dt,reference_rad=ref,reference_velocity_rad_s=v,reference_acceleration_rad_s2=a,
                                    actual_rad=q,actual_velocity_rad_s=qd,error_rad=ref-q,feedback_raw=d.get('feedback_raw',d.get('raw_output')),
                                    velocity_feedforward=d.get('velocity_feedforward',0.),acceleration_feedforward=d.get('acceleration_feedforward',0.),
                                    total_raw=d.get('total_raw',d.get('raw_output')),output=d['output'],physics_actuator_min_nm=sentlo,physics_actuator_max_nm=senthi,
                                    external_world_z_torque_nm=forces[k],integral=d['integral'],integral_limited=d['integral_limited'],output_limited=d['output_limited'],code=0))
                        finally:
                            plant.close()
                            if rows:save_csv(args.output/(name+'.csv'),rows)
                        sat=saturation(rows,c);release=sat['final_release_s']
                        sat['recovery_time_after_final_release_s']=settled(rows,release,settings['recovery_band_rad'],settings['recovery_dwell_s']) if release is not None else None
                        diag=diagnose(name,rows,c['trajectory']['start_s']+profile['move_duration_s'],args.plant,profile['diagnostic_only'])
                        good=len(rows)==count and all(all(math.isfinite(x) for x in r.values()) for r in rows) and not any(sat[k] for k in ('actual_limit_violation_count','clipping_mismatch_count','integral_limit_violation_count','integral_windup_direction_violation_count'))
                        good &= sat['longest_clipped_interval_s']>=settings['minimum_clipped_interval_s']
                        good &= all(-limit-1e-12<=r['physics_actuator_min_nm']<=r['physics_actuator_max_nm']<=limit+1e-12 for r in rows)
                        if not profile['diagnostic_only']:good &= release is not None and sat['recovery_time_after_final_release_s'] is not None
                        passed &= good
                        report['trials'].append(dict(name=name,profile=profile['id'],mode=mode,repeat=repeat+1,load_kg=load,diagnostic_only=profile['diagnostic_only'],
                            status='passed' if good else 'failed',reference_disturbance_sha256=signature,parameters=dict(pid=c['pid'],plant=plant.parameters),saturation=sat,
                            oscillation_status=diag['status'],final_error_rad=rows[-1]['error_rad'],maximum_abs_position_rad=max(abs(r['actual_rad']) for r in rows),csv_sha256=sha(args.output/(name+'.csv'))))
                        if sat['request_exceeded_count']:report['exceptions'].append(dict(trial=name,category='request_saturation',diagnostic_only=profile['diagnostic_only'],value=sat['request_exceeded_count']))
                        if sat['actual_limit_violation_count']:report['exceptions'].append(dict(trial=name,category='actual_limit_violation',diagnostic_only=profile['diagnostic_only'],value=sat['actual_limit_violation_count']))
                        if sat['recovery_time_after_final_release_s'] is None:report['exceptions'].append(dict(trial=name,category='not_recovered_in_observation',diagnostic_only=profile['diagnostic_only'],value=None))
                        curves[(mode,repeat)]=rows
                delta=max(abs(a['actual_rad']-b['actual_rad']) for mode in ('pid','pid_ff') for rep in range(1,c['repetitions']) for a,b in zip(curves[(mode,0)],curves[(mode,rep)]))
                fair=parameters['pid']==parameters['pid_ff'];passed &= fair and delta<=base['functional_gates']['repeatability_tolerance_rad']
                report['paired_checks'].append(dict(profile=profile['id'],load_kg=load,matched_parameters=fair,reference_disturbance_sha256=signature,repeatability_max_abs_delta_rad=delta))
                plot_pair(curves[('pid',0)],curves[('pid_ff',0)],args.output/f"{profile['id']}-load{load:g}-recovery.svg",limit)
        # The earlier nominal cases also receive an explicit decision, never an implicit pass.
        original=json.loads((args.baseline_output/'report.json').read_text(encoding='utf-8'))
        for trial in original['trials']:
            with (args.baseline_output/(trial['name']+'.csv')).open(newline='',encoding='utf-8') as f:rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(f)]
            diagnose(trial['name'],rows,trial['metrics']['settling_anchor_s'],original['plant']+'_nominal',False)
        report['sample_count']=sum(count for _ in report['trials'])
        report['oscillation_triggered_count']=sum(x['triggered'] for x in report['oscillation_cases'])
        report['oscillation_not_triggered_count']=sum(x['status']=='not_detected' for x in report['oscillation_cases'])
        report['diagnostic_oscillation_observed']=any(x['triggered'] and x['diagnostic_only'] for x in report['oscillation_cases'])
        passed &= report['diagnostic_oscillation_observed']
        report['status']='passed' if passed else 'failed'
    except Exception as exc:
        report['error']=type(exc).__name__+': '+str(exc);traceback.print_exc()
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    if report['trials']:
        keys=['request_exceeded_count','actual_clipped_count','actual_limit_violation_count','request_peak_abs_nm','actual_peak_abs_nm','integral_peak_abs_nm','integral_windup_direction_violation_count','longest_clipped_interval_s','final_release_s','recovery_time_after_final_release_s']
        save_csv(args.output/'saturation-cases.csv',[dict(trial=t['name'],diagnostic_only=t['diagnostic_only'],**{k:t['saturation'][k] for k in keys}) for t in report['trials']])
        save_csv(args.output/'oscillation-cases.csv',[dict(trial=t['trial'],source=t['source'],diagnostic_only=t['diagnostic_only'],status=t['status'],trigger_time_s=t['trigger_time_s'],full_windows=len(t['windows']),maximum_peak_to_peak_rad=max((w['peak_to_peak_rad'] for w in t['windows']),default=0.)) for t in report['oscillation_cases']])
        save_csv(args.output/'exception-cases.csv',report['exceptions'],['trial','category','diagnostic_only','value'])
    print(json.dumps(dict(status=report['status'],plant=args.plant,trials=len(report['trials']),oscillation_triggered=report.get('oscillation_triggered_count'),error=report.get('error'))))
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':sys.exit(main())
