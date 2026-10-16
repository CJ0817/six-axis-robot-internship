"""Explicit finite-window oscillation criterion for constant-reference phases."""
import math


def oscillation(rows,start_s,dt_s,rule):
    """Analyze error around zero, returning every window and persistence decision.

    Non-overlapping full windows only. Hysteretic signed excursions define
    extrema; each qualified half-cycle must exceed the peak-to-peak threshold.
    Two qualified half-cycles form one full cycle. Consecutive qualifying
    windows are required. A missing full window is INSUFFICIENT, not negative.
    This is a finite-duration threshold diagnosis, not a proof of instability.
    """
    W=rule['window_s'];h=rule['hysteresis_rad'];amplitude=rule['minimum_peak_to_peak_rad']
    cycles=rule['minimum_full_cycles'];required=rule['consecutive_windows']
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (start_s,dt_s,W,h,amplitude)) or start_s<0 or dt_s<=0 or W<=0 or h<=0 or amplitude<=2*h:
        raise ValueError('Invalid oscillation window/amplitude/hysteresis')
    if type(cycles) is not int or cycles<1 or type(required) is not int or required<2:
        raise ValueError('Positive cycle count and at least two consecutive windows required')
    if not rows:return dict(status='insufficient_window',triggered=False,windows=[],trigger_time_s=None)
    if any(not math.isfinite(r['error_rad']) or not math.isfinite(r['time_s']) for r in rows):raise ValueError('Nonfinite diagnostic input')
    if any(abs((b['time_s']-a['time_s'])-dt_s)>1e-8 for a,b in zip(rows,rows[1:])):raise ValueError('Nonuniform/missing diagnostic samples')
    first=math.ceil((start_s-rows[0]['time_s'])/dt_s-1e-8);first=max(0,first)
    size=round(W/dt_s)
    if size<2 or abs(size*dt_s-W)>1e-8:raise ValueError('Window must contain an integer number of samples')
    windows=[];streak=0;trigger=None
    for offset in range(first,len(rows)-size+1,size):
        block=rows[offset:offset+size];extrema=[];state=0;peak=0.
        for row in block:
            e=row['error_rad'];sign=1 if e>=h else -1 if e<=-h else 0
            if state==0:
                if sign:state=sign;peak=e
            elif sign and sign!=state:
                extrema.append(peak);state=sign;peak=e
            else:peak=max(peak,e) if state==1 else min(peak,e)
        if state:extrema.append(peak)
        half=longest=0
        for left,right in zip(extrema,extrema[1:]):
            half=half+1 if abs(right-left)>=amplitude else 0;longest=max(longest,half)
        full=longest//2;p2p=max(r['error_rad'] for r in block)-min(r['error_rad'] for r in block)
        passed=full>=cycles and p2p>=amplitude;streak=streak+1 if passed else 0
        end=block[-1]['time_s']+dt_s
        windows.append(dict(start_s=block[0]['time_s'],end_s=end,peak_to_peak_rad=p2p,
            qualified_full_cycles=full,qualifies=passed,consecutive_qualifying_windows=streak,extrema_rad=extrema))
        if streak>=required and trigger is None:trigger=end
    return dict(status='detected' if trigger is not None else 'not_detected' if windows else 'insufficient_window',
        triggered=trigger is not None,trigger_time_s=trigger,analysis_start_s=start_s,windows=windows,rule=rule)


def saturation(rows,config):
    """Return complete clipping intervals and final-release recovery from raw rows."""
    p=config['pid'];dt=config['dt_s'];lo=p['output_min'];hi=p['output_max']
    indices=[i for i,r in enumerate(rows) if r['total_raw']<lo or r['total_raw']>hi]
    intervals=[]
    for index in indices:
        if not intervals or index!=intervals[-1][-1]+1:intervals.append([index])
        else:intervals[-1].append(index)
    events=[]
    for group in intervals:
        first,last=group[0],group[-1];after=rows[last+1] if last+1<len(rows) else None
        events.append(dict(start_s=rows[first]['time_s'],last_clipped_s=rows[last]['time_s'],
            release_s=after['time_s'] if after else None,duration_s=len(group)*dt,
            request_min_nm=min(rows[i]['total_raw'] for i in group),request_max_nm=max(rows[i]['total_raw'] for i in group),
            integral_at_start_nm=rows[first]['integral'],integral_at_last_clip_nm=rows[last]['integral'],
            integral_at_release_nm=after['integral'] if after else None,error_at_release_rad=after['error_rad'] if after else None))
    release_index=indices[-1]+1 if indices else None
    return dict(request_exceeded_count=len(indices),actual_clipped_count=sum(bool(r['output_limited']) for r in rows),
        actual_limit_violation_count=sum(r['output']<lo-1e-12 or r['output']>hi+1e-12 for r in rows),
        clipping_mismatch_count=sum(abs(r['output']-min(hi,max(lo,r['total_raw'])))>1e-10 for r in rows),
        request_peak_abs_nm=max(abs(r['total_raw']) for r in rows),actual_peak_abs_nm=max(abs(r['output']) for r in rows),
        integral_peak_abs_nm=max(abs(r['integral']) for r in rows),
        integral_limit_violation_count=sum(r['integral']<p['integral_min']-1e-12 or r['integral']>p['integral_max']+1e-12 for r in rows),
        integral_limited_during_clip_count=sum(bool(rows[i]['integral_limited']) for i in indices),
        integral_windup_direction_violation_count=sum(
            (rows[i]['integral']-(rows[i-1]['integral'] if i else 0.))*(1 if rows[i]['total_raw']>hi else -1)>1e-10 for i in indices),
        longest_clipped_interval_s=max((len(x)*dt for x in intervals),default=0.),
        final_release_s=rows[release_index]['time_s'] if release_index is not None and release_index<len(rows) else None,
        intervals=events)
