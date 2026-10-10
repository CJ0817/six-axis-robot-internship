#include "robot_planning.h"
#include <math.h>
#include <string.h>

/* 用归一化路径s(t)同步六轴：q_i=start_i+delta_i*s。
 * 由各活动轴的vmax/|delta|、amax/|delta|取最小标量限值。 */
int robot_plan_joint(int method, const double *start, const double *goal,
                     const double *lower, const double *upper,
                     const double *vmax, const double *amax, size_t count,
                     double requested_s, int stretch, double sample_s,
                     double max_s, robot_joint_plan *out)
{
    robot_joint_plan p;
    double v=INFINITY, a=INFINITY, minimum=0.0, duration, geometry_duration;
    int moving=0;
    if (!start || !goal || !lower || !upper || !vmax || !amax || !out || count!=6)
        return 1001;
    if (method<1 || method>3) return 1008;
    if (stretch!=0 && stretch!=1) return 1001;
    if (!isfinite(requested_s) || requested_s<0.0 || !isfinite(sample_s) || sample_s<=0.0 ||
        !isfinite(max_s) || max_s<sample_s || (requested_s>0.0 && requested_s<sample_s))
        return 1006;
    memset(&p,0,sizeof p); p.method=method;
    for (size_t i=0;i<6;++i) {
        if (!isfinite(start[i]) || !isfinite(goal[i]) || !isfinite(lower[i]) ||
            !isfinite(upper[i]) || lower[i]>=upper[i] || !isfinite(vmax[i]) ||
            !isfinite(amax[i]) || vmax[i]<=0.0 || amax[i]<=0.0) return 1001;
        if (start[i]<lower[i] || start[i]>upper[i] || goal[i]<lower[i] || goal[i]>upper[i])
            return 1005;
        p.q_start[i]=start[i]; p.delta[i]=goal[i]-start[i];
        if (!isfinite(p.delta[i])) return 1001;
        if (p.delta[i]!=0.0) {
            moving=1;
            v=fmin(v,vmax[i]/fabs(p.delta[i])); a=fmin(a,amax[i]/fabs(p.delta[i]));
        }
    }
    if (moving) {
        if (!isfinite(v) || !isfinite(a) || v<=0.0 || a<=0.0) return 1001;
        if (method==1) minimum=fmax(1.5/v,sqrt(6.0/a));
        else if (method==2) minimum=fmax(1.875/v,sqrt((10.0/sqrt(3.0))/a));
        else {
            /* 总归一化行程1；v²/a >=1无匀速段，短行程三角形退化。 */
            p.ramp_s=fmin(v/a,1.0/sqrt(a));
            p.cruise_s=fmax(0.0,1.0/v-p.ramp_s);
            minimum=2.0*p.ramp_s+p.cruise_s;
        }
    }
    if (!isfinite(minimum)) return 3001;
    geometry_duration=minimum;
    /* 发布可实际执行的最短时长，包含浮点舍入裕度；strict可复用该值。 */
    if (moving) minimum*=1.0+1e-12;
    p.minimum_duration_s=minimum;
    if (requested_s>0.0 && requested_s<minimum && !stretch) return 3002;
    duration=fmax(sample_s,minimum);
    if (requested_s>0.0) duration=fmax(duration,requested_s);
    if (!isfinite(duration) || duration>max_s) return 3001;
    if (method==3 && moving) {
        double factor=duration/geometry_duration;
        p.ramp_s*=factor; p.cruise_s*=factor;
    }
    p.duration_s=duration;
    *out=p;
    return 0;
}

/* 三次/五次使用归一化时间u，导数显式除以T和T²。
 * 梯形按加速/匀速/减速分别解析求值，避免差分噪声。 */
int robot_joint_evaluate(const robot_joint_plan *p, double t,
                         double *q, double *qd, double *qdd, size_t count)
{
    double s=0.0, sd=0.0, sdd=0.0, u, T;
    double position[6], velocity[6], acceleration[6];
    if (!p || !q || !qd || !qdd || count!=6 || p->method<1 || p->method>3 ||
        !isfinite(p->duration_s) || p->duration_s<=0.0 ||
        !isfinite(p->ramp_s) || p->ramp_s<0.0 ||
        !isfinite(p->cruise_s) || p->cruise_s<0.0) return 1001;
    T=p->duration_s;
    if (!isfinite(t) || t<0.0 || t>T) return 1006;
    for (size_t i=0;i<6;++i)
        if (!isfinite(p->q_start[i]) || !isfinite(p->delta[i])) return 1001;
    u=t/T;
    if (p->method==1) {
        s=u*u*(3.0-2.0*u); sd=6.0*u*(1.0-u)/T; sdd=(6.0-12.0*u)/(T*T);
    } else if (p->method==2) {
        s=u*u*u*(10.0+u*(-15.0+6.0*u));
        sd=30.0*u*u*(1.0-u)*(1.0-u)/T;
        sdd=60.0*u*(1.0-u)*(1.0-2.0*u)/(T*T);
    } else if (p->ramp_s>0.0) {
        double ramp=p->ramp_s, cruise=p->cruise_s, peak, acc;
        if (!isfinite(ramp) || !isfinite(cruise) || cruise<0.0 ||
            fabs(2.0*ramp+cruise-T)>1e-10*fmax(1.0,T)) return 1001;
        peak=1.0/(ramp+cruise); acc=peak/ramp;
        if (t<ramp) { s=0.5*acc*t*t; sd=acc*t; sdd=acc; }
        else if (t<ramp+cruise) { s=0.5*peak*ramp+peak*(t-ramp); sd=peak; }
        else { double left=T-t; s=1.0-0.5*acc*left*left; sd=acc*left; sdd=-acc; }
    } else {
        for (size_t i=0;i<6;++i) if (p->delta[i]!=0.0) return 1001;
    }
    if (t==0.0) { s=0.0; sd=0.0; }
    if (t==T) { s=1.0; sd=0.0; }
    for (size_t i=0;i<6;++i) {
        position[i]=p->q_start[i]+p->delta[i]*s;
        velocity[i]=p->delta[i]*sd; acceleration[i]=p->delta[i]*sdd;
        if (!isfinite(position[i]) || !isfinite(velocity[i]) || !isfinite(acceleration[i])) return 1001;
    }
    memcpy(q,position,sizeof position); memcpy(qd,velocity,sizeof velocity);
    memcpy(qdd,acceleration,sizeof acceleration);
    return 0;
}
