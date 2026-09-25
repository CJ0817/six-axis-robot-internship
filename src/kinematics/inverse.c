#define _POSIX_C_SOURCE 200809L
#include "robot_kinematics.h"
#include <math.h>
#include <string.h>
#include <time.h>

/* Standard-DH UR5 CB, immutable ABI v1. No allocation or global state. */
static const double PI=3.14159265358979323846;
static double now(void) {
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC,&t)) return -1;
    return (double)t.tv_sec+1e-9*(double)t.tv_nsec;
}
static int finite_values(const double *p,size_t n) {
    for(size_t i=0;i<n;i++) if(!isfinite(p[i])) return 0;
    return 1;
}
static double clip(double x) { return fmax(-1.,fmin(1.,x)); }
static int lex_less(const double *a,const double *b) {
    for(int i=0;i<6;i++) { if(a[i]<b[i]) return 1; if(a[i]>b[i]) return 0; }
    return 0;
}
static double distance(const double *a,const double *b) {
    double d=0;
    for(int i=0;i<6;i++) d=hypot(d,a[i]-b[i]);
    return d;
}
static void residual(const double *a,const double *b,double *p,double *r) {
    *p=hypot(hypot(a[3]-b[3],a[7]-b[7]),a[11]-b[11]);
    double R[9]={0};
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        for(int k=0;k<3;k++) R[3*i+j]+=a[4*k+i]*b[4*k+j];
    double s=.5*hypot(hypot(R[7]-R[5],R[2]-R[6]),R[3]-R[1]);
    *r=atan2(s,clip(.5*(R[0]+R[4]+R[8]-1)));
}
static int lifted(const robot_fk_model *m,const double *t,const double *seed,double margin,double *q) {
    for(int i=0;i<6;i++) {
        double base=remainder((t[i]-m->theta_offset_rad[i])/m->joint_sign[i],2*PI);
        double lo=m->q_min_rad[i]+margin,hi=m->q_max_rad[i]-margin;
        /* Only roundoff-sized endpoint overshoot may be snapped; verify FK later. */
        double k0=ceil((lo-base-1e-12)/(2*PI)),k1=floor((hi-base+1e-12)/(2*PI));
        if(k0>k1) return 0;
        double k=fmax(k0,fmin(k1,floor((seed[i]-base)/(2*PI))));
        double a=base+2*PI*k,b=base+2*PI*fmin(k+1,k1);
        q[i]=fabs(a-seed[i])<=fabs(b-seed[i])?a:b;
        if(q[i]<lo && lo-q[i]<=1e-12) q[i]=lo;
        if(q[i]>hi && q[i]-hi<=1e-12) q[i]=hi;
        if(!isfinite(q[i]) || q[i]<lo || q[i]>hi) return 0;
    }
    return 1;
}
static int supported(const robot_fk_model *m) {
    const double a[6]={0,-.425,-.39225,0,0,0};
    const double d[6]={.089159,0,0,.10915,.09465,.0823};
    const double al[6]={1.5707963267948966,0,0,1.5707963267948966,-1.5707963267948966,0};
    for(int i=0;i<6;i++) {
        if(fabs(m->a_m[i]-a[i])>1e-12 || fabs(m->d_m[i]-d[i])>1e-12 || fabs(m->alpha_rad[i]-al[i])>1e-12) return 0;
        /* Beyond this range float64 phase/endpoint resolution is not guaranteed. */
        if(fabs(m->theta_offset_rad[i])>1e6 || fabs(m->q_min_rad[i])>1e6 || fabs(m->q_max_rad[i])>1e6) return 0;
    }
    return 1;
}
int robot_inverse_v1(const robot_fk_model *m,const double *target,size_t pose_count,
                     const double *seed,size_t seed_count,const robot_ik_options_v1 *o,
                     robot_ik_result_v1 *out) {
    double start=now();
    if(!target || !seed || !o || !out || pose_count!=16 || seed_count!=6) return 1001;
    if(!m) return 1004;
    if(!finite_values(target,16) || !finite_values(seed,6)) return 1001;
    double seedT[16],inv[16],T[16];
    int code=robot_forward(m,seed,6,seedT); /* full shared model validation */
    if(code!=0 && code!=1005) return code;
    if(robot_transform_inverse(target,m->pose_validation_tol,inv)) return 1001;
    const double opts[7]={o->position_tol_m,o->orientation_tol_rad,o->joint_margin_rad,o->timeout_s,
                          o->characteristic_length_m,o->damping,o->max_step_rad};
    if(!finite_values(opts,7) || o->position_tol_m<=0 || o->orientation_tol_rad<=0 ||
       o->orientation_tol_rad>PI || o->joint_margin_rad<0 || o->timeout_s<=0) return 1001;
    if(o->method==ROBOT_IK_ANALYTIC_UR5_V1 && (o->max_iterations || o->line_search_max_steps ||
       o->characteristic_length_m!=0 || o->damping!=0 || o->max_step_rad!=0)) return 1001;
    if(o->method==ROBOT_IK_DLS_V1 && (!o->max_iterations || !o->line_search_max_steps ||
       o->characteristic_length_m<=0 || o->damping<=0 || o->max_step_rad<=0)) return 1001;
    for(int i=0;i<6;i++) {
        double lo=m->q_min_rad[i]+o->joint_margin_rad,hi=m->q_max_rad[i]-o->joint_margin_rad;
        if(!isfinite(lo) || !isfinite(hi) || lo>hi) return 1001;
    }
    if(code==1005) return 1005;
    for(int i=0;i<6;i++) if(seed[i]<m->q_min_rad[i]+o->joint_margin_rad || seed[i]>m->q_max_rad[i]-o->joint_margin_rad) return 1005;
    if(o->method!=ROBOT_IK_ANALYTIC_UR5_V1 || o->branch_policy>1 || !supported(m)) return 1008;
    if(start<0) return 9000;
    #define BUDGET() do { double n=now(); if(n<0) return 9000; if(n-start>=o->timeout_s) return 2002; } while(0)
    BUDGET();
    /* inv(B) target inv(G) inv(F), never assume fixed transforms cancel. */
    if(robot_transform_inverse(m->T_base_dh0,m->pose_validation_tol,inv) || robot_mat4_multiply(inv,target,T) ||
       robot_transform_inverse(m->T_flange_tool,m->pose_validation_tol,inv) || robot_mat4_multiply(T,inv,T) ||
       robot_transform_inverse(m->T_dh6_flange,m->pose_validation_tol,inv) || robot_mat4_multiply(T,inv,T)) return 2002;
    double reach=0;
    for(int i=0;i<6;i++) reach+=fabs(m->a_m[i])+fabs(m->d_m[i]);
    if(hypot(hypot(T[3],T[7]),T[11])>reach+1e-12) return 2001;
    double X=T[3]-m->d_m[5]*T[2],Y=T[7]-m->d_m[5]*T[6],rho=hypot(X,Y),d4=m->d_m[3];
    if(!isfinite(rho)) return 2002;
    if(rho<d4-1e-12) return 2001;
    double beta=asin(fmin(1.,d4/rho)),az=atan2(Y,X),shoulder[2]={az+beta,az+PI-beta};
    robot_ik_result_v1 result={0};
    for(int i=0;i<8;i++) result.branch_ids[i]=UINT32_MAX;
    int geometric=0,singular=0,numerical=0;
    for(int s=0;s<2;s++) {
        double t1=shoulder[s],h0=sin(t1),h1=-cos(t1);
        double u=h0*T[0]+h1*T[4],v=h0*T[1]+h1*T[5],c=h0*T[2]+h1*T[6],mag=hypot(u,v);
        if(mag<=1e-12) { singular=1; continue; }
        for(int w=-1;w<=1;w+=2) {
            BUDGET();
            double t5=atan2(w*mag,c),t6=atan2(-w*v,w*u),U[16],A[16];
            int indices[3]={0,5,4}; double angles[3]={t1,t6,t5};
            memcpy(U,T,sizeof(U));
            for(int j=0;j<3;j++) {
                int i=indices[j];
                if(robot_dh_transform(m->a_m[i],m->alpha_rad[i],m->d_m[i],angles[j],A) ||
                   robot_transform_inverse(A,m->pose_validation_tol,inv)) return 9000;
                code=j==0?robot_mat4_multiply(inv,U,U):robot_mat4_multiply(U,inv,U);
                if(code) return 2002;
            }
            double x=U[3],y=U[7],a2=m->a_m[1],a3=m->a_m[2];
            double c3=(x*x+y*y-a2*a2-a3*a3)/(2*a2*a3);
            if(!isfinite(c3)) { numerical=1; continue; }
            if(fabs(c3)>1+1e-12) continue;
            geometric=1;
            for(int e=-1;e<=1;e+=2) {
                double t3=e*acos(clip(c3)),t2=atan2(y,x)-atan2(a3*sin(t3),a2+a3*cos(t3));
                double t[6]={t1,t2,t3,atan2(U[4],U[0])-t2-t3,t5,t6},q[6],back[16],pe,re;
                if(!lifted(m,t,seed,o->joint_margin_rad,q)) continue;
                if(robot_forward(m,q,6,back)) { numerical=1; continue; }
                residual(back,target,&pe,&re);
                if(!isfinite(pe) || !isfinite(re) || pe>o->position_tol_m || re>o->orientation_tol_rad) { numerical=1; continue; }
                int duplicate=0;
                for(uint32_t k=0;k<result.solution_count;k++) {
                    double max=0;for(int j=0;j<6;j++) max=fmax(max,fabs(q[j]-result.candidates_rad[k][j]));
                    if(max<=1e-9) duplicate=1;
                }
                if(!duplicate) {
                    if(result.solution_count==8) return 9000;
                    memcpy(result.candidates_rad[result.solution_count++],q,sizeof(q));
                }
            }
        }
    }
    BUDGET();
    if(singular) {
        /* A matching seed is the provably closest representative (distance zero).
         * Other singular nearest/all requests require a continuum solver. */
        double pe,re; residual(seedT,target,&pe,&re);
        if(o->branch_policy!=0 || pe>1e-12 || re>1e-12 || pe>o->position_tol_m || re>o->orientation_tol_rad) return 2003;
        memset(&result,0,sizeof(result));
        for(int i=0;i<8;i++) result.branch_ids[i]=UINT32_MAX;
        result.solution_count=1;memcpy(result.candidates_rad[0],seed,6*sizeof(double));
    } else if(numerical) return 2002; /* Never report a partial enumeration as all. */
    if(!result.solution_count) return geometric?1005:2001;
    for(uint32_t i=1;i<result.solution_count;i++) for(uint32_t j=i;j>0 && lex_less(result.candidates_rad[j],result.candidates_rad[j-1]);j--) {
        double tmp[6];memcpy(tmp,result.candidates_rad[j],sizeof(tmp));
        memcpy(result.candidates_rad[j],result.candidates_rad[j-1],sizeof(tmp));memcpy(result.candidates_rad[j-1],tmp,sizeof(tmp));
    }
    uint32_t best=0;
    for(uint32_t i=1;i<result.solution_count;i++) if(distance(result.candidates_rad[i],seed)<distance(result.candidates_rad[best],seed)) best=i;
    memcpy(result.q_rad,result.candidates_rad[best],sizeof(result.q_rad));
    if(o->branch_policy==0) {
        memset(result.candidates_rad,0,sizeof(result.candidates_rad));
        memcpy(result.candidates_rad[0],result.q_rad,sizeof(result.q_rad));result.solution_count=1;best=0;
    }
    result.selected_index=best;
    for(uint32_t i=0;i<result.solution_count;i++) result.branch_ids[i]=i;
    double back[16]; if(robot_forward(m,result.q_rad,6,back)) return 9000;
    residual(back,target,&result.position_error_m,&result.orientation_error_rad);
    BUDGET();result.elapsed_s=now()-start;
    if(!isfinite(result.elapsed_s) || result.elapsed_s<0) return 9000;
    if(result.elapsed_s>=o->timeout_s) return 2002;
    memcpy(out,&result,sizeof(result));return 0;
    #undef BUDGET
}
