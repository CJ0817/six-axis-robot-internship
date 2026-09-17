#include "robot_kinematics.h"
#include <math.h>
#include <string.h>

static int finite_array(const double *a, size_t n) {
    if (!a) return 0;
    for (size_t i=0; i<n; ++i) if (!isfinite(a[i])) return 0;
    return 1;
}
static int rigid(const double *t, double tol) {
    if (!finite_array(t,16) || !isfinite(tol) || tol<=0 || tol>1e-6) return 0;
    for (size_t j=0; j<4; ++j) if (fabs(t[12+j]-(j==3?1.0:0.0))>tol) return 0;
    double norm=0;
    for (size_t i=0; i<3; ++i) for (size_t j=0; j<3; ++j) {
        double e=0;
        for (size_t k=0; k<3; ++k) e+=t[4*k+i]*t[4*k+j];
        e-=(i==j?1.0:0.0); norm+=e*e;
    }
    double det=t[0]*(t[5]*t[10]-t[6]*t[9])-t[1]*(t[4]*t[10]-t[6]*t[8])+t[2]*(t[4]*t[9]-t[5]*t[8]);
    return isfinite(norm) && sqrt(norm)<=tol && isfinite(det) && det>0 && fabs(det-1)<=tol;
}
int robot_mat4_identity(double *out) {
    if (!out) return 1001;
    const double v[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    memcpy(out,v,sizeof(v)); return 0;
}
int robot_mat4_multiply(const double *a, const double *b, double *out) {
    if (!out || !finite_array(a,16) || !finite_array(b,16)) return 1001;
    double v[16]={0};
    for (size_t i=0;i<4;++i) for (size_t j=0;j<4;++j)
        for (size_t k=0;k<4;++k) v[4*i+j]+=a[4*i+k]*b[4*k+j];
    if (!finite_array(v,16)) return 1001;
    memcpy(out,v,sizeof(v)); return 0;
}
int robot_mat4_transpose(const double *a, double *out) {
    if (!out || !finite_array(a,16)) return 1001;
    double v[16];
    for (size_t i=0;i<4;++i) for (size_t j=0;j<4;++j) v[4*i+j]=a[4*j+i];
    memcpy(out,v,sizeof(v)); return 0;
}
int robot_transform_inverse(const double *t, double tol, double *out) {
    if (!out || !rigid(t,tol)) return 1001;
    double v[16]={0}; v[15]=1;
    for (size_t i=0;i<3;++i) {
        for (size_t j=0;j<3;++j) { v[4*i+j]=t[4*j+i]; v[4*i+3]-=t[4*j+i]*t[4*j+3]; }
    }
    if (!finite_array(v,16)) return 1001;
    memcpy(out,v,sizeof(v)); return 0;
}
int robot_dh_transform(double a,double alpha,double d,double theta,double *out) {
    if (!out || !isfinite(a) || !isfinite(alpha) || !isfinite(d) || !isfinite(theta)) return 1001;
    double c=cos(theta),s=sin(theta),ca=cos(alpha),sa=sin(alpha);
    const double v[16]={c,-s*ca,s*sa,a*c,s,c*ca,-c*sa,a*s,0,sa,ca,d,0,0,0,1};
    if (!finite_array(v,16)) return 1001;
    memcpy(out,v,sizeof(v)); return 0;
}
int robot_forward(const robot_fk_model *m,const double *q,size_t count,double *out) {
    if (!out || count!=6 || !finite_array(q,6)) return 1001;
    if (!m) return 1004;
    if (!finite_array(m->a_m,6) || !finite_array(m->alpha_rad,6) || !finite_array(m->d_m,6) ||
        !finite_array(m->theta_offset_rad,6) || !finite_array(m->q_min_rad,6) || !finite_array(m->q_max_rad,6) ||
        !rigid(m->T_base_dh0,m->pose_validation_tol) || !rigid(m->T_dh6_flange,m->pose_validation_tol) ||
        !rigid(m->T_flange_tool,m->pose_validation_tol)) return 1001;
    for (size_t i=0;i<6;++i)
        if ((m->joint_sign[i]!=1 && m->joint_sign[i]!=-1) || m->q_min_rad[i]>=m->q_max_rad[i]) return 1001;
    for (size_t i=0;i<6;++i) if(q[i]<m->q_min_rad[i] || q[i]>m->q_max_rad[i]) return 1005;
    double t[16],a[16]; memcpy(t,m->T_base_dh0,sizeof(t));
    for (size_t i=0;i<6;++i) {
        int code=robot_dh_transform(m->a_m[i],m->alpha_rad[i],m->d_m[i],m->joint_sign[i]*q[i]+m->theta_offset_rad[i],a);
        if (code) return code;
        code=robot_mat4_multiply(t,a,t); if (code) return code;
    }
    int code=robot_mat4_multiply(t,m->T_dh6_flange,t); if (code) return code;
    code=robot_mat4_multiply(t,m->T_flange_tool,t); if (code) return code;
    memcpy(out,t,sizeof(t)); return 0;
}
