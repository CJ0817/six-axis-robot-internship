#include "robot_kinematics.h"
#include <math.h>
#include <string.h>

/* 检查数组指针和数值有限性。参数：a数组、n元素数；返回1合法，0空指针或非有限。 */
static int finite_array(const double *a, size_t n) {
    if (!a) return 0;
    for (size_t i=0; i<n; ++i) if (!isfinite(a[i])) return 0;
    return 1;
}
/* 校验刚体齐次矩阵。参数：t[16]和tol；返回1满足末行、正交性和正行列式约束，否则0。 */
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
/* 生成4×4单位矩阵。
 * 参数：out为可写的16个double，按行存储。
 * 返回：0成功；out为空返回1001，失败不写输出。 */
int robot_mat4_identity(double *out) {
    if (!out) return 1001;
    const double v[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    memcpy(out,v,sizeof(v)); return 0;
}
/* 计算两个4×4矩阵的乘积a·b。
 * 参数：a、b各为16个有限double；out提供16个可写double，可与输入重叠。
 * 返回：0成功；参数或计算结果非有限返回1001，失败不写输出。 */
int robot_mat4_multiply(const double *a, const double *b, double *out) {
    if (!out || !finite_array(a,16) || !finite_array(b,16)) return 1001;
    double v[16]={0};
    for (size_t i=0;i<4;++i) for (size_t j=0;j<4;++j)
        for (size_t k=0;k<4;++k) v[4*i+j]+=a[4*i+k]*b[4*k+j];
    if (!finite_array(v,16)) return 1001;
    memcpy(out,v,sizeof(v)); return 0;
}
/* 转置4×4矩阵，支持原地操作。
 * 参数：a为16个有限double；out为16个可写double。
 * 返回：0成功；非法参数返回1001，失败不写输出。 */
int robot_mat4_transpose(const double *a, double *out) {
    if (!out || !finite_array(a,16)) return 1001;
    double v[16];
    for (size_t i=0;i<4;++i) for (size_t j=0;j<4;++j) v[4*i+j]=a[4*j+i];
    memcpy(out,v,sizeof(v)); return 0;
}
/* 计算刚体齐次变换的逆：[R转置,-R转置·p;0,1]。
 * 参数：t为合法4×4刚体矩阵；tol为(0,1e-6]校验容差；out为16个double。
 * 返回：0成功；非法矩阵/指针/容差返回1001。不能用于任意4×4矩阵求逆。 */
int robot_transform_inverse(const double *t, double tol, double *out) {
    if (!out || !rigid(t,tol)) return 1001;
    double v[16]={0}; v[15]=1;
    for (size_t i=0;i<3;++i) {
        for (size_t j=0;j<3;++j) { v[4*i+j]=t[4*j+i]; v[4*i+3]-=t[4*j+i]*t[4*j+3]; }
    }
    if (!finite_array(v,16)) return 1001;
    memcpy(out,v,sizeof(v)); return 0;
}
/* 按标准DH生成单个连杆的齐次变换Rz·Tz·Tx·Rx。
 * 参数：a、d为米；alpha、theta为弧度；out为16个可写double。
 * 返回：0成功；非有限数值或空输出返回1001。 */
int robot_dh_transform(double a,double alpha,double d,double theta,double *out) {
    if (!out || !isfinite(a) || !isfinite(alpha) || !isfinite(d) || !isfinite(theta)) return 1001;
    double c=cos(theta),s=sin(theta),ca=cos(alpha),sa=sin(alpha);
    const double v[16]={c,-s*ca,s*sa,a*c,s,c*ca,-c*sa,a*s,0,sa,ca,d,0,0,0,1};
    if (!finite_array(v,16)) return 1001;
    memcpy(out,v,sizeof(v)); return 0;
}
/* 从六关节角计算base→tool0位姿，依次乘DH链和显式固定变换。
 * 参数：model为纯几何模型；q_rad为J1～J6角度(rad)；count必须为6；
 * T_base_tool为16个可写double，按行存储。输入输出容量由调用方保证。
 * 返回：0成功；1004模型为空，1001非法参数/模型，1005关节越限。
 * 失败不写输出。奇异位形可正常求FK，本接口不启动设备运动。
 * 实现中的m/q/out或m/target/seed/o/out对应头文件同序参数；具体布局见robot_kinematics.h。 */
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
