#include "robot_kinematics.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"failed line %d: %s\n",__LINE__,#x); return 1; } } while(0)
static int near(const double *a,const double *b) {
    for(int i=0;i<16;i++) if(fabs(a[i]-b[i])>1e-12) return 0;
    return 1;
}
int main(void) {
    double pi=acos(-1.0),id[16],r[16],t[16],out[16],v[16],saved[16];
    CHECK(robot_mat4_identity(id)==0);
    CHECK(robot_dh_transform(0,0,0,pi/2,r)==0);
    memcpy(t,id,sizeof(t));t[3]=2;t[7]=3;t[11]=4;
    CHECK(robot_mat4_multiply(r,t,out)==0);
    CHECK(fabs(out[3]+3)<1e-12 && fabs(out[7]-2)<1e-12 && out[11]==4);
    memcpy(v,r,sizeof(v));CHECK(robot_mat4_multiply(v,t,v)==0 && near(v,out));
    memcpy(v,t,sizeof(v));CHECK(robot_mat4_multiply(r,v,v)==0 && near(v,out));
    CHECK(robot_transform_inverse(out,1e-9,v)==0);
    CHECK(robot_mat4_multiply(out,v,v)==0 && near(v,id));
    memcpy(v,out,sizeof(v));CHECK(robot_transform_inverse(v,1e-9,v)==0);
    CHECK(robot_mat4_multiply(out,v,v)==0 && near(v,id));
    CHECK(robot_mat4_transpose(t,v)==0 && v[12]==2 && v[13]==3);
    CHECK(robot_mat4_transpose(v,v)==0 && near(v,t));
    for(int i=0;i<16;i++) saved[i]=42;
    memcpy(out,saved,sizeof(out));t[0]=NAN;
    CHECK(robot_mat4_multiply(t,id,out)==1001 && near(out,saved));
    CHECK(robot_mat4_transpose(t,out)==1001 && near(out,saved));
    memcpy(t,id,sizeof(t));t[0]=2;
    CHECK(robot_transform_inverse(t,1e-9,out)==1001 && near(out,saved));
    t[0]=-1;CHECK(robot_transform_inverse(t,1e-9,out)==1001);
    CHECK(robot_mat4_identity(NULL)==1001);
    CHECK(robot_mat4_multiply(NULL,id,out)==1001);
    CHECK(robot_dh_transform(INFINITY,0,0,0,out)==1001 && near(out,saved));
    memcpy(t,id,sizeof(t));t[0]=DBL_MAX;
    CHECK(robot_mat4_multiply(t,t,out)==1001 && near(out,saved));
    robot_fk_model m={0};
    for(int i=0;i<6;i++){m.joint_sign[i]=1;m.q_min_rad[i]=-pi;m.q_max_rad[i]=pi;}
    memcpy(m.T_base_dh0,id,sizeof(id));memcpy(m.T_dh6_flange,id,sizeof(id));memcpy(m.T_flange_tool,id,sizeof(id));m.pose_validation_tol=1e-9;
    double q[6]={0};
    CHECK(robot_forward(NULL,q,6,out)==1004 && near(out,saved));
    CHECK(robot_forward(&m,q,5,out)==1001 && near(out,saved));
    CHECK(robot_forward(&m,NULL,6,out)==1001);
    CHECK(robot_forward(&m,q,6,NULL)==1001);
    q[0]=NAN;CHECK(robot_forward(&m,q,6,out)==1001 && near(out,saved));q[0]=0;
    m.joint_sign[0]=0;CHECK(robot_forward(&m,q,6,out)==1001 && near(out,saved));m.joint_sign[0]=1;
    q[0]=pi+.01;CHECK(robot_forward(&m,q,6,out)==1005 && near(out,saved));
    q[0]=pi;CHECK(robot_forward(&m,q,6,out)==0);
    q[0]=-pi;CHECK(robot_forward(&m,q,6,out)==0);q[0]=0;
    /* Independent simple chain: all alpha=0; nontrivial base, sign, offset and TCP. */
    memcpy(m.T_base_dh0,r,sizeof(r));m.T_base_dh0[3]=.1;m.T_base_dh0[7]=.2;m.T_base_dh0[11]=.3;
    m.a_m[0]=.5;m.d_m[0]=.2;m.joint_sign[0]=-1;m.theta_offset_rad[0]=pi/2;
    m.T_flange_tool[3]=.1;m.T_flange_tool[11]=.05;
    const double cases[3]={0,pi/2,-pi/2};
    const double expected[3][16]={{-1,0,0,-.5,0,-1,0,.2,0,0,1,.55,0,0,0,1},
      {0,-1,0,.1,1,0,0,.8,0,0,1,.55,0,0,0,1},
      {0,1,0,.1,-1,0,0,-.4,0,0,1,.55,0,0,0,1}};
    for(int i=0;i<3;i++){q[0]=cases[i];CHECK(robot_forward(&m,q,6,out)==0 && near(out,expected[i]));}
    puts("matrix operations, aliasing, validation, limits and 3 independent FK poses: passed");return 0;
}
