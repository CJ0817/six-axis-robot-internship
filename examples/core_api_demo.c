/* Complete standalone C usage. This prints numeric references, never enables a robot. */
#include "robot_kinematics.h"
#include "robot_planning.h"
#include "robot_control.h"
#include <math.h>
#include <stdio.h>

/* Create an explicit nominal standard-DH fixture; production loads its audited model. */
static void model_fixture(robot_fk_model *m) {
    const double pi = acos(-1.0);
    const double a[6] = {0,-.425,-.39225,0,0,0};
    const double d[6] = {.089159,0,0,.10915,.09465,.0823};
    const double alpha[6] = {pi/2,0,0,pi/2,-pi/2,0};
    size_t i;
    for (i=0;i<6;i++) {
        m->a_m[i]=a[i]; m->d_m[i]=d[i]; m->alpha_rad[i]=alpha[i];
        m->joint_sign[i]=1; m->q_min_rad[i]=-2*pi; m->q_max_rad[i]=2*pi;
    }
    (void)robot_mat4_identity(m->T_base_dh0);
    (void)robot_mat4_identity(m->T_dh6_flange);
    (void)robot_mat4_identity(m->T_flange_tool);
    m->pose_validation_tol=1e-9;
}

/* Run FK -> IK, joint interpolation and PID+velocity feedforward, checking every status. */
int main(void) {
    robot_fk_model model={0};
    const double seed[6]={.3,-1.2,1.1,-1.1,1.,.2};
    double target[16], goal[6], vmax[6], amax[6], q[6], v[6], a[6];
    robot_ik_options_v1 options={0};
    robot_ik_result_v1 inverse={0};
    robot_joint_plan plan={0};
    robot_pid_config config={.02,4.,.2,0.,-.01,.01,-.05,.05};
    robot_pid_state state={0};
    robot_pid_ff_result result={0};
    size_t i;
    model_fixture(&model);
    options.method=ROBOT_IK_ANALYTIC_UR5_V1;
    options.branch_policy=ROBOT_IK_NEAREST_SEED_V1;
    options.position_tol_m=1e-5; options.orientation_tol_rad=1e-4;
    options.timeout_s=.5;
    if (robot_abi_version()!=ROBOT_ABI_VERSION_V1) return 1;
    if (robot_forward(&model,seed,6,target)) return 2;
    if (robot_inverse_v1(&model,target,16,seed,6,&options,&inverse)) return 3;
    for (i=0;i<6;i++) { goal[i]=seed[i]+.01; vmax[i]=.05; amax[i]=.5; }
    if (robot_plan_joint(2,seed,goal,model.q_min_rad,model.q_max_rad,vmax,amax,6,4.,0,.02,30.,&plan)) return 4;
    if (robot_joint_evaluate(&plan,.02,q,v,a,6)) return 5;
    if (robot_pid_reset(&config,&state)) return 6;
    if (robot_pid_step_ff(&state,q[0],seed[0],v[0],a[0],1.,0.,&result)) return 7;
    if (!isfinite(result.output) || fabs(result.output)>.05) return 8;
    /* Missing model is rejected and the output buffer is preserved. */
    {
        double sentinel[16];
        for (i=0;i<16;i++) sentinel[i]=123.;
        if (robot_forward(NULL,seed,6,sentinel)!=1004) return 9;
        for (i=0;i<16;i++) if (sentinel[i]!=123.) return 10;
    }
    printf("{\"code\":0,\"abi\":%u,\"ik_solutions\":%u,\"pid_velocity_rad_s\":%.12g,\"executable\":false}\n",
           (unsigned)robot_abi_version(),(unsigned)inverse.solution_count,result.output);
    return 0;
}
