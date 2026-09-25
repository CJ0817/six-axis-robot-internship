/* Include the implementation to test the exact private production filter.
 * This executable does not link another inverse.c or export new library ABI. */
#include "../../src/kinematics/inverse.c"
#include <stdio.h>
#include <float.h>

static int checks=0,failed=0;
static void check(const char *name,int ok) {
    checks++;failed+=!ok;
    printf("%s: %s\n",name,ok?"passed":"FAILED");
}
static void trial(const char *name,const robot_fk_model *m,const double *t,const double *seed,
                  const double *target,const robot_ik_options_v1 *o,robot_ik_result_v1 *r,
                  enum candidate_status expected) {
    robot_ik_result_v1 before=*r;
    enum candidate_status actual=filter_candidate(m,t,seed,target,o,r);
    check(name,actual==expected && (actual==CANDIDATE_ACCEPTED || memcmp(&before,r,sizeof(before))==0));
}
int main(void) {
    robot_fk_model m={0};
    const double a[6]={0,-.425,-.39225,0,0,0},d[6]={.089159,0,0,.10915,.09465,.0823};
    const double al[6]={1.5707963267948966,0,0,1.5707963267948966,-1.5707963267948966,0};
    memcpy(m.a_m,a,sizeof(a));memcpy(m.d_m,d,sizeof(d));memcpy(m.alpha_rad,al,sizeof(al));
    for(int i=0;i<6;i++) {m.joint_sign[i]=1;m.q_min_rad[i]=-2*PI;m.q_max_rad[i]=2*PI;}
    m.q_min_rad[2]=-PI;m.q_max_rad[2]=PI;m.pose_validation_tol=1e-9;
    robot_mat4_identity(m.T_base_dh0);robot_mat4_identity(m.T_dh6_flange);robot_mat4_identity(m.T_flange_tool);
    const double q[6]={.2,-.6,.8,-.5,.4,-.2};double target[16],bad[6];
    robot_forward(&m,q,6,target);
    robot_ik_options_v1 o={1,1,1e-5,1e-4,0,.2,0,0,0,0,0};
    robot_ik_result_v1 r={0};
    memcpy(bad,q,sizeof(bad));bad[1]+=.1;
    trial("position_residual_rejected",&m,bad,q,target,&o,&r,CANDIDATE_RESIDUAL);
    memcpy(bad,q,sizeof(bad));bad[5]+=.1;
    trial("orientation_only_residual_rejected",&m,bad,q,target,&o,&r,CANDIDATE_RESIDUAL);
    trial("valid_survives_bad_predecessors",&m,q,q,target,&o,&r,CANDIDATE_ACCEPTED);
    trial("exact_duplicate",&m,q,q,target,&o,&r,CANDIDATE_DUPLICATE);
    memcpy(bad,q,sizeof(bad));bad[0]+=2*PI;
    trial("two_pi_duplicate_after_lift",&m,bad,q,target,&o,&r,CANDIDATE_DUPLICATE);
    memcpy(bad,q,sizeof(bad));bad[5]+=5e-10;
    trial("duplicate_inside_1e_9",&m,bad,q,target,&o,&r,CANDIDATE_DUPLICATE);
    memcpy(bad,q,sizeof(bad));bad[5]+=2e-9;
    trial("distinct_outside_1e_9",&m,bad,q,target,&o,&r,CANDIDATE_ACCEPTED);
    memcpy(bad,q,sizeof(bad));bad[0]=NAN;
    trial("nan_candidate",&m,bad,q,target,&o,&r,CANDIDATE_NONFINITE);
    bad[0]=INFINITY;
    trial("infinite_candidate",&m,bad,q,target,&o,&r,CANDIDATE_NONFINITE);
    robot_fk_model narrow=m;narrow.q_min_rad[0]=-.01;narrow.q_max_rad[0]=.01;
    double seed[6];memcpy(seed,q,sizeof(seed));seed[0]=0;
    trial("no_legal_two_pi_lift",&narrow,q,seed,target,&o,&r,CANDIDATE_LIMIT);
    robot_ik_options_v1 margin=o;margin.joint_margin_rad=.1;
    narrow=m;narrow.q_min_rad[0]=.2;narrow.q_max_rad[0]=.8;seed[0]=.4;
    trial("margin_excludes_endpoint",&narrow,q,seed,target,&margin,&r,CANDIDATE_LIMIT);
    check("mixed_stream_has_two_valid_representatives",r.solution_count==2);
    /* Compute residual spread from the public output, then place tolerance
     * between extremes: the old global-numerical flag rejected all of them. */
    robot_ik_result_v1 all;
    int code=robot_inverse_v1(&m,target,16,q,6,&o,&all);
    check("public_all_baseline",code==0 && all.solution_count>1);
    if(code==0) {
        double low=DBL_MAX,high=0,minp=DBL_MAX;
        for(uint32_t i=0;i<all.solution_count;i++) {
            double back[16],p,e;robot_forward(&m,all.candidates_rad[i],6,back);residual(back,target,&p,&e);
            low=fmin(low,e);high=fmax(high,e);minp=fmin(minp,p);
        }
        robot_ik_options_v1 strict=o;strict.orientation_tol_rad=(low+high)/2;
        robot_ik_result_v1 kept;
        code=robot_inverse_v1(&m,target,16,q,6,&strict,&kept);
        check("public_partial_residual_filter",high>low && code==0 && kept.solution_count>0 && kept.solution_count<all.solution_count);
        if(code==0) {
            int valid=1;
            for(uint32_t i=0;i<kept.solution_count;i++) {
                double back[16],p,e;robot_forward(&m,kept.candidates_rad[i],6,back);residual(back,target,&p,&e);
                if(p>strict.position_tol_m || e>strict.orientation_tol_rad) valid=0;
            }
            check("every_retained_candidate_passes",valid);
            printf("retained=%u / initial=%u; orientation_tol=%.17g rad\n",kept.solution_count,all.solution_count,strict.orientation_tol_rad);
        }
        if(minp>0) {
            strict=o;strict.position_tol_m=minp/2;
            memset(&kept,0x5A,sizeof(kept));robot_ik_result_v1 before=kept;
            code=robot_inverse_v1(&m,target,16,q,6,&strict,&kept);
            check("all_residual_rejected_2002_output_unchanged",code==2002 && memcmp(&kept,&before,sizeof(kept))==0);
        } else check("nonzero_residual_fixture_required",0);
    }
    printf("checks=%d failed=%d\n",checks,failed);
    return failed?1:0;
}
