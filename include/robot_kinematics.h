#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H
#include <stddef.h>
#include <stdint.h>
#include "robot_errors.h"
#ifdef __cplusplus
extern "C" {
#endif
/* ABI v1, float64, row-major 4x4, column vectors, m/rad, J1..J6.
 * Caller owns valid buffers (16 doubles for matrices, 6 for joints).
 * All outputs remain untouched on error; matrix/FK aliasing is supported.
 * IK result must not overlap its inputs.
 * Codes are the existing contract.json codes, not a new numbering scheme. */
typedef struct {
    double a_m[6], alpha_rad[6], d_m[6], theta_offset_rad[6];
    int joint_sign[6];
    double T_base_dh0[16], T_dh6_flange[16], T_flange_tool[16];
    double q_min_rad[6], q_max_rad[6];
    double pose_validation_tol;
} robot_fk_model;
int robot_mat4_identity(double *out);
int robot_mat4_multiply(const double *a, const double *b, double *out);
int robot_mat4_transpose(const double *a, double *out);
/* Only rigid transforms, NOT a general 4x4 inverse. 0<tol<=1e-6. */
int robot_transform_inverse(const double *t, double tol, double *out);
int robot_dh_transform(double a, double alpha, double d, double theta, double *out);
/* Pure kinematic FK. NULL model:1004; invalid:1001; joint limit:1005.
 * No speed/acceleration data required; this API cannot enable motion. */
int robot_forward(const robot_fk_model *model, const double *q_rad,
                  size_t count, double *T_base_tool);
/* Frozen additive ABI 1.0.0; old FK signatures/layout remain unchanged. */
#define ROBOT_ABI_VERSION_V1 UINT32_C(0x00010000)
#define ROBOT_IK_MAX_SOLUTIONS_V1 8
#define ROBOT_IK_ANALYTIC_UR5_V1 1u
#define ROBOT_IK_DLS_V1 2u
#define ROBOT_IK_NEAREST_SEED_V1 0u
#define ROBOT_IK_ALL_V1 1u
typedef struct {
    uint32_t method, branch_policy;
    double position_tol_m, orientation_tol_rad, joint_margin_rad, timeout_s;
    uint32_t max_iterations, line_search_max_steps;
    double characteristic_length_m, damping, max_step_rad;
} robot_ik_options_v1;
typedef struct {
    double q_rad[6];
    double candidates_rad[8][6];
    uint32_t branch_ids[8];
    uint32_t solution_count, selected_index;
    double position_error_m, orientation_error_rad, elapsed_s;
    uint32_t iterations, reserved;
} robot_ik_result_v1;
uint32_t robot_abi_version(void);
/* Analytic UR5 CB implementation; failures never change output.
 * Exact target and seed lengths: 16 and 6. Caller owns result (504 bytes).
 * DLS remains unsupported (1008); unresolved singular requests return 2003. */
int robot_inverse_v1(const robot_fk_model *model,
                     const double *T_base_tool, size_t pose_count,
                     const double *q_seed_rad, size_t seed_count,
                     const robot_ik_options_v1 *options,
                     robot_ik_result_v1 *result);
#ifdef __cplusplus
}
#endif
#endif
