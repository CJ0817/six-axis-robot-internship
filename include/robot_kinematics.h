#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
/* ABI v1, float64, row-major 4x4, column vectors, m/rad, J1..J6.
 * Caller owns valid buffers (16 doubles for matrices, 6 for joints).
 * All outputs remain untouched on error; input/output aliasing is supported.
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
#ifdef __cplusplus
}
#endif
#endif
