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
/* 几何模型：六轴DH的a/d用m、alpha/offset用rad；
 * theta=joint_sign*q+offset。固定变换为row-major double[16]。
 * q_min/q_max为逻辑关节角限位(rad)；pose_validation_tol为刚体校验容差。 */
typedef struct {
    double a_m[6], alpha_rad[6], d_m[6], theta_offset_rad[6];
    int joint_sign[6];
    double T_base_dh0[16], T_dh6_flange[16], T_flange_tool[16];
    double q_min_rad[6], q_max_rad[6];
    double pose_validation_tol;
} robot_fk_model;
/** 生成4×4单位矩阵。
 * 参数：out为可写的16个double，按行存储。
 * 返回：0成功；out为空返回1001，失败不写输出。 */
int robot_mat4_identity(double *out);
/** 计算两个4×4矩阵的乘积a·b。
 * 参数：a、b各为16个有限double；out提供16个可写double，可与输入重叠。
 * 返回：0成功；参数或计算结果非有限返回1001，失败不写输出。 */
int robot_mat4_multiply(const double *a, const double *b, double *out);
/** 转置4×4矩阵，支持原地操作。
 * 参数：a为16个有限double；out为16个可写double。
 * 返回：0成功；非法参数返回1001，失败不写输出。 */
int robot_mat4_transpose(const double *a, double *out);
/* Only rigid transforms, NOT a general 4x4 inverse. 0<tol<=1e-6. */
/** 计算刚体齐次变换的逆：[R转置,-R转置·p;0,1]。
 * 参数：t为合法4×4刚体矩阵；tol为(0,1e-6]校验容差；out为16个double。
 * 返回：0成功；非法矩阵/指针/容差返回1001。不能用于任意4×4矩阵求逆。 */
int robot_transform_inverse(const double *t, double tol, double *out);
/** 按标准DH生成单个连杆的齐次变换Rz·Tz·Tx·Rx。
 * 参数：a、d为米；alpha、theta为弧度；out为16个可写double。
 * 返回：0成功；非有限数值或空输出返回1001。 */
int robot_dh_transform(double a, double alpha, double d, double theta, double *out);
/* Pure kinematic FK. NULL model:1004; invalid:1001; joint limit:1005.
 * No speed/acceleration data required; this API cannot enable motion. */
/** 从六关节角计算base→tool0位姿，依次乘DH链和显式固定变换。
 * 参数：model为纯几何模型；q_rad为J1～J6角度(rad)；count必须为6；
 * T_base_tool为16个可写double，按行存储。输入输出容量由调用方保证。
 * 返回：0成功；1004模型为空，1001非法参数/模型，1005关节越限。
 * 失败不写输出。奇异位形可正常求FK，本接口不启动设备运动。 */
int robot_forward(const robot_fk_model *model, const double *q_rad,
                  size_t count, double *T_base_tool);
/* Frozen additive ABI 1.0.0; old FK signatures/layout remain unchanged. */
#define ROBOT_ABI_VERSION_V1 UINT32_C(0x00010000)
#define ROBOT_IK_MAX_SOLUTIONS_V1 8
#define ROBOT_IK_ANALYTIC_UR5_V1 1u
#define ROBOT_IK_DLS_V1 2u
#define ROBOT_IK_NEAREST_SEED_V1 0u
#define ROBOT_IK_ALL_V1 1u
/* IK配置：method/branch_policy采用冻结枚举；误差m/rad，预算s。
 * 解析法的迭代/阻尼专用字段必须填0；DLS方法当前返回1008。 */
typedef struct {
    uint32_t method, branch_policy;
    double position_tol_m, orientation_tol_rad, joint_margin_rad, timeout_s;
    uint32_t max_iterations, line_search_max_steps;
    double characteristic_length_m, damping, max_step_rad;
} robot_ik_options_v1;
/* IK结果：q_rad为选定解；candidates_rad前solution_count行有效。
 * branch_ids仅为本次排序索引；双误差属于所选解，elapsed_s为C耗时。
 * 解析iterations为0，reserved必须为0；仅返回码0时可读。 */
typedef struct {
    double q_rad[6];
    double candidates_rad[8][6];
    uint32_t branch_ids[8];
    uint32_t solution_count, selected_index;
    double position_error_m, orientation_error_rad, elapsed_s;
    uint32_t iterations, reserved;
} robot_ik_result_v1;
/** 查询冻结C接口版本。
 * 参数：无。返回：0x00010000（ABI 1.0.0），不分配内存。 */
uint32_t robot_abi_version(void);
/* Analytic UR5 CB implementation; failures never change output.
 * Exact target and seed lengths: 16 and 6. Caller owns result (504 bytes).
 * DLS remains unsupported (1008); unresolved singular requests return 2003. */
/** 求UR5 CB解析逆解，枚举、限位映射、逐候选FK回代并选解。
 * 参数：model为已审定模型；T_base_tool为目标16个double，pose_count=16；
 * q_seed_rad为参考关节角(rad)，seed_count=6；options为显式IK选项；
 * result为调用方分配的504字节结果，禁止与输入重叠。
 * 返回：0时result含有效候选、所选解和回代误差；非0时result逐字节不变。
 * 1001参数，1004模型缺失，1005限位，1008不支持/DLS未实现；
 * 2001证明不可达，2002预算/数值未收敛，2003不能完成奇异请求，9000内部错误。 */
int robot_inverse_v1(const robot_fk_model *model,
                     const double *T_base_tool, size_t pose_count,
                     const double *q_seed_rad, size_t seed_count,
                     const robot_ik_options_v1 *options,
                     robot_ik_result_v1 *result);
#ifdef __cplusplus
}
#endif
#endif
