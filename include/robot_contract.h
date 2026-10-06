#ifndef ROBOT_CONTRACT_H
#define ROBOT_CONTRACT_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
/* ABI connectivity probe only; not FK, IK or a controller.
 * Copies six finite doubles in J1..J6 order. On failure output is untouched.
 * 0=OK, 1001=INVALID_ARGUMENT, matching src/common/contract.json.
 */
/* 用途：验证六轴数值跨C/Python边界的复制。
 * 入参：input为J1～J6顺序的6个有限double，count=6；output提供6个double。
 * 返回：0成功，1001参数错误；失败不修改输出。此探针不求运动学。 */
int robot_copy_joints(const double *input, size_t count, double *output);
#ifdef __cplusplus
}
#endif
#endif
