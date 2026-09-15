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
int robot_copy_joints(const double *input, size_t count, double *output);
#ifdef __cplusplus
}
#endif
#endif
