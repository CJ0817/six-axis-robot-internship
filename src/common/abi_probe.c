#include "robot_contract.h"
#include <math.h>
/* 复制六个关节值以验证C/Python连通性。input为6个有限double，
 * count必须为6，output提供6个double；0成功，1001参数错误。失败输出不变。 */
int robot_copy_joints(const double *input, size_t count, double *output) {
    if (!input || !output || count != 6) return 1001;
    for (size_t i = 0; i < 6; ++i) if (!isfinite(input[i])) return 1001;
    for (size_t i = 0; i < 6; ++i) output[i] = input[i];
    return 0;
}
