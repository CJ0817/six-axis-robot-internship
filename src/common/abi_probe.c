#include "robot_contract.h"
#include <math.h>
int robot_copy_joints(const double *input, size_t count, double *output) {
    if (!input || !output || count != 6) return 1001;
    for (size_t i = 0; i < 6; ++i) if (!isfinite(input[i])) return 1001;
    for (size_t i = 0; i < 6; ++i) output[i] = input[i];
    return 0;
}
