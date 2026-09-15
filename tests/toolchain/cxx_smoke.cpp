#include "robot_contract.h"
#include <limits>
int main() {
    double q[6] = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6}, out[6] = {};
    if (robot_copy_joints(q, 6, out) != 0) return 1;
    for (int i = 0; i < 6; ++i) if (out[i] != q[i]) return 2;
    if (robot_copy_joints(q, 5, out) != 1001) return 3;
    q[2] = std::numeric_limits<double>::quiet_NaN();
    if (robot_copy_joints(q, 6, out) != 1001 || out[0] != 0.1) return 4;
    return 0;
}
