# C 矩阵运算与正运动学验证

2026-09-17，云端Linux，GCC 13.3.0，C11，Release；Python3.11.9、RTB1.1.1、NumPy1.26.4。模型来源与参数沿用已核对的UR5 CB模型，未改URDF及第三方模型。

交付：连杆变换推导、固定4×4矩阵运算、单节DH、robot_forward、ctypes Result包装及示例、独立C测试和URDF/RTB对照入口。

| 验证 | 结果 |
| --- | --- |
| CTest | 3/3通过，含既有C++及Python ABI |
| 独立已知FK位形 | 3/3通过；包含非单位基座/工具、负关节方向和非零偏置 |
| 完整UR5对照 | 115/115通过，失败0 |
| C对URDF最大位置误差 | 2.11727994531e-10 m |
| C对URDF最大旋转矩阵元素差 | 5.71722377773e-10 |
| C对显式RTB最大矩阵元素差 | 3.33066907388e-16 |
| 既有入口/模型错误回归 | 2项入口回归、1项错误d1回归通过 |

编译与重跑命令见[实现说明](../../docs/forward-kinematics.md)。本轮实际使用相邻已锁定的test-baseline-task虚拟环境，通过CMake显式指定其Python与Ninja路径；正式入口使用仓库本身.venv与.venv-baseline。

原始证据见 `results/fk-validation/`：report.json含误差有效数、均值、RMSE、p95、最大值、源码/模型/动态库哈希；c-samples.json含逐样本C误差；samples.json含对应q与模型核对；ctest.log及demo.json为实际执行输出。失效输出不填零、不伪造成功。极小姿态角用atan2等价形式避免消减，矩阵元素误差按既定阈值判定。

本次关闭功能项KIN-01/KIN-02；不宣称整个运动学模块完成。雅可比、解析IK、C/FFI性能验收仍待实现/测量。加速度限值未定不影响纯FK，不能据此启用规划/控制。
