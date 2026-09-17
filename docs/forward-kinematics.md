# 连杆变换推导与 C 正运动学

本实现采用[已核对的 UR5 CB 名义模型](ur5-model-conventions.md)。长度 m、角度 rad、double、J1～J6；矩阵按行存储，几何计算使用齐次列向量。C 计算只负责纯运动学，不开启设备或仿真控制。

## 1. 从单节变换到末端位姿

标准 DH（不是 modified DH）依次执行：绕前一坐标系 z 轴旋转 theta、沿该 z 轴平移 d、沿旋转后的 x 轴平移 a、绕该 x 轴旋转 alpha。

令 c=cos(theta)、s=sin(theta)、ca=cos(alpha)、sa=sin(alpha)。由四个齐次矩阵相乘得到：

$$
A_i=\begin{bmatrix}
c & -s c_\alpha & s s_\alpha & a c\\
s & c c_\alpha & -c s_\alpha & a s\\
0 & s_\alpha & c_\alpha & d\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

平移列中 x/y 是 a*cos(theta)、a*sin(theta)，不能直接将 a 放到全局 x 坐标。`theta_i = joint_sign_i*q_i + theta_offset_i`；零偏只添加一次，URDF origin 的 RPY 不再叠加到 q。

`T_dh0_dhi = A1 A2 ... Ai`，每步右乘下一节，顺序不可交换。完整末端变换为：

`T_base_tool = T_base_dh0 A1 A2 A3 A4 A5 A6 T_dh6_flange T_flange_tool`。

本项目 base 对应 URDF base，tool 对应 tool0。UR5 中 T_dh6_flange 与 T_flange_tool 互逆，但实现仍分别读取和相乘，不能对一般工具假设两者抵消。需要世界系结果时，由调用方另乘 `T_world_base`。

## 2. 矩阵运算

4×4乘法：`C[i,j] = sum(A[i,k]*B[k,j], k=0..3)`，按行偏移 `4*i+j`。所有运算先写局部数组、检查有限性，再一次性复制输出，因此支持输出与输入重叠，错误时输出保持不变。

转置：`B[i,j]=A[j,i]`。刚体逆仅适用于 `T=[R,p;0,1]`，利用 `R^-1=R^T` 得到 `T^-1=[R^T,-R^T p;0,1]`。先校验末行、R正交性及det(R)=+1；这不是任意4×4矩阵求逆器。

## 3. C 接口与错误语义

公开头文件：[robot_kinematics.h](../include/robot_kinematics.h)。实现：[forward.c](../src/kinematics/forward.c)。沿用 `librobot_contract.so`，原 robot_copy_joints ABI 保留。

| 函数 | 输入／输出 |
| --- | --- |
| robot_mat4_identity(out) | 写入4×4单位阵 |
| robot_mat4_multiply(a,b,out) | 计算a*b，支持原地运算 |
| robot_mat4_transpose(a,out) | 转置，支持原地运算 |
| robot_transform_inverse(t,tol,out) | 刚体逆，0<tol≤1e-6 |
| robot_dh_transform(a,alpha,d,theta,out) | 单节标准DH矩阵 |
| robot_forward(model,q,count,out) | count必须为6；输出T_base_tool |

所有矩阵指针须指向至少16个double；q须有count个可读double。指针必须指向有效内存，C不能推断悬空指针或实际分配容量。model为头文件中的固定ABI结构，含a/alpha/d/offset各6个double、sign 6个int、三个16元素固定变换、6轴位置上下限及校验容差。它是完整工程模型的**纯FK所需字段子集**，不要求速度/加速度或控制状态；不得据此绕过控制模型的完整校验。

返回0成功；1001为非法参数、非有限数、非法刚体矩阵、无效限位或计算溢出；NULL模型返回1004；有限q越界返回1005。边界等号允许，不自动折返角度，奇异位形不影响FK求值。仅在所有计算成功后写out，失败保留原缓冲区；调用方必须检查返回码，不能执行旧输出。

C的整数码和输出参数是底层ABI；[Python适配器](../src/adapters/c_kinematics.py)包装为 `{code,message,data,details}`，失败data=null，并检查显式单位、J1～J6顺序、standard_dh及base/tool0坐标标签。C结构本身不携带字符串标签，直接C调用方须履行这些前置约定。未提供模型字段时返回1004；模型文件/动态库路径错误由示例报告，不隐式加载另一个模型。

## 4. 编译、调用与验收

先按环境说明准备 `.venv` 与 `.venv-baseline`，然后在仓库根目录执行：

```bash
export PATH="$PWD/.venv/bin:$PATH"
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13 -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build
ctest --test-dir build --output-on-failure
.venv/bin/python examples/fk_demo.py --q 0 0 0 0 0 0
.venv-baseline/bin/python scripts/run_tests.py --suite fk --output results/test-runs/fk
```

FK示例仅使用Python标准库ctypes，不依赖NumPy或RTB。基准对照使用锁定的RTB环境。也可运行 `scripts/verify_model_alignment.py --library build/librobot_contract.so --output results/fk-validation`；此模式先执行相邻构建目录中的fk_unit，再比较完整链。

独立已知位形：测试使用所有alpha为0的合成链，第一节a=0.5 m、d=0.2 m、sign=-1、offset=π/2；基座Rz(π/2)并平移[0.1,0.2,0.3] m；工具平移[0.1,0,0.05] m。其末端旋转为Rz(π−q1)，位置为[0.1+0.6*cos(π−q1),0.2+0.6*sin(π−q1),0.55]，q1=0、π/2、−π/2的期望矩阵直接写入C测试，不由被测FK生成。

完整UR5比较使用既有115组位形，独立URDF XML origin/axis链及显式项目参数的RTB对照，不用有d1差异的内置UR5作真值。阈值保留1e-9 m及旋转矩阵元素1e-9；逐样本与汇总保存于验证目录。C纯计算/FFI性能暂未测量，本次不关闭KIN-05性能指标、雅可比、IK或规划控制待办。
