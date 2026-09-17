# UR5 型号、DH 与坐标核对（2026-09-17）

本轮冻结的是 **UR5 CB 系列通用名义运动学模型**，不是 UR5e，也不是某台实机的标定文件。使用范围为 simulation_only。机器可读配置见 [kinematics.json](../models/ur5/kinematics.json)，核对结果见 [验证报告](../reports/week01/ur5-model-audit.md)。

## 模型来源与可追溯性

| 来源 | 固定版本／定位 | 用途 |
| --- | --- | --- |
| ROS-Industrial universal_robot | 提交 `39ad110d8f2e8f66856a201cca88aa7a7025e3eb`；`ur_description/urdf/ur5.xacro`，`urdf/inc/ur_macro.xacro` 与 `config/ur5/` | 生成当前 URDF、固定坐标变换及关节限位 |
| Universal Robots 厂商 DH 表 | [官方参数页](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)，2026-09-17 核对 UR5 行 | 名义标准 DH 参数交叉核对，不使用 UR5e 行 |
| Robotics Toolbox for Python | 安装锁定 1.1.1；`roboticstoolbox/models/DH/UR5.py`；报告记录文件 SHA256 | 对照发现 d1 差异；随后用显式项目参数构造独立 DHRobot |

[上游固定提交](https://github.com/ros-industrial/universal_robot/tree/39ad110d8f2e8f66856a201cca88aa7a7025e3eb/ur_description)可追溯全部配置。默认运动学与关节限位 YAML 的原样快照保存在 `models/ur5/source-config/`，许可沿用该目录上层的 LICENSE；其 SHA256、URDF SHA256 和来源提交记入 kinematics.json。URDF 与所有网格仍由 manifest.json 校验，未改动既有模型资产。来源 YAML 中的 calib 哈希只是该通用配置的标识，不是本用户设备的标定证据。

既有适配仅涉及 mesh 相对路径、简化 visual 和四个坐标链接显式零质量；实体连杆惯量、运动链与限位沿用上游。本次不调整惯性参数，也不宣称动力学等价已验收。

## 标准 DH 与关节映射

使用标准 DH，非 modified DH：

`A_i = RotZ(theta_i) · TransZ(d_i) · TransX(a_i) · RotX(alpha_i)`

`theta_i = joint_sign_i * q_i + theta_offset_i`，右手系、列向量、长度 m、角度 rad。旋转正方向遵循每个关节自身 +Z 轴右手规则，不是全局 +Z。

| 逻辑轴 | URDF 关节 | a_i (m) | d_i (m) | alpha_i (rad) | sign | theta_offset (rad) |
| --- | --- | --- | --- | --- | --- | --- |
| J1 | shoulder_pan_joint | 0 | 0.089159 | π/2 | +1 | 0 |
| J2 | shoulder_lift_joint | -0.425 | 0 | 0 | +1 | 0 |
| J3 | elbow_joint | -0.39225 | 0 | 0 | +1 | 0 |
| J4 | wrist_1_joint | 0 | 0.10915 | π/2 | +1 | 0 |
| J5 | wrist_2_joint | 0 | 0.09465 | -π/2 | +1 | 0 |
| J6 | wrist_3_joint | 0 | 0.0823 | 0 | +1 | 0 |

零偏 0 表示当前 URDF 关节值与上述名义 DH 角直接对应，不能推导出实机编码器无需校准。URDF origin 的固定 RPY 属于连杆坐标变换，不能再次当作关节 theta 零偏加到 q。演示初始姿态 `[0,-π/2,0,-π/2,0,0]` 是预设姿态，不是零位定义。

上游 URDF 中 alpha 相关 RPY 使用有限小数（例如 1.570796327），并含约 1e-11 m 的微小平移；本 DH 使用精确 π/2。保留原 URDF，不强行抹去数值；在预定 1e-9 m / 旋转矩阵元素 1e-9 容差内验收。

## 基座、法兰与工具

`T_A_B` 把 B 系坐标变换到 A 系。工程契约的 base 对应 **URDF link base**；工程 tool 在本次无附加工具配置中对应 **tool0**。两者不是 base_link 与 flange 的别名。

下面各矩阵均无平移；齐次末行为 `[0,0,0,1]`。完整 4×4 数值在配置中。

| 变换 | 3×3 旋转矩阵（各行以分号分隔） | 含义 |
| --- | --- | --- |
| T_base_link_base | `[-1,0,0; 0,-1,0; 0,0,1]` | base_link 到 base 的固定父子关系：绕 Z 轴 π |
| T_base_dh0 | `[1,0,0; 0,1,0; 0,0,1]` | DH 起始系与工程 base 对齐 |
| T_dh6_flange | `[0,1,0; 0,0,1; 1,0,0]` | DH 第六系到 URDF flange 的坐标关系 |
| T_flange_tool | `[0,0,1; 1,0,0; 0,1,0]` | flange 到 tool0，来自 URDF 固定关节 |

表中“到”描述坐标系链的父子关系；点坐标变换方向始终由 T_A_B 下标定义。例如 `p_base_link = T_base_link_base · p_base`。T_dh6_flange 与 T_flange_tool 互逆，故 DH6 与 tool0 名义重合，但不能因此将 flange 也视为 tool0。

`T_base_tool = T_base_dh0 · A1…A6 · T_dh6_flange · T_flange_tool`。

若仿真将 URDF 根 base_link 放在世界原点且姿态为单位阵，则 `T_world_base_link=I`，而 `T_world_base=T_base_link_base`，不是 I。任意安装位姿下应使用 `T_world_base=T_world_base_link · T_base_link_base`。新增夹具/TCP 时需显式追加 tool0 到 TCP 的变换并重新核对，不沿用本次“无附加工具”假设。

零关节向量在工程 base 中给出工具位置 `[-0.81725,-0.19145,-0.005491] m`，旋转名义上为 `RotX(π/2)`。因此关节零位不等于末端位姿单位阵。

## Robotics Toolbox 差异与基准用法

已安装 RTB 1.1.1 内置 UR5 的 `d1=0.089459 m`，而厂商 UR5 表和当前 URDF 是 `0.089159 m`。115 组测试显示直接比较约产生 0.0003 m 的位置差，超出既定基准门槛。该差异不是关节零偏，也不应通过伪造基座平移来掩盖。

`verify_model_alignment.py` 使用独立 `DHRobot([RevoluteDH(...)])`，从已核对的项目参数显式构造参考模型并设置 base/tool。保留原依赖版本和内置 UR5，不修改 site-packages。之前 `results/baseline-readiness/` 的内置 UR5 样本只证明依赖可运行，仍禁止直接当作本项目 FK 真值。

本轮比较：URDF XML 的 origin/axis 链、显式标准 DH 矩阵连乘、项目参数的 RTB DHRobot。没有调用未来 C FK，因此结果只关闭模型坐标核对，不关闭 C 算法验收。

## 限位与待确认项

位置限位：J1/J2/J4/J5/J6 为 ±2π，J3 为 ±π；速度上限均为 π rad/s，来自当前 URDF。上游注释明确 J3 的 ±π 是规划模型限制，不能描述成实机机械极限。加速度上限上游未提供，不填猜测数值；配置中 qdd_max_rad_s2 为 null。因此本文件是核对用运动学配置，不是满足工程契约所有必填字段的可启动控制模型。

待确认：仿真规划/控制加速度限值、实际 TCP/安装位姿（如偏离当前演示）、若进入实机阶段则需设备序列号对应的标定数据。仿真限值在第3周规划接口冻结前确定；实机参数在任何实机启用前确认。现有无附加工具、世界根位姿为 I 的仿真约定已明确，无须把它们继续列为未知。

## 重跑

```bash
# 已按 docs/testing.md 创建 .venv-baseline 后，从任意目录调用脚本绝对路径亦可
.venv-baseline/bin/python scripts/verify_model_alignment.py --output results/model-audit
.venv-baseline/bin/python scripts/run_tests.py --suite model --output results/test-runs/model
```

固定 3 个命名位形、每轴正负扰动共 12 个位形、100 个限位内随机位形；seed=20260917。位置误差与旋转矩阵元素误差分别 ≤1e-9；同时验证法兰及世界映射。报告保存逐样本数据、汇总、模型/源码哈希与依赖版本。极小姿态误差用相对旋转的 atan2 等价形式避免 acos 的消减误差；旋转矩阵元素阈值是本项验收判据。
