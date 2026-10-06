# 运动学阶段统一技术基准 v1（2026-10-06）

本页供后续规划、控制和适配器集成使用。正式功能/性能判定见[阶段测试报告](../reports/week02/kinematics-stage-test-report.md)；字段级二进制规范以[ABI 1.0.0](abi-v1.md)、源码头文件和[机器快照](../src/common/abi-v1.json)为准，本页不创建新的 ABI 版本。尚未实现的规划和控制接口以[工程总方案](engineering-contract.md)对应阶段的待办为准。

## 固定输入、算法与版本

| 类别 | 集成时使用的基准 |
|---|---|
| 构建/语言 | Linux x86_64 C11 / C++17，GCC/G++ 13.3.0，CMake 3.31.6；Python 3.11.9、NumPy 1.26.4；PyBullet 3.2.7；RTB 1.1.1 在独立基准环境。完整版本见 [environment.lock.json](../environment.lock.json)。 |
| 模型 | `ur5_cb_generic`，标准 DH，6 个旋转轴 J1～J6，从 base 到 tool0，长度 m、角度 rad、时间 s；ROS-Industrial URDF 固定上游提交 `39ad110d8f2e8f66856a201cca88aa7a7025e3eb`。`a=[0,-0.425,-0.39225,0,0,0]` m，`d=[0.089159,0,0,0.10915,0.09465,0.0823]` m，`alpha=[π/2,0,0,π/2,-π/2,0]` rad；逻辑关节零偏均0，符号均+1。 |
| 变换与限位 | `A_i=Rz(sign_i*q_i+offset_i)Tz(d_i)Tx(a_i)Rx(alpha_i)`，`T_base_tool=T_base_dh0·A1…A6·T_dh6_flange·T_flange_tool`；从[模型 JSON](../models/ur5/kinematics.json)读取显式矩阵、逐轴 `q_min/q_max` 和 `qd_max`，不能用这里的摘要重建固定矩阵。`qdd_max_rad_s2=null`，规划/控制前必须另行冻结仿真加速度限值；没有实机标定。 |
| FK/IK | C FK 顺序累乘；UR5 C 解析 IK 枚举最多8个离散候选，等价角映射、限位筛选、逐候选 C FK 回代与去重。Python状态层对候选按行程/限位距离/奇异风险评分；近奇异限步，必要时在限定步长内局部阻尼退化。精确奇异连续族的全局最优仍未实现。 |

不要用 RTB 内置 UR5 的 DH 参数作本模型真值：其 `d1=0.089459 m`，比项目显式参数大 0.0003 m。参数来源、矩阵与限制以[模型约定](ur5-model-conventions.md)为准。

## 稳定接口和返回语义

```c
int robot_forward(const robot_fk_model *model,
                  const double *q_rad, size_t count, double *T_base_tool);
int robot_inverse_v1(const robot_fk_model *model,
                     const double *T_base_tool, size_t pose_count,
                     const double *q_seed_rad, size_t seed_count,
                     const robot_ik_options_v1 *options, robot_ik_result_v1 *result);
```

`count=seed_count=6`，`pose_count=16`；4×4 齐次矩阵为连续 row-major `double[16]`，候选为 `[8][6]`，实际行数取 `solution_count`。`robot_fk_model` 704 字节、`robot_ik_options_v1` 72 字节、`robot_ik_result_v1` 504 字节，Linux x86_64 SysV LP64 自然8字节对齐。符号、参数顺序、布局、错误码和输出原子性均冻结。`robot_abi_version()` 返回 `0x00010000`。新增能力另起版本化符号，不能修改现有结构。合法模型指针/缓冲区由调用方管理。

只有返回码0时位姿或解可交给下游；失败时 C 输出缓冲不变，上层 `data=null`。关节状态不得因错误推进。常见编号：1001 参数、1004 模型缺失、1005 限位、1008 未实现、2001 可证明不可达、2002 未收敛/预算、2003 无法完成请求的奇异性、9000 内部错误；编号以[contract.json](../src/common/contract.json)为源。C 解析选项 `method=1` 已实现；C `method=2` 阻尼法仍1008。Python [StatefulIKSelector](ik-stateful-selection.md) 的局部阻尼是单独的规划适配策略，不改变 C 调用 ABI；返回 `mode=dls_near_singular` 时 `selected_candidate_index=null`，必须查回代误差和路径诊断。

## 冻结阈值、测试与状态

| 指标 | 条件与门槛 | 当前结论 |
|---|---|---|
| FK 模型/已知位形 | 115 模型位形对齐；位置及旋转矩阵元素各≤1e-9；独立已知位形≥3 | 通过，见[正式报告](../reports/week02/kinematics-stage-test-report.md)。 |
| IK 回代 | 正常100、宽初值20、近奇异20各组独立报告；每一保留候选位置≤1e-5 m、姿态≤1e-4 rad、有限且不越限；空间120另列 | 140/140和120/120；空间120的890个候选合格。精确连续奇异不计成功。 |
| FK 耗时 | 115组各100次、每层预热1000；原生C和Python→C分别p95≤1 ms且**每次**≤1 ms | C层通过；Python→C 1次2.046632 ms超限，整项待关闭。 |
| IK 耗时 | 正常100个目标各重复10次，预先冻结硬件与预热；p95≤50 ms、每调用预算≤200 ms | 现有一次性140组观测不替代正式性能验收。 |
| 奇异区域 | 近奇异步长系数0.25；最小奇异值soft=0.02、hard=1e-5；局部阻尼成功还需相同回代阈值、路径采样、限位、连续性通过 | Python层103/103加6/6回归通过；尚无连续路径与碰撞证明。 |

统一统计按[统计规则](statistics-rules.md)：有效数、均值、RMSE、p95、最大值和无效数分列；成功率的分母为应执行目标全数，异常/超时计失败；C 函数、Python→C、全进程的计时边界分开。功能观测与性能正式重复协议分开，失败不能以平均值掩盖。机器可读门槛来自[tests/metrics.json](../tests/metrics.json)，报表可由 `python scripts/render_metrics.py` 再生成。

## 下游接入条件

规划器读取同一份模型和单位/轴序，逐路径节点核验 `code==0`、FK 误差、步长、限位和状态推进；无解时停止输出该段并重规划。数值阻尼仅在局部容差及路径保护都满足时提供有效节点，不能把失败的 `details.candidates` 或部分迭代作为轨迹。后续应补连续雅可比/奇异路径验证、速度及加速度极值、碰撞检查和动态限值。控制器需等待独立的反馈、状态机、增量 PID、前馈与周期验收；本阶段成功不构成实机运动许可。
