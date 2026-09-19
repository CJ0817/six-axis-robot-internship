# 工程边界与数据约定 v1.2.0（正式方案与接口扩展计划）

本文件定义本实习工程的接口规范，不代表算法或设备控制已经实现。字段采用语言无关表示，拟实现技术栈见第 8 节；机器可读常量及错误码以 [contract.json](../src/common/contract.json) 为唯一编号来源。

> 正式技术基线：C11 算法库、C++17 编译支持、Python 3.11.9 / PyBullet 3.2.7 仿真、UR5 URDF，固定版本及环境验证状态见[环境配置](environment-setup.md)。任务书要求解析 IK、三次/五次/梯形规划、直线/圆弧、避障、增量 PID 及速度/加速度前馈。第 7.1～7.4 节保留辅助基线接口定义，第 7.5 节列出正式要求的接口扩展计划；尚未实现的接口不代表可运行能力。早期选型已移至[历史方案归档](history/engineering-plan-python.md)，不作为当前方案或验收依据。

## 1. 模块边界

| 模块 | 负责 | 输入 → 输出 | 边界 |
| --- | --- | --- | --- |
| common | 共享常量、数据校验、结果封装 | 原始数据 → 合法数据或错误 | 不依赖其他业务模块 |
| kinematics | 正运动学、逆运动学、几何雅可比 | 模型、q / 目标位姿 → 位姿 / q / J | 不生成时间轨迹，不读写设备，不运行控制循环 |
| planning | 路径插值、时间参数化、轨迹约束检查 | 起终点、模型限值、规划参数 → 带时间的参考轨迹 | 可调用运动学；不读取实时硬件，不输出设备命令 |
| control | 参考跟踪、状态管理、反馈有效性检查 | 单点参考、测量状态、dt、控制器配置 → 带模式的控制命令 | 不求全局路径；不直接进行网络或设备 I/O |
| adapters | 设备/仿真读写、外部单位与轴序转换 | 外部协议 ↔ 工程规范数据 | 不实现运动学或规划算法 |
| examples | 应用编排、加载模型、按时采样轨迹、调用控制器和适配器 | 实验配置 → 运行记录 | 决定异常后的停止策略与运行周期 |

依赖方向：kinematics → common；planning → kinematics/common；control → common；adapters → common。examples 组装各模块，不允许底层反向导入 examples。
动力学参数、重力补偿等若后续需要，应作为独立模型/服务提供；不混入正逆运动学。

## 2. 单位与关节顺序

| 量 | 统一单位 / 表示 |
| --- | --- |
| 长度、位置、模型连杆尺寸 | m |
| 关节角、姿态角 | rad |
| 时间、控制周期 | s |
| 关节速度、加速度 | rad/s、rad/s² |
| 线速度、线加速度 | m/s、m/s² |
| 力、关节力矩 | N、N·m |

全部数值必须有限，内部计算优先使用双精度浮点数。输入输出字段尽量带单位后缀，例如 q_rad、qd_rad_s、time_s。
毫米转米：乘 0.001；度转弧度：乘 π/180。转换仅发生在导入、显示、设备适配边界；不得根据数值大小猜测单位，裸数值无法自动检测度/弧度误用。

关节逻辑顺序固定为从基座向末端的 [J1, J2, J3, J4, J5, J6]。位置、速度、加速度、力矩、限位以及雅可比列均使用相同顺序。
零基索引 0…5 对应 J1…J6；MATLAB 等一基索引 1…6 对应 J1…J6。
外部轴名/轴号不同必须通过显式映射重排，不能只更改标签。六关节向量逻辑形状为 (6,)，矩阵运算视为 6×1 列向量，JSON 表示为长度 6 的数组。
不自动把关节角折返到 [-π,π]；保留连续性与实际关节限位。角度展开策略由规划器依据模型显式选择。

## 3. 坐标与模型

2026-09-17 核对结果：UR5 CB 名义 DH、关节正方向/零偏及 base/flange/tool0 映射已确认，见[模型核对说明](ur5-model-conventions.md)。加速度限值与实机标定仍待确认。核对配置不是完整可启动控制模型，C算法实现状态不变。

使用右手坐标系。T_A_B 表示把 B 系坐标转换到 A 系：p_A = T_A_B · p_B（齐次列向量）。
位姿接口默认使用 T_base_tool，4×4，左上为旋转矩阵 R，右上为以米为单位的平移，末行为 [0,0,0,1]。
R 应满足 RᵀR≈I、det(R)≈1；校验容差必须显式配置并记录。工具相对法兰的变换必须来自模型，不默认二者重合。
雅可比 J 为 6×6，满足 [v_base; ω_base] = J(q) qd，前三行为工具原点线速度，后三行为角速度，均在 base 系表达。
对纯 FK 求值，奇异位形本身不必报错；只有请求无法完成时返回 KINEMATIC_SINGULARITY。数值 IK 未收敛不能直接认定不可达。

机器人型号已确定为 UR5，仿真加载已锁定来源的 URDF；C 运动学采用标准 DH 加显式固定变换。URDF 与 DH 的轴正方向、名义零偏、位置/速度限位及工具映射已通过 BASE-02 核对；加速度限值及动力学参数的适用范围仍须记录，确认节点见第 8 节。
所有实现必须从 models/ 加载这些信息；所需参数缺失返回 MODEL_NOT_CONFIGURED，不能用猜测值作为真机参数。
世界系与基座系不同必须提供 T_world_base；姿态输入暂以矩阵为准，后续引入欧拉角或四元数时另行规定旋转顺序与分量顺序。

## 4. 语言无关接口契约

以下为待实现的接口定义，不是可运行函数。

| 接口 | 输入 | 成功时 data |
| --- | --- | --- |
| forward(model, q_rad) | 模型、6 维关节位置 | T_base_tool |
| inverse(model, T_base_tool, q_seed_rad, options) | 目标位姿、初始解、迭代预算及位置/角度容差 | 满足容差与限位的 q_rad |
| jacobian(model, q_rad) | 模型、6 维关节位置 | J_base_tool |
| plan_joint(model, q_start_rad, q_goal_rad, options) | 关节起终点、采样/时长与约束 | Trajectory |
| plan_cartesian(model, T_start, T_goal, q_seed_rad, options) | 工具位姿起终点、初始解与约束 | Trajectory；通过运动学转换为关节参考 |
| step(reference, measured, dt_s, config, state, context) | 单点参考、反馈、正控制周期、控制配置、持久状态与当前时刻/事件 | command、action 与 next_state；详见第 7 节 |

Trajectory：time_s 为长度 N 的数组，N≥2，首项为 0，后续严格递增；q_rad、qd_rad_s、qdd_rad_s2 均为 N×6，每一行对应同一时刻。
完整轨迹须检查连续段内的位置/速度/加速度约束；仅离散采样通过不能声称段间约束全部满足。碰撞检查仅在配置模型与检查器后才可声明完成。
reference 包含 q_rad、qd_rad_s、qdd_rad_s2；measured 包含 q_rad、qd_rad_s、time_s，时间戳使用同一单调时钟。
command 非 null 时必须有 mode 和相应字段：position → q_rad；velocity → qd_rad_s；torque → tau_nm。一个命令只能采用一种模式，适配器必须确认设备支持，不能把位置命令解释为力矩。
控制周期、反馈超时、跟踪误差阈值、输出限值及控制增益来自配置，不能用默认猜测值启动控制。

## 5. 统一结果与错误传播

所有模块公开接口返回 Result：{code, message, data, details}。
code 为整数；仅 0 表示成功。成功时 data 符合接口定义；失败时 data 必须为 null，不允许用全零向量伪装成功。
message 用于可读说明，不供程序分支判断；details 是结构化上下文，可包括 module、field、joint、actual、limit、cause_code。不得放入凭据。
共享校验错误使用 1xxx；运动学 2xxx；规划 3xxx；控制 4xxx；I/O 5xxx；未预期异常为 9000。编号发布后不重用。
跨模块传播优先保留原 code，增加上下文；如果必须转换为上层错误，details.cause_code 保留根因。
遇到多个错误按确定顺序返回首个：类型/形状/有限性 → 单位/轴序/坐标系 → 模型/时间 → 限位 → 算法。可在 details 补充其余问题。
接口占位必须返回 NOT_IMPLEMENTED，不得产生模拟成功结果。

错误时本次 data 不可执行。应用编排停止发送新的轨迹参考，并调用设备/仿真明确支持的停止机制；不得将全零命令当作通用停止命令。物理急停独立于这些软件错误码。

### 结果示例

```json
{"code":1005,"message":"J2 超出配置限位","data":null,"details":{"module":"planning","joint":"J2","field":"q_rad"}}
```

## 6. 后续实现验收

- 对同一姿态输入，显式完成 1000 mm → 1 m、180° → π rad 转换后，计算结果应一致。
- 用六个互不相同的值验证外部轴序重排；拒绝缺失、重复或未知轴名。
- 拒绝长度非 6、NaN/Inf、错误矩阵、重复/逆序时间以及非正 dt。
- 模型配置完成后验证 FK/IK 回代误差、雅可比数值差分与轨迹段间约束。
- 强制触发模型缺失、求解不收敛、反馈超时，确认错误码稳定且失败数据不会下发。

当前交付的是工程结构和接口规范；上述算法与控制行为测试在对应实现落地后执行。


## 7. 参数字段与控制器状态（v1.1 新增）

第 7.1～7.4 节定义辅助基线接口，不覆盖全部正式算法。第 7.5 节是待冻结的扩展计划，未冻结前不得把新增字段发送给旧接口。v1.1 为 step 增加必填 context，并允许无运动命令的成功状态转换；旧调用方必须升级。
contract.json 的 1.0.0 是共用常量/错误码表版本，本次不新增、不修改编号；本文文档版本为 1.2.0；第 7.1～7.4 节既有接口与 state.schema_version 仍为 1.1.0，扩展实现前须单独发布新接口版本和迁移规则。
下列 F 指 float64 有限实数，I 指整数，B 指布尔，S 指字符串；(6,) 固定对应 J1…J6，二维数组按行存储。
所有表格逐个列出字段；“条件必填”以所述分支为准。未列出的字段拒绝，缺必填字段拒绝；可选字段只能采用表内明示默认值，并写入实验配置。
模型所需参数缺失用 1004；其他缺字段/类型/形状错误用 1001；显式单位、轴序、坐标不符分别用 1002/1003/1007。

### 7.1 model

| 字段 | 类型、形状 | 必填 | 约束 / 含义 |
| --- | --- | --- | --- |
| model_id | S，标量 | 是 | 非空，机器人模型标识 |
| revision | S，标量 | 是 | 非空；实验记录还须保存模型文件 SHA-256 |
| usage | S，标量 | 是 | simulation_only 或 hardware_validated；标签本身不构成设备启用许可 |
| dof | I，标量 | 是 | 恒为 6 |
| joint_names | S，(6,) | 是 | 严格为 J1…J6 |
| joint_types | S，(6,) | 是 | 全部 revolute；本版不支持移动关节 |
| length_unit | S，标量 | 是 | m |
| angle_unit | S，标量 | 是 | rad |
| base_frame | S，标量 | 是 | base |
| tool_frame | S，标量 | 是 | tool |
| representation | S，标量 | 是 | standard_dh；其他方式本版返回 1008 |
| a_m | F，(6,) | 是 | 标准 DH 连杆长度，允许有符号值 |
| alpha_rad | F，(6,) | 是 | 标准 DH 扭转角 |
| d_m | F，(6,) | 是 | 标准 DH 轴向偏移，允许有符号值 |
| theta_offset_rad | F，(6,) | 是 | q=0 时 DH 角偏置 |
| joint_sign | I，(6,) | 是 | 每项为 +1 或 -1；DH theta=joint_sign*q+theta_offset |
| T_base_dh0 | F，(4,4) | 是 | DH 起始系到 base；不可隐式假定单位阵 |
| T_dh6_flange | F，(4,4) | 是 | 法兰系到 DH 第六系 |
| T_flange_tool | F，(4,4) | 是 | 工具系到法兰系 |
| q_min_rad | F，(6,) | 是 | 逐轴严格小于 q_max_rad |
| q_max_rad | F，(6,) | 是 | 有限限位；本版无无限转动轴 |
| qd_max_rad_s | F，(6,) | 是 | 逐轴 >0 |
| qdd_max_rad_s2 | F，(6,) | 是 | 逐轴 >0 |
| pose_validation_tol | F，标量 | 是 | 0<值≤1e-6，实验基线 1e-9 |
| T_world_base | F，(4,4) | 条件 | 使用世界坐标时必填；否则可省略 |

标准 DH 单节变换明确为 RotZ(theta)·TransZ(d)·TransX(a)·RotX(alpha)。
FK 为 T_base_dh0·A1…A6·T_dh6_flange·T_flange_tool。雅可比列须计入 joint_sign。
所有变换须满足：末行与 [0,0,0,1] 的最大绝对差、||RᵀR-I||F、|det(R)-1| 均不超过 pose_validation_tol，且 det(R)>0。
逻辑关节 q 的零位/限位定义不得与 DH theta 混用。真实参数未确认时只允许显式命名的合成仿真模型，禁止把合成参数标为实机参数。

### 7.2 options：按接口区分，禁止共用含义不明的选项包

inverse 的 options：

| 字段 | 类型、形状 | 必填 | 约束 / 实验基线 |
| --- | --- | --- | --- |
| method | S，标量 | 是 | damped_least_squares |
| max_iterations | I，标量 | 是 | >0；基线 200 |
| timeout_s | F，标量 | 是 | >0；基线 0.2，超时或迭代耗尽返回 2002 |
| position_tol_m | F，标量 | 是 | >0；基线 1e-5 |
| orientation_tol_rad | F，标量 | 是 | >0；基线 1e-4 |
| characteristic_length_m | F，标量 | 是 | >0；由模型尺度明确设置，记录实际值 |
| damping | F，标量 | 是 | >0；对无量纲残差/雅可比使用，基线 1e-3 |
| max_step_rad | F，标量 | 是 | >0；限制迭代步最大绝对分量，基线 0.1 |
| line_search_max_steps | I，标量 | 是 | ≥1；基线 10，步长逐次乘 0.5 |
| joint_margin_rad | F，标量 | 是 | ≥0；两端收缩后区间非空，基线 0 |
| seed_policy | S，标量 | 是 | single；多初值搜索后续扩展，本版不隐式随机重启 |

q_seed_rad 为 F(6,)，在收缩限位内；T_base_tool 为合法 F(4,4)。
位置残差为 p_target-p_current；姿态残差为 log(R_target R_currentᵀ) 的旋转向量，均在 base 表达。
求解使用 [位置残差/characteristic_length_m; 姿态残差] 及相同缩放的几何雅可比，避免直接混加米和弧度。
每步回溯须降低残差并符合限位；不允许最后一次迭代未经 FK 回代就返回成功。
失败只在有几何证明时使用 2001；普通不收敛用 2002，保留迭代数、耗时、最终误差于 details。

plan_joint 和 plan_cartesian 共用字段：

| 字段 | 类型、形状 | 必填 | 约束 / 实验基线 |
| --- | --- | --- | --- |
| sample_period_s | F，标量 | 是 | >0；基线 0.01 |
| duration_s | F 或 null，标量 | 是 | null 表示自动定时；数值须 >0 且≥sample_period_s |
| duration_policy | S，标量 | 是 | stretch 或 strict；null 时必须 stretch |
| max_duration_s | F，标量 | 是 | ≥sample_period_s；自动/延长定时上限，基线 60 |
| velocity_scale | F，标量 | 是 | 0<值≤1；基线 0.5，乘模型速度限值 |
| acceleration_scale | F，标量 | 是 | 0<值≤1；基线 0.5，乘模型加速度限值 |
| joint_margin_rad | F，标量 | 是 | ≥0；收缩后位置区间非空 |
| timeout_s | F，标量 | 是 | >0；关节基线 0.5，笛卡尔基线 10 |
| collision_policy | S，标量 | 是 | unchecked；返回轨迹必须标记未检查碰撞 |

plan_joint 另含 method：S 标量，必填且为 quintic_rest_to_rest。
其 q_start_rad、q_goal_rad 均为限位内 F(6,)；端点速度、加速度固定为零。
strict 时指定时长违反约束直接失败，stretch 时允许延长但不可超过 max_duration_s；details 记录请求/实际时长。
不执行隐式关节角折返。

plan_cartesian 另含：

| 字段 | 类型、形状 | 必填 | 约束 |
| --- | --- | --- | --- |
| method | S，标量 | 是 | line_slerp_ik_quintic |
| path_samples | I，标量 | 是 | ≥2；基线 101 |
| max_joint_step_rad | F，标量 | 是 | >0；相邻 IK 解最大逐轴差，基线 0.1 |
| path_position_tol_m | F，标量 | 是 | >0；基线 1e-3 |
| path_orientation_tol_rad | F，标量 | 是 | >0；基线 1e-3 |
| validation_subsamples | I，标量 | 是 | 每段均匀细分数≥10；基线 20 |
| ik | 对象 | 是 | 完整 inverse options，同名字段语义相同 |

T_start、T_goal 均为合法 F(4,4)；q_seed_rad 为 F(6,)，先验证 FK(seed) 与 T_start 满足 ik 的双误差阈值，否则 1001。
平移直线插值，旋转沿最短旋转测地线；内部若用四元数固定 [x,y,z,w] 并统一相邻符号。
前一 IK 解作为下一初值；关节跳变超限或任一 IK 失败则不输出部分轨迹。每段关节五次多项式端点静止，保证 C2，但允许中间停顿。
这是可实现的保守首版；直线跟踪按段内采样验收，不宣称连续精确直线。路径误差超限返回 3001，后续可增加 path_samples 重新规划。

Trajectory 成功 data 是以下对象，不是裸采样矩阵：

| 字段 | 类型、形状 | 必填 | 约束 |
| --- | --- | --- | --- |
| time_s | F，(N,) | 是 | N≥2，首项 0，严格递增；包含精确终点，末步可短于采样周期 |
| q_rad | F，(N,6) | 是 | 采样关节位置 |
| qd_rad_s | F，(N,6) | 是 | 同一连续多项式的一阶导数 |
| qdd_rad_s2 | F，(N,6) | 是 | 同一连续多项式的二阶导数 |
| segment_times_s | F，(M+1,) | 是 | M≥1，首项0，末项等于 time_s 末项，严格递增 |
| coefficients_rad | F，(M,6,6) | 是 | 每段每关节升幂系数 c0…c5，以 u=(t-t0)/(t1-t0)∈[0,1] 求 q=Σc_k*u^k |
| collision_checked | B，标量 | 是 | 本版恒为 false |

采样必须与系数一致；qd、qdd 按段时长分别除以时长及其平方。控制采样任意时刻须求多项式及其导数，禁止对采样值作未定义的二次插值。
上述五次辅助基线内部拼接点保持同一 q/qd/qdd；正式三次及梯形扩展按第 7.5、9 节的分方法连续性规则执行。超出轨迹末端时应用编排发 complete 事件，不越界外推。

### 7.3 config、reference、measured、context

本辅助基线控制器定义速度前馈加位置比例反馈，仅覆盖 velocity 模式，不包含积分器或力矩环；正式增量 PID 及双前馈接口按第 7.5 节扩展。position/torque 命令格式仍保留，但本控制器请求这些模式返回 1008。

| config 字段 | 类型、形状 | 必填 | 约束 |
| --- | --- | --- | --- |
| config_id | S，标量 | 是 | 非空；运行中不可替换，换配置须停止后重新初始化 |
| mode | S，标量 | 是 | velocity |
| kp_s_inv | F，(6,) | 是 | 逐轴 >0，单位 1/s；合成积分仿真基线全 4 |
| nominal_period_s | F，标量 | 是 | >0；仿真基线 0.01 |
| period_tolerance_s | F，标量 | 是 | 0≤值<nominal_period_s；基线 0.002 |
| feedback_timeout_s | F，标量 | 是 | ≥nominal_period_s+period_tolerance_s；基线 0.03 |
| tracking_error_limit_rad | F，(6,) | 是 | 逐轴 >0；仿真基线全 0.05 |
| tracking_error_duration_s | F，标量 | 是 | ≥0；基线 0.05，0 表示首次超限即故障 |
| q_min_rad | F，(6,) | 是 | 从已加载模型逐项复制 |
| q_max_rad | F，(6,) | 是 | 从模型复制；须逐轴大于 q_min_rad |
| qd_limit_rad_s | F，(6,) | 是 | 0<值≤模型速度限值，应用编排负责交叉校验 |
| qdd_limit_rad_s2 | F，(6,) | 是 | 0<值≤模型加速度限值，应用编排负责交叉校验 |
| stopped_velocity_tol_rad_s | F，标量 | 是 | >0，且小于最小 qd_limit；基线 1e-3 |
| stop_timeout_s | F，标量 | 是 | >feedback_timeout_s；仿真基线 1.0 |

上述增益/周期为仿真验收配置，不能作为尚未确认设备的默认配置。输入、状态、反馈均校验后才允许计算命令。
q_cmd_velocity = qd_reference + kp*(q_reference-q_measured)，不对误差做 modulo。
超过 qd_limit 或相邻命令变化率超过 qdd_limit 时返回 4004，不默默裁剪；启动首条命令以当前实测速度作比较基准。
reference 的 q/qd/qdd 均须符合 config 的位置/速度/加速度限制；预测 q_measured+command*dt 超限则返回 4004，实测位置超限返回 1005。

| 对象.字段 | 类型、形状 | 必填 | 约束 |
| --- | --- | --- | --- |
| reference | 对象或 null | 是 | RUNNING 的 tick 必须为对象；其余事件/状态用 null |
| reference.q_rad | F，(6,) | 条件 | 参考对象存在时必填 |
| reference.qd_rad_s | F，(6,) | 条件 | 同上 |
| reference.qdd_rad_s2 | F，(6,) | 条件 | 同上；用于约束校验，首版控制律不用加速度前馈 |
| measured.q_rad | F，(6,) | 是 | 同模型零位/轴向/单位 |
| measured.qd_rad_s | F，(6,) | 是 | 实际反馈速度，不以零向量代替缺失反馈 |
| measured.time_s | F，标量 | 是 | 与 now_s 同源单调时钟；不得在未来 |
| context.now_s | F，标量 | 是 | ≥0；相邻调用严格递增 |
| context.event | S，标量 | 是 | tick、enable、start、complete、stop、reset |
| context.adapter_ready | B，标量 | 是 | 适配器已配置、速度模式及停止能力已确认 |
| context.stop_ack | B，标量 | 是 | 当前停止请求已完成的确认；必须对应本次请求，不接受旧确认 |
| context.external_fault_code | I，标量 | 是 | 0 或 contract.json 内非零错误码；适配器故障通常 5001 |

dt_s 为 F 标量 >0，且 |dt_s-nominal_period_s|≤period_tolerance_s；从第二次调用起还须与 now_s-last_step_time_s 相差≤1e-9 s，否则 1006。
反馈年龄须满足 0≤now_s-measured.time_s≤feedback_timeout_s；重复时间戳在未超时时允许，倒退为 1006，超时为 4002。
应用编排必须在每个周期提供真实时钟和实际间隔，不得以名义周期掩盖延迟。
start/complete/stop 等事件优先于轨迹采样；一次调用只接受一个事件。

### 7.4 state / next_state：持久数据完整定义

step 为纯状态更新：不得原地修改 state；成功 data.next_state 与 state 使用相同结构。状态不存放算法外的设备句柄。

| 字段 | 类型、形状 | 必填 | 约束 / 初始化 |
| --- | --- | --- | --- |
| schema_version | S，标量 | 是 | 1.1.0 |
| config_id | S，标量 | 是 | 必须匹配 config.config_id |
| mode | S，标量 | 是 | IDLE、READY、RUNNING、STOPPING、FAULT；初始 IDLE |
| step_index | I，标量 | 是 | ≥0，初始0；每次成功调用+1 |
| last_step_time_s | F 或 null，标量 | 是 | 初始化 null；成功时写 now_s |
| last_feedback_time_s | F 或 null，标量 | 是 | 初始化 null；成功时写 measured.time_s |
| hold_q_rad | F(6,) 或 null | 是 | 初始化 null；READY 时必为进入 READY 时的 measured.q_rad |
| last_command_qd_rad_s | F(6,) 或 null | 是 | 初始化 null；有命令时存该命令，无命令时清 null |
| tracking_violation_time_s | F，标量 | 是 | ≥0，初始0；READY/RUNNING 任一轴连续超限时累加实际 dt，全部恢复时清0 |
| stop_requested_at_s | F 或 null，标量 | 是 | 初始 null；STOPPING 必为首次请求停止的 now_s，重复 stop 不重置 |
| fault_code | I，标量 | 是 | 初始0；FAULT 必须是已定义非零码，其他状态为0 |

READY 用 hold_q_rad 作位置目标、零参考速度，输出同一比例控制律。RUNNING 用 reference；二者都执行跟踪/输出约束检查。
start 的那次调用只完成 READY→RUNNING 并维持 READY 保持命令，下一 tick 才使用轨迹首点；应用编排据此设置轨迹起始时刻。
complete 进入 READY，捕获当时实测位置作保持点；只有应用已采完终点且实测速度≤stopped_velocity_tol 才可发送 complete，否则 4001。

成功结果：data={command, action, next_state}；command 为 {mode:"velocity", qd_rad_s:F(6,)} 或 null；
action 为 none 或 request_stop。IDLE/STOPPING/FAULT 的 command 必为 null；STOPPING 和 FAULT 的 action 为 request_stop。
null 意为“本次无新运动指令”，不是“设备已经停下”。适配器按 request_stop 执行其已确认的停止机制，重复请求需幂等。

| 当前状态 | 事件 / 条件 | next_state | 输出及动作 |
| --- | --- | --- | --- |
| IDLE | tick；反馈健康且确认已停止 | IDLE | 无命令 |
| IDLE | enable；adapter_ready 且速度≤停止阈值 | READY | 捕获保持点，计算保持命令 |
| READY | tick | READY | 保持位置 |
| READY | start；adapter_ready | RUNNING | 本次保持，下一 tick 开始参考 |
| RUNNING | tick；参考、反馈及约束合法 | RUNNING | 跟踪参考 |
| RUNNING | complete；满足上述终点/停止条件 | READY | 捕获保持点 |
| IDLE / READY / RUNNING | stop | STOPPING | 无命令，request_stop，记录首次请求时刻 |
| STOPPING | tick 或 stop；未收到有效停止确认且未超时 | STOPPING | 持续 request_stop |
| STOPPING | tick 或 stop；stop_ack 且速度≤停止阈值 | IDLE | 清保持点、停止计时和误差计时 |
| STOPPING | 等待时间>stop_timeout_s | FAULT | 4001，details.reason=stop_timeout；继续请求停止 |
| FAULT | tick 或 stop | FAULT | 锁存原 fault_code，继续请求停止 |
| FAULT | reset；external_fault_code=0、adapter_ready、健康反馈、stop_ack 且速度≤停止阈值 | IDLE | 清故障和累计量，无命令；须重新 enable、start |
| 任意 | 新故障或非法事件/条件 | FAULT | 失败结果、停止请求；禁止运动命令 |

IDLE 的“确认已停止”由适配器初始化或当前停止确认加速度阈值共同建立；设备尚在运动时不得直接建立 IDLE。
除表内事件外均视为非法转换（4001），包括 reset 自动重启、RUNNING 中直接换配置等。
同周期优先级：先校验输入合法性（第5节），再处理外部故障、反馈/实测限位、当前状态适用的超时/跟踪故障（RUNNING 仅 tick 计算参考跟踪误差，stop 不要求 reference），再处理 stop，最后普通事件/命令。
FAULT 已锁存时保留首个 fault_code，后续故障可记录在 details，不得由 tick 清除。

**失败时状态交接（避免与 data=null 矛盾）：**
失败仍返回 {code, message, data:null, details}。details.action 固定 request_stop；
输入状态和时间可验证时，details.next_state 为上述同结构的 FAULT 快照（fault_code 为本次错误或已锁存首错，命令历史清 null）。
该字段仅用于诊断和状态持久化，不是可执行 data。应用必须先锁存本地 FAULT、停止参考下发、调用适配器停止，再接纳经过校验的快照。
若 state/config/时间本身损坏，details.next_state=null；应用保持外部故障锁存，停止后才能用健康配置和反馈重新初始化为 FAULT，不能回退到旧 RUNNING 状态。
异常抛出、进程中断和 I/O 失败也由编排/适配器独立停止路径处理；纯 step 无法替代设备看门狗。
reset 不绕过任何反馈/停止条件。上电初始化要求适配器已确认停止；初始化 state 使用表内初值，其后首个事件可以是 enable。

### 7.5 正式算法要求与接口扩展计划（待实现）

以下是计划增加的字段和返回结构，不修改现有可执行 C ABI，也不表示算法已实现。F/I/B/S 类型、单位、轴序及 Result 错误语义沿用前文；表内新增字段在相应方法分支启用时必填，除明确标注可选项外不得隐式取默认值。第 7.2 节的 DLS、静止端点五次和直线基线，以及第 7.3 节 P 控制，仅用于对照与分阶段验证，不能替代正式算法。

| 正式要求 / 拟扩展接口 | 拟增加的输入、类型与约束 | 拟增加的成功输出 | 补齐与确认节点 |
| --- | --- | --- | --- |
| UR5 解析 IK：inverse | options.method 增加 analytic_ur5；branch_policy:S，nearest_seed 或 all；q_seed_rad:F(6,) 作为选支/角度展开参考；joint_margin_rad、双误差阈值及 timeout_s 继续必填。解析分支不要求 damping/max_iterations 等数值专用字段。model 增加经核验的解析几何参数及映射修订号，具体字段待模型对齐后冻结 | 从裸 q 改为版本化对象：q_rad:F(6,) 为选定解；candidates_rad:F(K,6)，1≤K≤8，有限、限位内、去重并逐解 FK 回代；branch_ids:S(K,) 与 selected_index:I 对应。奇异族只返回可验证的有限代表解，不能声称枚举无限解；奇异选支策略待冻结。失败仍 data=null | 第2周周二冻结 URDF/DH 映射与参数字段；周四冻结选支、奇异与返回结构；周日完成接口及解析算法验收。数值 DLS 保留独立 method，不静默替代解析失败 |
| 三次/五次/梯形：plan_joint | method 增加 cubic、quintic、trapezoidal；cubic/quintic 增加 qd_start_rad_s、qd_goal_rad_s:F(6,)；quintic 另加 qdd_start_rad_s2、qdd_goal_rad_s2:F(6,)；均须满足模型限值。梯形首版明确静止端点，显式边界速度均为0，不接收加速度端值；沿用速度/加速度缩放、同步时长与 strict/stretch | Trajectory 增加 method:S、continuity:S（C1/C2）、segment_kind:S(M,)；保留统一 coefficients_rad:F(M,6,6)，三次高阶项补0，梯形每个恒加速/匀速子段用二次多项式高阶项补0。每个加速度切换必须进入 segment_times_s；零位移或零持续时间阶段合并，禁止重复时间 | 第3周周二冻结边界字段、三种方法同步策略与序列化；周四补齐接口校验及约束报告；周日三种方法分别验收 |
| 直线/圆弧：plan_cartesian | options.path_type:S，line 或 arc；arc 增加 T_via:F(4,4)，三点位置为起点/途经点/终点，非共线且不重复；arc_direction:S，through_via，选取经过途经点的有向弧；orientation_policy:S，piecewise_shortest_slerp，姿态经三个位姿分段插值；退化几何容差:F>0，以 m 为单位，数值待冻结；整圆另行扩展，不由重合端点猜测 | Trajectory 增加 path_type、路径几何元数据（圆心:F(3,) m、单位法向:F(3,)、半径:F>0 m、有向扫角:F rad）；提供路径位置/姿态最大误差、IK分支跳变检查结果及实际时间标度。圆弧角度与途经点顺序须唯一，失败不返回部分轨迹 | 第3周周二冻结三点弧与姿态/时间参数化；周四补齐路径输入输出及退化拒绝规则；周日直线、圆弧分别验收。分段 SLERP 不自动保证笛卡尔速度连续，须按路径时间标度验证 |
| 避障：plan_joint / plan_cartesian | collision_policy 增加 required；新增 planning_context，含 scene_id/revision:S非空、T_world_base:F(4,4)、碰撞几何来源、clearance_m:F≥0、检查分辨率及 checker_version:S。规划器算法、障碍物几何字段与连续/离散检查协议待第3周周二确认，不能把设备句柄塞入 model | collision_checked:B 仅实际检查后为true；新增 collision_report 对象：scene_revision、检查器版本、检查范围/分辨率、最小间距及碰撞计数。离散采样不得声称连续无碰撞；无检查器/无可行路径时返回现有适用错误码，data=null | 第3周周二冻结避障方法和场景协议；周四冻结检查/耗时及改善率指标；周日完成对应接口与避障验收 |
| 增量 PID + 速度/加速度前馈：step | config 增加 controller_type:S，incremental_pid；kp/ki/kd:F(6,) 非负并记录单位；kv_ff、ka_ff:F(6,) 显式配置；reference.q/qd/qdd 全部参与控制；output_mode、增益单位、控制律离散式、导数滤波参数和抗饱和策略在第4周周二一并冻结。不得沿用 P 基线的 kp_s_inv 含义猜测新增益 | command 保留 mode 与匹配单位的6维输出；next_state 增加 error_prev/error_prev2:F(6,) rad、pid_output_prev:F(6,)（单位随输出模式）、滤波记忆及 history_valid:B；初始化为无历史，并依据实测/当前输出建立首步值。READY/start/complete/stop/reset/FAULT 的历史保持或清除规则须逐事件冻结，禁止沿用过期积分/增量；可记录反馈/前馈分量与限幅诊断 | 第4周周二冻结控制模式、离散式、增益单位、状态迁移及抗饱和规则；周四补齐字段/状态契约和适配器接口；周六完成 PID、双前馈及故障验收。P 基线作为独立 controller_type 保留 |

轨迹扩展的连续性约定：五次仅在相邻段 q、qd、qdd 边界匹配时为 C2；三次匹配 q、qd 为 C1，不默认 qdd 连续；梯形速度轨迹为 C1，加速度允许在加速/匀速/减速切换处有有限跳变。梯形首末加速度不要求为零。非静止端点三次/五次不强制零端速；五次只在明确采用 rest_to_rest 时零端速、零端加速度。

在加速度跳点，采样 qdd 取右极限，最后时刻取左极限；轨迹元数据必须写 acceleration_sample_side="right_except_final_left"。验收同时检查左右极限，不把采样侧选择误判为 C2。位置/速度/加速度约束按每个连续子段及其左右边界验证。

扩展发布前须完成：字段级 schema 与 C 头文件/缓冲区容量约定、版本协商、旧调用方迁移、错误码映射（编号仍只由 contract.json 分配）、测试入口与指标映射。解析 IK 对应 KIN-03/06，三种关节规划对应 PLAN-01/02/04，直线/圆弧对应 PLAN-03/04，避障对应 PLAN-05，正式 PID 对应 CTRL-02/03/04。验收报告遵循[统一统计规则](statistics-rules.md)；若确认节点已过但未落实，标为逾期待确认，下一可工作日先补齐，不以基线结果抵扣。

## 8. 正式技术方案与确认节点

本表为当前唯一方案表；已确定的技术要求不再标为可选。环境安装/演示完成情况以已保存报告为准，算法实现情况以测试证据为准。[历史探索方案](history/engineering-plan-python.md) 不参与当前验收。

| 事项 | 正式方案 / 范围 | 状态与确认节点 |
| --- | --- | --- |
| 开发语言与工具 | 核心 C11 动态库；C++17 编译支持；Python 3.11.9 负责调用、仿真与测试；GCC/G++ 13.3.0、CMake 3.31.6，依赖见版本锁 | 技术栈已确定；最小 C/Python 调用已验证，业务算法待实现 |
| 仿真环境 | PyBullet 3.2.7，UR5 URDF，GUI 与 DIRECT；Linux/WSL 安装按环境说明，既有本地 GUI 证据独立记录 | 必需仿真环境，不是可选项；积分模型仅用于辅助单元测试，不能代替物理仿真验收 |
| 建模 | URDF 提供仿真模型；C 侧标准 DH + base/flange/tool 显式变换；质量、惯量、碰撞几何追溯到模型来源 | UR5 CB 名义参数与固定变换已于2026-09-17核对，BASE-02通过；加速度配置与实机标定不在本次关闭范围 |
| FK 与雅可比 | 齐次矩阵顺序累乘；基坐标轴构造几何雅可比并计入 joint_sign | 第2周周日验收，独立已知位形与中心差分核验 |
| IK | UR5 解析 IK 为正式要求；DLS 数值法作为对照和诊断 | 第2周周四冻结扩展接口和分支/奇异处理；周日验收，详见7.5 |
| 关节规划 | 三次、五次、梯形速度规划，显式边界与多轴同步，检查连续段极值 | 第3周周二冻结接口；周日三种方法分别验收，连续性按方法区分 |
| 笛卡尔规划与避障 | 直线、圆弧、连续选支 IK 和时间参数化；配置场景与碰撞检查的避障规划 | 第3周周二冻结路径/场景接口及避障算法，周四冻结验证细节，周日验收；不能把单纯直线采样视为避障实现 |
| 控制 | 增量 PID + 速度/加速度前馈；保留故障锁存、停止确认、周期和输出约束 | 第4周周二冻结模式/离散式/增益单位；周四补齐接口，周六验收；速度 P 仅作辅助对照 |
| 参考基准 | 独立环境 Robotics Toolbox 1.1.1，依赖版本与哈希锁定，float64 | 依赖准备及显式项目参数对齐已完成；内置UR5有d1差异，旧样本不可直接作为项目真值；C验收仍待实现 |
| 实机适配 | 协议、可用模式、停止确认、周期、看门狗及厂家限值 | 待设备资料和明确实机范围；资料不足时仅声明仿真能力，不影响上述必需仿真要求 |

确认节点按每周周二、周四、周六、周日安排；第4周周日汇总。逾期未确认项须标为逾期待确认，并在下一可工作日优先处理。执行者记录资料来源和决定，模型/任务书疑点向实习负责人核对；任何基线变更须同步接口、指标及变更原因，不得在测试失败后无记录地放宽阈值。

## 9. 量化验收计划

本节与[指标表](test-metrics.md)配套；误差/成功率/分层耗时统一遵循[统计规则](statistics-rules.md)。P 积分仿真行只验辅助基线；正式 PID/双前馈以及新增圆弧、避障须按7.5和相应指标单独验收。所有数值是拟验收门槛或性能目标，**本次未运行算法验收，不存在已通过的测量结果**。
验收记录至少含仓库 commit、模型 SHA-256、随机种子、操作系统、CPU/核数、内存、Python/依赖版本、完整 options/config、输入集合、最大误差、失败码分布和耗时分位数。
基线 seed=20260915；CPU 单进程、BLAS 单线程、关闭可视化与日志 I/O 的计时路径；浮点 float64。
阈值以米/弧度等原始物理量评价；不得用 IK 加权残差替代双误差验收。

| 项目 | 测试条件 / 样本 | 通过标准或目标 |
| --- | --- | --- |
| 单位与轴序 | 1000 mm→1 m、180°→π rad；六个互异轴值与其置换；错误/重复/缺失轴名 | 转换后数值绝对差≤1e-12；错误标记/轴名全部拒绝，编号符合常量表 |
| 数据校验 | 每个字段至少覆盖缺失、错误类型/形状、NaN/Inf、上下边界；矩阵与时间边界专测 | 非法输入拒绝率100%；失败 data=null；合法边界可接受，不输出伪成功 |
| FK 独立正确性 | 至少3个独立已知位形（含非单位工具变换、非零偏置、joint_sign=-1）；参考不得由同一 FK 生成 | 位置误差≤1e-9 m、旋转矩阵最大元素差≤1e-9；仅 IK 回代一致不能替代此项 |
| FK/IK 回代 | 已确认模型安全限位内100组 q 生成目标，seed 从 q 附近各轴±0.05 rad 生成且合法；另20个宽初值与20个近奇异样本单列 | 100组局部回代全部成功；norm2(Δp)≤1e-5 m，角误差≤1e-4 rad；不要求解关节等于原 q。宽初值/奇异组报告成功率，失败可为2002但不得假成功 |
| 姿态误差算法 | ΔR=R_target R_actualᵀ；θ=acos(clip((tr(ΔR)-1)/2,-1,1))；近0/π另测 | 采用稳定实现并交叉核对；IK 成功必须同时满足位置与θ阈值 |
| 雅可比中心差分 | 100组非限位 q、20组近奇异 q；每轴扰动 h=1e-6 rad；确保 q±h 合法 | Jv=(p+−p−)/(2h)，Jω=log(R+R−ᵀ)/(2h)；每元素 abs(解析−差分)≤1e-6+1e-4*abs(差分)，线性行单位 m/rad，角行 rad/rad |
| 关节轨迹连续约束 | 100组合法端点，含零位移、近限位、长短位移；至少10个 strict 时长不可行样本 | 每段位置导数根检查位置极值，速度导数根检查速度极值，加速度导数根检查加速度极值，并包含端点；位置容差1e-9 rad、速度1e-8 rad/s、加速度1e-7 rad/s²；不可行必须失败 |
| 轨迹一致性 | 系数/采样交叉比对；端点、每个拼接点左右极限；严格递增时间与精确终点 | q 误差≤1e-9 rad、qd≤1e-8 rad/s、qdd≤1e-7 rad/s²；五次在 q/qd/qdd 边界匹配时 C2（rest_to_rest 才要求零端速零端加速度）；三次匹配 q/qd 为 C1、qdd 可跳变；梯形为 C1、加速度切换允许跳变，首末加速度不强制为零。各方法按声明边界验收，跳点左右约束均检查，qdd 采样侧见7.5 |
| 笛卡尔路径 | 10条在确认可用工作区内的直线路径，每条101个路径节点、每段20等分检查；另加IK失败/关节跳变案例 | 每段将 FK(q(t)) 与该段相同五次进度的理想直线/旋转插值比较，采样位置差≤1e-3 m、姿态差≤1e-3 rad；不声称采样点间绝对保证；失败不交付部分轨迹 |
| 控制跟踪 | 积分仿真无噪声无延迟，dt=0.01 s，Kp=4/s，初始误差0；20条持续≥5 s的轨迹，规划速度/加速度缩放各≤0.5；先确认原始输出不会触限 | 每轴 RMSE≤0.005 rad、全程最大误差≤0.02 rad、结束保持1 s后误差≤0.001 rad；无故障、命令及其变化率不超限 |
| 状态与故障 | 覆盖表中每条合法转换及非法事件；注入反馈年龄>0.03 s、超差累计≥0.05 s、超限命令、停止超时、损坏 state、外部5001 | 条件达到的当次 step 返回对应错误并锁存 FAULT；所有错误零运动命令；编排同周期发停止请求；故障不自动解除，reset 后仍需 enable/start |
| 停止行为 | 仿真停止适配器按配置加速度减速并发送对应请求确认 | 发停止请求后一个名义周期内适配器接收；实际停止时间≤max_i(abs(qd_i)/a_stop_i)+2dt；若该上界超过 stop_timeout 则测试配置必须预先调整并记录 |
| FK / J 耗时 | 固定模型1000次预热，10000次测量，每次包含公开接口校验 | 各自 p95≤1 ms |
| IK 耗时 | 上述100组局部回代，各重复10次；计入所有失败/超时调用 | p95≤50 ms，单次超时预算200 ms；预算检查点设在每次迭代起止，超预算无成功输出 |
| 规划耗时 | 10 s关节轨迹，10 ms采样、1001点，重复100次；笛卡尔101路径节点单列 | 关节规划 p95≤100 ms；笛卡尔规划 p95≤5 s，预算10 s；报告失败及超预算，不隐去 |
| 控制计算耗时 / 周期 | 1000次预热，10000次有效 step（含校验/状态更新，不含 I/O）；另运行60 s含调度的墙钟实验 | step p99≤2 ms、最大≤10 ms；墙钟周期10 ms，99%间隔偏差≤2 ms，越界全部记录并触发1006；软件调度结果不等于硬实时保证 |

非限位随机采样区间取 [q_min+margin,q_max-margin]，margin_i=min(0.05 rad,0.1*(q_max-q_min))。
近奇异集合与宽初值集合在第2周周四根据模型显式固定并提交，不能仅挑选已成功案例；如果无法定义则相关验收标“阻塞”。
离线可达性目标由 FK 构造只能证明目标可达，不能保证任意初值的数值 IK 都收敛。
适配器必须用自身停止请求序号关联确认，再生成 context.stop_ack；step 接口不承担底层确认关联。stop_ack 的适配器仿真测试须注入旧确认并确认被拒绝；真实设备停止距离/响应时间须依据设备资料另建验收项。
性能暂以第1周周日记录的实际开发机为基准；记录环境前状态为“待测”，不能宣称跨设备性能达标。
依赖型号、动力学或实机资料的项目在资料缺失时标记“阻塞/未测”，不能用纯积分仿真结果替代。

## 10. v1.2 修订记录

- 将已被替代的 Python 探索方案移入历史归档，第8节只保留正式 C11 / PyBullet / UR5 技术方案。
- 第7.5节建立解析 IK、三次/五次/梯形、直线/圆弧、避障、增量 PID 与双前馈的接口扩展计划，列出输入输出、约束、状态历史及确认节点。
- 同步第3节模型状态、第7节基线适用范围和第9节连续性要求，区分五次 C2、三次/梯形 C1 及加速度跳变。
- 本次仅修订文档，未实现算法、未修改既有 C ABI/状态 schema、未新增错误码或宣称算法验收通过。

## 11. 实现状态更新（2026-09-17）

C矩阵运算、单节DH与forward已实现，语言无关forward由ctypes适配器映射至robot_forward低层ABI，详见[推导与接口](forward-kinematics.md)。本次仅使用完整模型的纯FK字段子集，未放宽控制模型要求。KIN-01/02功能检查已通过；本文其他尚未实现的接口仍为设计契约，C/FFI性能未测。

## 12. ABI正式冻结（2026-09-19）

正逆运动学低层签名、参数顺序、结构布局、分支编号与内存语义以[ABI 1.0.0](abi-v1.md)为正式C接口约束，第7.5节原C布局待冻结项由此关闭。既有FK接口不变，IK算法未实现，占位返回1008。新增[性能及鲁棒性验收](abi-v1-validation.md)，不以共享CI绿色替代性能报告。
