# 工程边界与数据约定 v1.1.0（方案与接口细化）

本文件定义本实习工程的接口规范，不代表算法或设备控制已经实现。字段采用语言无关表示，拟实现技术栈见第 8 节；机器可读常量及错误码以 [contract.json](../src/common/contract.json) 为唯一编号来源。

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

使用右手坐标系。T_A_B 表示把 B 系坐标转换到 A 系：p_A = T_A_B · p_B（齐次列向量）。
位姿接口默认使用 T_base_tool，4×4，左上为旋转矩阵 R，右上为以米为单位的平移，末行为 [0,0,0,1]。
R 应满足 RᵀR≈I、det(R)≈1；校验容差必须显式配置并记录。工具相对法兰的变换必须来自模型，不默认二者重合。
雅可比 J 为 6×6，满足 [v_base; ω_base] = J(q) qd，前三行为工具原点线速度，后三行为角速度，均在 base 系表达。
对纯 FK 求值，奇异位形本身不必报错；只有请求无法完成时返回 KINEMATIC_SINGULARITY。数值 IK 未收敛不能直接认定不可达。

机器人型号、轴正方向、零位偏置、关节限位、速度/加速度限值、工具变换尚待确认；暂定标准 DH 建模，确认节点见第 8 节。
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

本节为规范性定义。v1.1 为 step 增加必填 context，并允许无运动命令的成功状态转换；旧调用方必须升级。
contract.json 的 1.0.0 是共用常量/错误码表版本，本次不新增、不修改编号；本文接口版本独立为 1.1.0。
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
内部拼接点同一 q/qd/qdd；超出轨迹末端时应用编排发 complete 事件，不越界外推。

### 7.3 config、reference、measured、context

本版控制算法采用速度前馈加位置比例反馈，仅实现 velocity 模式，不包含积分器或力矩环。position/torque 命令格式仍保留，但本控制器请求这些模式返回 1008。

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

## 8. 技术方案与确认节点

以下为拟实施基线，尚无算法实测结果；未获得原始实习文件的新增约束确认，不能把暂定选型视为实习方已批准。
确认节点按可工作日：第1周周四冻结技术栈，第1周周六冻结模型，第1周周日冻结基准环境；第2周周日运动学验收，第3周周日规划验收，第4周周六控制验收、周日汇总。
若相应节点已过而尚未确认，则列为“逾期待确认”，下一次可工作日先处理，不补写为已完成。

| 事项 | 拟采用方法 / 范围 | 状态与确认节点 |
| --- | --- | --- |
| 开发语言 | Python 3.11、float64；NumPy 做矩阵运算、pytest 做验证、Matplotlib 出图；依赖精确版本写锁文件 | 暂定；第1周周四对照实习要求确认，若指定 MATLAB 则在实现前调整技术栈 |
| 开发/仿真环境 | 首版 Windows 11 或 Ubuntu 22.04 的 Python 虚拟环境；离散关节积分仿真 q[k+1]=q[k]+dt*qd_cmd[k]，同一工程记录轨迹和三维连杆图 | 暂定；第1周周四确认实际系统，第1周周日记录 CPU/RAM/版本；没有惯性、摩擦或接触，不能验证真实动力学 |
| 可选物理仿真 | 若实习要求动力学/碰撞，再评估 PyBullet 与 URDF，补充惯量、质量、碰撞网格及适配器 | 待定；第1周周四决定是否纳入，未决定不作为已具备能力 |
| 建模 | 六旋转关节标准 DH，加基座/法兰/工具显式变换；JSON 模型；所有参数可追溯 | 方法暂定；第1周周六确认机器人型号、DH 表、零位/方向/限值/工具。缺项阻塞该机器人验收 |
| 正运动学 | 顺序累乘齐次矩阵，独立已知位形/独立实现交叉核验 | 拟采用；第2周周日前验收 |
| 几何雅可比 | 根据 base 系各关节轴和工具原点构造线/角速度列，计入 joint_sign | 拟采用；用中心差分验证 |
| 逆运动学 | 限位约束下阻尼最小二乘、步长限幅与回溯；位置/旋转向量误差双判据 | 拟采用；解析 IK 是否增加待第2周周四根据机器人结构决定，不作为首版依赖 |
| 关节轨迹 | 同步五次时间标度 s(u)=10u³-15u⁴+6u⁵，q=q0+(q1-q0)s；端点静止，按逐轴速度/加速度峰值定总时长 | 拟采用；第3周周日前验收，连续极值用多项式导数根检查 |
| 笛卡尔轨迹 | 直线平移+旋转最短路径，连续初值 IK，关节分段五次插值；检查跳变及工具路径偏差 | 拟采用；首版允许路径节点停顿。连续不停顿时间参数化为待定扩展，第3周周四决定 |
| 控制 | 速度前馈+位置 P，10 ms 仿真周期；显式状态机、超时/超差/输出约束及故障锁存 | 拟采用；第4周周六验收。PID、力矩/重力补偿不在首版，若要求必须先补动力学及新配置契约 |
| 实机适配 | 协议、可用命令模式、停止确认、设备周期、看门狗及厂家限值 | 待定；若第1周周日仍无设备资料，本轮限定仿真交付，不声明实机实时性 |

确认事项由实习执行者记录资料来源和决定；机器人参数/实习指定环境需向实习负责人核对。
更改本表基线必须同步更新接口、测试配置及阈值变更原因，不能在测试失败后无记录地放宽指标。

## 9. 量化验收计划

所有数值是拟验收门槛或性能目标，**本次未运行算法验收，不存在已通过的测量结果**。
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
| 轨迹一致性 | 系数/采样交叉比对；端点、每个拼接点左右极限；严格递增时间与精确终点 | q 误差≤1e-9 rad、qd≤1e-8 rad/s、qdd≤1e-7 rad/s²；起终点零速零加速度；所有段 C2 |
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

## 10. 本次修订检查记录

- 已将 model、各接口 options、config、reference/measured/context、state/next_state 与 Trajectory 补成字段表。
- 已明确 velocity 首版控制律、六类事件、五种状态、停止确认、故障锁存、复位条件与 data=null 的失败状态交接。
- 已列出拟采用技术栈、建模/算法方案、未定事项和可工作日确认节点。
- 已给出精度、约束、跟踪、周期、耗时目标及可复现实验条件。
- 本次交付范围为规范修订；算法、仿真适配器、测试程序和测量报告仍待对应开发阶段实现。
