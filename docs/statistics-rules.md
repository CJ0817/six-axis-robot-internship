# 统一统计规则 v1

本规则适用于后续运动学、规划、控制基准；以 `statistics_version: 1` 标记新报告。既有依赖自检报告不追溯改写为完整算法验收。每轮先冻结测试清单、分组、阈值、每样本预算、预热次数和计时重复次数，并保存 seed、输入、配置、提交、模型/依赖锁哈希、硬件与线程数。

## 1. 样本与误差

一次预定的输入及求解预算构成一个样本。普通初值 `normal`、宽初值 `wide_initial`、近奇异 `near_singular` 三组分别汇总，不可运行后更改分组；总体统计由所有原始样本合并计算，不能平均各组 p95 或成功率。性能重复测量另计，不增加求解成功率的分母。

位置误差：`position_error_m = ||p_actual - p_reference||₂`。
姿态误差：`orientation_error_rad = acos(clip((trace(R_referenceᵀ R_actual)-1)/2,-1,1))`，取值 [0,π]。计算前检查旋转矩阵形状、有限性、正交性及 det≈+1；姿态合法性容差固定为 1e-9。IK 使用返回关节解经 FK 回代的位姿。单位分别为米和弧度，不混合成一个误差分数。

每种误差分别输出以下字段，统计域为具有可验证、有限且非负误差的样本，**包括误差超限或约束失败但仍有有效位姿输出的样本**：

| 字段 | 定义 |
|---|---|
| `valid_count` | 实际参与该误差统计的样本数 n |
| `invalid_count` | 应执行总数 N − n；缺失、非有限或无法验证输出 |
| `mean` | Σe / n |
| `rmse` | sqrt(Σe² / n)；对每样本标量误差求均方根 |
| `p95` | 下述统一线性插值分位数 |
| `max` | 最大标量误差 |

n=0 时四项数值全部写 JSON `null`，不可写 0、NaN、Infinity；n=1 时四项均等于该误差。有效误差计数不等于成功计数。超时被终止、异常、无输出、错误码明确使结果无效的样本不计算误差。禁止复用上一样本输出，禁止将失败样本填成零误差。

## 2. 成功率与失败计数

`success_rate = success_count / expected_count`，其中 `expected_count` 为预先冻结的应执行样本总数；分子必须同时满足：返回码成功、输出形状/有限性及姿态合法、收敛、位置与姿态各自不超过冻结阈值、所有适用的关节/速度/加速度/碰撞等约束通过、在预算内完成。不适用的约束在测试配置中事先列明，不能临时省略。

超时、异常、未收敛均计入失败；中途终止导致未尝试的样本仍留在分母并计为 `not_run`。同一输入自动重试不能将多个失败从分母移除；允许的内部迭代/重启应事先计入该样本总预算。完整算法尚未实现时标 `blocked`，不虚构一批全失败或全成功结果。

每组及总体输出 `expected_count`、`attempted_count`、`success_count`、`failure_count`、`success_rate`、`failure_counts`；必须满足 `success_count + failure_count = expected_count`。N=0 时成功率为 null，标记 `no_samples`，不得宣称通过。

`failure_counts` 使用互斥主原因，按优先级归类：`not_run` → `timeout` → `exception` → `return_code` → `nonconverged` → `invalid_output` → `error_limit` → `constraint_violation`。额外原因可记录在逐样本 `failure_flags`，但不重复增加主失败计数。普通、宽初值和近奇异组均须单独报告以上计数、误差和耗时；缺少某一必测组时整体验收不完整。

## 3. 耗时边界

| 层级 / 字段 | 计时起止与包含范围 |
|---|---|
| `c_function_ms` | C 层使用单调墙钟，在被测函数调用前取 t0，返回后立即取 t1；排除输入生成、Python、结果验证及日志。记录钟源/分辨率，不减去估计的计时开销。当前最小 C 库未接入此计时，填 null 并标 not_measured。 |
| `python_to_c_ms` | Python `perf_counter_ns()` 在调用 ctypes 函数前至返回后取差；数组与 argtypes/restype 事先准备，包含 FFI 转换与 C 执行，排除加载动态库和输入构造。未接入时标 not_measured，不能以进程耗时替代。 |
| `process_wall_s` | 父进程 `monotonic()` 在 subprocess.run 前至正常返回或异常处理入口；包含进程启动、导入、预热、计算、输出，以及超时终止和回收。排除父进程报告解析/写入。 |
| `suite_elapsed_s` | 测试入口解析参数前至汇总写出前，包含所有子进程、准备、验证与汇总；不含安装、编译及最终汇总文件写盘。 |

现有 RTB `reference_fk_p95_ms` 是 Python 参考函数 `fkine` 的计时，包含其返回对象 `.A` 提取，既不是 C 函数耗时，也不是 Python→C ABI 耗时；只作信息记录。

函数/FFI 性能报告必须分别给出 `completed_count`、`timeout_count`、`exception_count`、`not_run_count`、`mean`、`rmse`、`p50`、`p95`、`p99`、`max` 和 `actual_wait_total_s`。完成调用即使求解失败也纳入完成耗时分布，成功子集如另报必须注明分母。超时是截尾数据，不把预算上限当成完成耗时放入分位数；独立列出预算、实际等待及终止开销。无完成样本时数值为 null。

所有分位数采用同一算法：将 n 个观测升序排列 x；p∈[0,1]，h=(n−1)p，i=floor(h)，j=ceil(h)，q=x[i]+(h−i)(x[j]−x[i])。这是线性插值规则；不得混用 nearest-rank。p95 的 p=0.95。不得先四舍五入再算统计量；原始值保存，展示时再舍入。

预热样本不参与性能分布或正确性成功率。记录实际预热数、重复数、线程数、是否包含初始化。C FK/J 性能沿用指标表 1000 次预热、10000 次计时；RTB 依赖自检现有 10 次预热、100 次计时只代表准备检查，不替代性能验收。

## 4. 超时与原始记录

未来单样本 C/IK 超时测试需使用独立工作进程隔离：父进程等待设定预算，超时后终止并回收，再记录实际经过时间。仅 Python 线程超时不能保证 C 调用已停止，不可用于硬超时证据。样本计时预算与整个测试进程 120 秒上限是不同层级，分别记录。

逐样本记录至少包括：`sample_id`、`group`、`attempted`、`return_code`、`converged`、`finite_output`、`constraints_passed`、`success`、`failure_reason`、两种可空误差、各层级可空耗时、`timeout_budget_s`、`timed_out`、`actual_wait_s`。`actual_wait_s` 从父进程开始启动/等待该工作单元至正常完成或终止回收，不能固定填预算；纯启动异常也记录已耗时间并计异常。

现有统一入口新增进程层 `process_wall_s`、`timeout_budget_s`、`timed_out`、`timeout_count`、`actual_wait_s`，以及汇总层 `suite_elapsed_s`、`process_timeout_count`。`elapsed_s` 保留为兼容字段，包含父进程准备和报告解析，不能当作 C/FFI 耗时。汇总的进程超时数不能冒充算法样本超时数。

## 5. 审核算例

某组应执行 5 个样本：3 个返回有效位置误差 [0, 0.003, 0.004] m，另 1 个超时、1 个无输出；其中 0.004 m 超过位置阈值 0.003 m，其他成功条件均满足。位置 `valid_count=3`、`invalid_count=2`、mean=0.0023333333333333335、RMSE≈0.002886751345948129、p95=0.0039、max=0.004；`success_count=2`、`failure_count=3`、成功率 2/5=0.4。超限的 0.004 仍参与误差统计，超时及无输出均不填零。
