# ABI冻结与FK性能、鲁棒性验证

日期2026-09-19。导师三项反馈对应：正式ABI v1、FK分层耗时基线、15边界+10异常专项。

## ABI

robot_fk_model保持704字节及原字段偏移，robot_forward参数顺序不变。新增IK options72字节、result504字节、robot_inverse_v1及版本查询；候选固定8个，避免容量参数反复变更。IK尚未实现，始终返回1008且不改输出。编译期断言和ctypes测试验证布局、符号、错误码、版本与占位不写输出；现有C++/Python ABI、C矩阵/已知位形CTest3/3通过。

## 性能条件及门槛

同一批既有115组q来自results/model-audit/samples.json；两层分别预热1000次，每组测100次，共11500次。编译GCC13.3.0、CMake Release（-O3 -DNDEBUG）；Linux x86_64，Python3.11.9；CPU INTEL(R) XEON(R) PLATINUM 8573C，可见逻辑CPU数9。单进程顺序运行，不绑核；共享云端负载未受控，包含调度影响，不代表硬实时保证。

门槛在本次运行前设为：原生C与FFI各自p95≤1 ms，单次≤1 ms；只有两层全部满足才整项通过。超限数量和最大值不丢弃，不在失败后调整阈值。

| 层级 | 次数 | 平均(ms) | p95(ms) | 最大(ms) | >1ms次数 | 判定 |
| --- | --- | --- | --- | --- | --- | --- |
| C函数 | 11500 | 0.000460240695652 | 0.000486 | 0.025369 | 0 | 通过 |
| Python→C | 11500 | 0.00164840826087 | 0.001386 | 2.046632 | 1 | 单次最大值未通过 |

**整项性能状态failed**：FFI有一次2.046632 ms长尾，C函数本身达到当前门槛。该观察不能仅凭墙钟数据归因于C算法、解释器或调度中的某一项，登记PERF-01待进一步定位；没有删异常值或重跑挑选更快结果。

C时钟CLOCK_MONOTONIC，函数调用前后取时间差，包含公开接口校验，不含输入生成、日志与输出统计；计时开销不扣除。FFI用perf_counter_ns，只围绕ctypes调用，模型/数组和指针事先准备，包含FFI转换、C执行及调用返回，不含JSON封装。完成的11500次调用均返回0，无因求解失败而剔除样本。

原生进程实际等待0.009052973 s，包括启动、预热、计时和CSV输出；30 s超时预算，超时0。父脚本工作墙钟0.068664576 s，起点在参数解析后、止于最终报告写入前，**不是操作系统意义的完整进程启动耗时**。全进程边界可通过统一入口process_wall_s测量，本次不将它与C/FFI混报。

分位数为线性插值h=(n-1)p。原始每次耗时（含异常长尾）见results/fk-performance/c-function.csv和python-to-c.csv；环境、模型/样本/库/脚本哈希与RMSE见report.json。本轮最后编译产物的SHA256仍与测量时相同，未用不同二进制替换证据。

## 鲁棒性

15组边界：J1～J6每轴上下限12组，加全下限、全上限、零位奇异姿态；全部返回0并与显式项目参数RTB比较，矩阵最大元素误差≤1e-9。

10组异常：NULL模型/关节指针/输出指针；长度0/5/7；NaN、Inf；低于下限1e-6 rad、高于上限1e-6 rad。模型缺失返回1004，参数异常1001，越界1005；每个独立子进程10秒预算，全部按预期结束，无崩溃、无超时。有有效输出缓冲区的案例均检查逐字节不变。所有测试仅传合法分配的指针或NULL，不用悬空指针制造未定义行为。

结果15/15边界、10/10异常通过；详细输入、错误码和等待时间见results/fk-robustness/report.json。

## 重跑及状态

```bash
# 按现有环境脚本配置并用Release编译后
.venv/bin/python -m unittest discover -s tests -p test_abi_v1.py -v
.venv-baseline/bin/python scripts/run_tests.py --suite fk_robustness
.venv/bin/python scripts/run_tests.py --suite fk_performance
```

现有8项功能检查全部通过；性能专项独立保留failed。available只含功能项；acceptance含性能检查。CI性能步骤在共享主机上为证据任务（continue-on-error），始终上传结果，不能据此将未达标项标为关闭。

ABI已冻结，边界/异常专项已完成，FK耗时基线已建立。尚未关闭：PERF-01的FFI单次长尾、雅可比与逆解实现；不宣称所有性能要求均已满足。
