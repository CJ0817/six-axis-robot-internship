# 运动学阶段正式测试报告（2026-10-06）

范围：UR5 CB **标称仿真模型**的 C11 正运动学、解析逆运动学和 Python 连续选解。数值均来自仓库内逐例报告；本报告统一验收口径，不把不同测试群体合并为重复样本。固定接口见 [ABI 1.0.0](../../docs/abi-v1.md)，模型见 [参数与来源](../../docs/ur5-model-conventions.md)，统计规则见 [v1](../../docs/statistics-rules.md)，指标源见 [tests/metrics.json](../../tests/metrics.json)。

## 条件、阈值与判定

| 项目 | 测试条件和事先确定的阈值 | 证据与本阶段判定 |
|---|---|---|
| 模型对齐 | 115 个固定位形/逐轴/随机样本；C 对显式参数 RTB 和 URDF，位置及旋转矩阵元素差分别≤1e-9 m、≤1e-9 | [FK 115 原始报告](../../results/fk-validation/report.json)：115/115；对 URDF 位置最大 2.11728e-10 m、矩阵元素最大 5.71722e-10，**通过**。内置 RTB UR5 的 d1 相差 0.0003 m，不作工程真值。 |
| FK 独立正确性 | 至少 3 个独立位形，含非单位固定变换、符号与零偏；位置≤1e-9 m、矩阵元素≤1e-9 | C 单元测试 3/3，[FK 对照](../../results/fk-rtb-comparison/report.json)和上述 115 组通过；10 个指定已知角 C 对显式 RTB 最大位置 1.388e-17 m、姿态 4.137e-17 rad。 |
| IK 正确性 | 每目标有限、限位内、成功码 0；全部保留候选逐一 C FK 回代，位置≤1e-5 m、姿态≤1e-4 rad；普通组 100% | [120 工作空间逐例](../../results/workspace120-batch-run/workspace120_batch/report.json)：120/120，890 候选全部合格；[140 专项逐例](../../results/closeout-ik/ik/report.json)：普通 100/100、宽初值 20/20、近奇异 20/20，合计 140/140。均**通过**。 |
| IK 边界/异常 | 16 个边界及 11 个异常按预期状态与错误码判定，失败输出不变 | 16/16 与 11/11 符合预期；4 个精确腕奇异请求返回 2003，**不计为成功逆解**。另有 52 项 C IK 专项在原始报告中通过。 |
| FK 单次耗时 | 115 个冻结输入；C 和 Python→C 各 1000 次预热、每输入 100 次，共各 11500 次；各层 p95≤1 ms，**每一次**≤1 ms，零超限才整项通过 | [FK 原始耗时](../../results/fk-performance/report.json)：C 平均/p95/最大 0.000460/0.000486/0.025369 ms，0 次超限；Python→C 0.001648/0.001386/2.046632 ms，**1 次超限，整项未通过**。保留 PERF-01，不能因 p95 合格删除长尾。 |
| IK 单次耗时 | 功能观察中每次预算 200 ms；正式性能验收须普通 100 个输入每个重复 10 次、p95≤50 ms，全部失败/超时计入 | [140 组汇总](../../results/ik-summary/summary.json)仅每目标一次、无预热：C 平均/p95/最大 0.06494/0.09935/2.09753 ms，Python→C 0.07121/0.11188/2.10548 ms。全部低于 200 ms，但**正式重复性能验收未运行**。 |
| 连续选解与奇异退化 | 限位、每轴单步 `min(0.15, qd_max×0.02)` rad，近奇异按 0.25 收紧；sigma 软/硬阈值 0.02/1e-5；DLS运动解须保留原误差容差和路径保护；原位保持单列 | [选解退化逐例](../../results/ik-selection-dls/summary.json)：103/103，附加 6/6 检查通过；精确奇异目标附近有 1 例经阻尼返回非奇异容差内解，关闭阻尼和较远起点各按预期返回 2003。此项是上层局部策略验证，不替代解析奇异连续族穷尽或完整规划验收。 |

FK 性能报告的状态为 `failed`，因此**本阶段功能精度通过，整体性能不得标为全部通过**。C 雅可比解析实现、轨迹规划、碰撞/动力学、控制器和真实机器人运行尚未验收。上表的 IK 耗时是描述性观测；数值长尾不删除、不事后放宽门槛。

## 可复算的误差和耗时

[机器可读汇总](../../results/ik-summary/summary.json)保存两份原始报告的 SHA-256（空间120为 `79c45f7a5e74a52812ea189d65a347f03b9e00610a2177fae835247177210a9d`，专项140为 `1aa001564c9f6055847787f8ce333f14c593c2d67a443339067cd3dd76fbc31e`），并保留有效数、均值、RMSE、p95、最大值和无效数。空间120的误差分母是 **890 个候选**：位置均值/RMSE/p95/最大为 2.051e-16/2.493e-16/4.959e-16/9.587e-16 m；姿态为 5.016e-16/5.781e-16/1.024e-15/1.946e-15 rad。专项140的误差按**每目标最大候选**计：位置 3.801e-16/4.047e-16/6.776e-16/7.745e-16 m，姿态 8.815e-16/9.310e-16/1.387e-15/2.042e-15 rad。

成功率分母固定为应执行目标数；无有效输出者仍计入失败，误差为 null，不填零。p95 按排序后 `h=(n-1)×0.95` 线性插值。C 函数计时使用 `CLOCK_MONOTONIC` 且包含公开接口校验，Python→C 使用已备参数的 `perf_counter_ns`，整个测试进程另计，不混报。FK 功能115、FK 性能11500、IK 空间120和专项140属于不同样本/重复协议。历史云端共享主机不提供硬实时或跨机器耗时保证。

## 奇异位姿的可用行为

C `robot_inverse_v1(method=analytic_ur5, branch_policy=all)` 在精确腕退化连续族不能完整枚举时返回 2003；C `method=2` DLS 仍返回 1008，**冻结 ABI 未改**。[Python 状态选解层](../../docs/ik-stateful-selection.md)已有奇异代价、近奇异限步、路径采样和腕部内部穿越拒绝。本轮增加仅在解析 2003 或候选因奇异保护全部拒绝时触发的局部阻尼退化。它先约束原状态±步长和关节限位，再以数值雅可比、自适应阻尼、线搜索寻找可回代的非奇异解；输出逐项实际残差、sigma 与迭代诊断。上述测试中的成功是**位姿容差内的非奇异近似**，不是精确奇异连续族的全部解。预算耗尽、限步不可行、交叉硬奇异或无合法解时仍失败且不推进 `next_state`；调用者必须停下该步并重规划，不能把失败当作成功轨迹节点。

## 复现与未关闭项

```bash
# 依据 docs/environment-setup.md 安装锁定环境并完成 Release 构建
python scripts/verify_ik_selection.py --library build/librobot_contract.so --output results/ik-selection-dls
python scripts/run_tests.py --suite ik_selection --output results/test-runs/ik-selection
python scripts/summarize_ik_evidence.py --workspace-report results/workspace120-batch-run/workspace120_batch/report.json --ik-report results/closeout-ik/ik/report.json --output results/ik-summary
```

复现耗时会改变哈希和统计数值；须重新保存逐例数据与汇总。未关闭：PERF-01 的 Python→C 单次 2.046632 ms 长尾及独立硬件条件下重复性能测试、精确腕奇异连续族全局选支、肩/肘连续路径保证、C 雅可比，以及后续规划/控制。局部阻尼属于纯仿真规划接口，尚无加速度限值、碰撞检查或实机安全证明。
