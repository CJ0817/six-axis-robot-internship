# 逐候选正运动学回代与过滤

2026-09-25。实现位于 `src/kinematics/inverse.c` 的私有 `filter_candidate`，由 `robot_inverse_v1` 对每个普通解析根调用；没有新增导出符号、字段或错误码，ABI 1.0.0不变。

## 筛选顺序与判据

1. 检查六个DH角有限；NaN/Inf候选剔除。
2. 依据joint_sign和theta_offset恢复逻辑关节角，在 `[q_min+margin,q_max-margin]` 内选择距seed最近的2π等价表示。没有合法表示的候选剔除。只容许≤1e-12 rad的端点舍入吸附，不能把真正越限解硬截断到限位。
3. **每个有限、合法限位候选均调用已有C `robot_forward`**，得到base→tool0矩阵，包含完整固定基座/法兰/工具变换。非法或越限输入先拒绝，不作为FK输入。
4. 位置误差为两平移列的欧氏距离；姿态误差为 `R_fk^T R_target` 的旋转角，用atan2计算，避免小角度acos失精度。两者必须有限，并分别≤options.position_tol_m和orientation_tol_rad；默认验收阈值1e-5 m、1e-4 rad，不合格候选剔除。
5. **回代通过后去重**：与已保留的规范化候选相比，六轴最大绝对差≤1e-9 rad则丢弃重复项。2π等价分支先归一为距seed最近的合法表示，再比较；先出现的回代不合格候选不会压掉后续合格候选。
6. 将剩余候选按字典序排序；all返回所有通过上述筛选的离散代表，nearest_seed返回距seed最近的一个。所选解在成功写回前再次FK核验。

输入目标仍为16个行主序double，seed为6个rad关节角。结果的solution_count仅统计最终有效候选，q_rad与selected_index指定行相同；无效槽清零，branch_ids无效槽UINT32_MAX。失败时整个输出保持原字节，调用方不得读取旧solution_count当作本次结果。

## 本次修复与失败分类

此前只要一个候选回代不合格，就设置整次求解的numerical标记，导致已有合格解也被丢弃。本次将“枚举无法完成”和“已枚举候选不合格”分开：完成枚举后，只要存在合格解就正常返回筛选结果。这里all意为**所有通过当前限位和误差约束的离散代表**，不是把不合格代数根也输出。

| 筛选后状态 | 返回码 |
|---|---:|
| 至少一个有效候选 | 0 |
| 无有效候选，存在非有限/回代不合格候选 | 2002 |
| 有几何根，但所有等价表示均越限 | 1005 |
| 全部几何分支均被不可达条件排除 | 2001 |
| 枚举本身数值异常或预算耗尽 | 2002，不返回未完成枚举的部分结果 |

精确腕奇异仍沿用上一任务策略：all返回2003；nearest仅在seed自身已匹配目标时允许返回seed，否则2003。连续族全局选解不是此次过滤功能的完成项。输入错误码、内存契约及失败原子性保持冻结ABI约定。

## 针对性证据

原生C测试直接包含生产实现，以调用同一个私有过滤函数，不在测试里复制算法；独立测试可执行文件不改变动态库的导出ABI。

16项筛选检查全部通过，覆盖：位置失败、仅姿态失败、坏候选之后保留好候选、完全重复、2π重复、去重阈值内/外、NaN/Inf、没有合法等价角、余量排除端点、混合集合有效数量、公开接口普通输出、部分回代失败、所有保留项复核、全部回代失败且输出不变。

公开接口专项从基线8候选的实际姿态误差分布中设置中间阈值。此次-O2运行阈值6.1421238246335785e-16 rad，保留7/8；不是重新放宽验收要求，而是有意制造“部分合格、部分不合格”来验证过滤。阈值随平台浮点舍入自适应，测试断言保留数量介于0和原数量之间，不硬编码7。

- 筛选专项：`results/filter-verified/ik_filter/report.json`，16/16通过。
- 原有回归：`results/filter-ik-regression/ik/report.json`，normal100/100、wide_initial20/20、near_singular20/20，52专项通过。
- GCC13.3.0严格C11编译通过；CMake3.31.6 Release构建CTest5/5通过，日志`results/filter-verified/ctest.log`；ABI布局、符号、错误码和失败输出契约回归通过。

本轮使用云端Linux x86_64与Python3.12.14。未更改冻结模型或目标文件；无需第三方C数学依赖。本轮不重新评定性能，也不把先前FK FFI性能问题标为关闭。

## 复现

```bash
# 使用既有环境配置，CMake同时构建共享库和原生过滤测试
.venv/bin/cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
.venv/bin/cmake --build build -j2
.venv/bin/ctest --test-dir build --output-on-failure
# 统一入口分别保存过滤专项与140目标回归
python3 scripts/run_tests.py --suite ik_filter --smoke-python "$(command -v python3)" --output results/test-runs/ik-filter
python3 scripts/run_tests.py --suite ik --smoke-python "$(command -v python3)" --output results/test-runs/ik
```

新增指标IK-FILTER，已接入available、acceptance、CTest及CI。原生过滤专项进程有10 s超时，统一入口另有120 s进程预算；两层分别记录实际等待时间。正式误差与成功率继续按原统计规则，失败不能填作零误差。
