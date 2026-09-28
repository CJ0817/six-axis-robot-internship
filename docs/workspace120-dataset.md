# 固定种子120组工作空间样本

2026-09-28。新增主组 `tests/ik/workspace120.json`；另列 `tests/ik/workspace120-special.json`。既有140组IK目标、40组目标分类测试及其报告不修改、不合并分母。

## 主组与覆盖规则

随机种子固定为 **20260928**，使用Python `random.Random`（MT19937）。每轴在已核对UR5 CB逻辑限位内收缩0.05 rad后均匀抽取q；所有角按J1～J6、rad保存。用标准DH独立Python实现生成base→tool0目标4×4矩阵，保留原模型的符号、零偏及固定变换。

以base中的肩部参考点 `(0,0,d1)=(0,0,0.089159)` m为分区原点，使用目标平移相对该点的X/Y/Z正负号划分8象限（零归正侧）；到该点的欧氏距离r划为：

| 半径档 | 区间(m) | 每象限数量 | 总数 |
|---|---|---:|---:|
| near | 0≤r<0.35 |5|40|
| middle | 0.35≤r<0.65 |5|40|
| outer | r≥0.65 |5|40|

24个空间区各接受首先到达的5个样本，共120组；本次抽取250次完成配额，已满区的后续样本丢弃。不调用IK，不按求解成功、耗时或误差选择“容易通过”的目标。这是**空间分层样本**，不是最终120组在关节空间或笛卡尔体积中的均匀分布；也不宣称穷尽工作空间或姿态空间。

每条主样本包含 `target_id`、`region`、`radius_from_shoulder_m`、`q_rad[6]` 和 `target_T_base_tool[4][4]`。文件头记录模型SHA256、单位、坐标系、关节顺序、随机算法、配额和实际抽样次数。见证q证明几何可达；后续IK评测应另定义初值，不把见证q偷偷当作求解答案。

本次base下位置范围：X约[-0.7893,0.8070]、Y约[-0.6092,0.7628]、Z约[-0.7470,0.9798] m。划分Z时减去了肩部高度，不能将区名误读成base原点的八象限。

## 边界和异常独立成组

- **16个边界输入**：每轴精确上下限12个，加零位、腕theta5=π、肘伸直、肘折叠4个。保存q与FK目标，预期FK返回0；其中部分测试目的可能对应同一几何位形，边界组不要求去重。精确奇异目标不要求IK all成功，沿用已有2003/代表解策略。
- **11个异常输入**：关节长度5/7、NaN/Inf、低于/高于限位、非法齐次底行、非法旋转、目标含NaN，以及外侧不可达和肩域不可达。每条保存operation、payload、expected_code及execution_status=not_run。inverse默认选项在文件头记录，使用相同模型。

严格JSON不写非标准NaN/Infinity常量。只有数值payload位置中的字符串`NaN`、`+Inf`按清单规则解码为IEEE非有限值；这是后续测试执行器的输入约定，不得原样传入C double数组。原生NULL指针仍由既有ABI专项覆盖。本轮只生成/核对异常清单，**没有运行这11例的错误码测试**。

边界/异常不计入主组120的成功率分母。后续统计分别报告，失败或无输出不能当作零误差。

## 校验与复现

```bash
# 日常入口：重算并对照冻结文件，不覆盖
.venv-baseline/bin/python scripts/run_tests.py --suite workspace120 --output results/test-runs/workspace120
# 仅明确要更新样本时生成，之后审查diff
.venv-baseline/bin/python scripts/prepare_workspace120.py --write-fixture
# 修改目标时应拒绝且不得覆盖的负向回归
.venv-baseline/bin/python -m unittest discover -s tests -p test_workspace120.py -v
```

生成代码为 `scripts/prepare_workspace120.py`。默认检查字段、顺序、固定种子、模型哈希、配额及全部重生成内容：非浮点字段精确一致，浮点差≤1e-12；缺文件或漂移返回非零，只有显式`--write-fixture`才允许覆盖。主组关节向量必须唯一，任意两目标矩阵的最大元素差必须>1e-9，避免重复目标。

另外用Robotics Toolbox显式DH模型（**不使用内置UR5的错误d1**）独立复核文件中120+16个姿态，矩阵最大元素差要求≤1e-12。报告 `results/workspace120-verified/workspace120/report.json` 逐项保存参考误差及文件SHA256；主组配额24×5全部满足，136个姿态校验通过，最大差2.220446049250313e-16。误改目标平移1 mm的负向测试正确失败且文件哈希不变。

本轮环境Python3.11.9、NumPy1.26.4、Robotics Toolbox1.1.1；缓存解释器恢复后复用同版本依赖，不改锁文件。已接入统一测试入口、available/acceptance及CI基准作业。报告明确 `ik_execution_status=not_run`、`solver_success_rate=null`，输入准备通过不能代替120组IK求解验收。

“可达”指模型关节限位内的FK几何可达，不包含地面、自碰撞、障碍物、速度或加速度约束；不得据此直接向实机执行样本。

## 主组统一预期规则补充（2026-09-28）

主文件新增 `expected_rules`，适用于全部120条，不改原样本。规则区分“应满足的条件”和“实际执行结果”：

- `geometric_reachable=true`：标称UR5几何与关节限位内有见证解，不承诺碰撞/动力学可行。
- `fk.expected_code=0`：输出有限4×4，和保存目标比较，位置误差≤1e-5 m、姿态角误差≤1e-4 rad。独立参考数据校验另要求RTB矩阵最大元素差≤1e-12。
- `ik.regular_target`：合法输入、受支持模型与普通非奇异目标预期返回0，至少1个有限限位内有效解；每个保留解FK回代≤1e-5 m、1e-4 rad，不要求恢复同一见证q。
- `ik.singular_exception`：解析腕分解hypot(u,v)≤1e-12时，未完成的连续族请求允许返回2003；nearest_seed可以返回已验证的匹配seed代表。仅“靠近奇异”不能自动豁免。2003须单列为奇异例外，不能算code-0成功，也不能悄悄移出原120分母。
- `ik.execution_plan_required`：真正运行前固定初值规则、branch_policy、joint_margin、误差与超时预算。改变模型或收缩限位可能排除原见证，必须另行说明，不能静默改变本数据集预期。

`ik.execution_status=not_run`、`solver_success_rate=null`继续保留。异常、超时、未收敛不能因为目标几何可达就当作成功；默认条件下2001与见证解矛盾，应调查。

本次只插入规则元数据，原文件所有已有字段相同、samples段逐字节一致，独立边界/异常文件不变。审计见`results/workspace120-rules/report.json`；规则缺失/阈值弱化负向检查通过。生成器同步保存并比较规则，CI同时运行规则测试。原数值校验报告是ba5c332时的历史证据，其旧文件哈希不回填；本次新主文件哈希由元数据审计报告记录，未再次执行FK或IK。

## 执行状态更新（2026-09-28 晚）

数据文件内`IK not_run`描述的是准备阶段快照。主组现已按[批量验证](workspace120-batch-validation.md)运行：120/120返回0、890候选完成C与RTB双回代；16边界和11异常另列。冻结的120输入、种子与原模型保持不变；真实运行状态由新报告记录。
