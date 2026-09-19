# 推导与接口整理：从 FK 进入 IK

本页是阶段交接与逆解输入准备，不是解析 IK 已实现的声明。完整连杆推导、矩阵存储和现有ABI见[正运动学说明](forward-kinematics.md)；模型和坐标来源见[UR5模型约定](ur5-model-conventions.md)。

## 已实现的推导与接口

- 标准DH：A_i=RotZ(theta_i)·TransZ(d_i)·TransX(a_i)·RotX(alpha_i)，theta_i=sign_i*q_i+offset_i。
- 正运动学：T_base_tool=B·A1…A6·F·G，其中B=T_base_dh0，F=T_dh6_flange，G=T_flange_tool。
- C入口：robot_dh_transform、robot_forward及4×4矩阵基础运算；按行存储、齐次列向量。输入6个弧度关节角，输出base→tool0的4×4矩阵。
- Python入口：Forward(library)(profile,q)，成功data为矩阵，失败data=null。保留原始错误码；纯FK模型子集不能启动控制。

## 逆解前的目标变换

在逆解几何方程前移除固定变换：

`T_dh0_dh6 = inverse(B) · T_base_tool · inverse(G) · inverse(F)`。

右端逆矩阵顺序必须反转；不能把inverse(F)放在inverse(G)前面。世界系目标先换到base：`T_base_tool=inverse(T_world_base)·T_world_tool`。求得DH角后恢复逻辑关节：`q_i=(theta_i-offset_i)/sign_i`，再按有限限位枚举允许的2π等价解。不能先把所有角强行折返到[-π,π]而丢掉合法分支。

每个候选必须通过有限性、限位及C FK回代的双误差检查；位置阈值1e-5 m、姿态角1e-4 rad。目标来自FK仅证明存在见证解，不保证任意初值收敛；见证q只供测试验证，禁止传给求解器当作隐藏答案。

## 待实现的逆解接口边界

| 项目 | 已约定内容 | 状态 |
| --- | --- | --- |
| 输入 | 已核对model；T_base_tool为有限合法4×4；q_seed_rad为限位内6维；options为方法、误差阈值、限位余量及预算 | 字段语义沿用工程规范7.2/7.5 |
| 解析方法 | analytic_ur5；nearest_seed或all分支策略。q_seed用于选支，不能偷偷替换为目标见证q | 待实现 |
| 数值辅助 | damped_least_squares，显式迭代/阻尼/步长/时间预算 | 待实现，不能静默替代解析法 |
| 成功输出 | 选定q、有限且回代通过的候选数组、分支标识、选中索引；不要求恢复生成目标的同一组关节角 | 版本化输出结构按7.5冻结 |
| 失败 | data=null；非法输入1001，模型缺失1004，证明不可达2001，未收敛/预算耗尽2002；奇异位形按是否能得到有效代表解处理 | 不把未收敛等同不可达 |
| C ABI | 候选输出容量、容量不足行为、结构体布局、版本及诊断字段 | 已冻结于abi-v1.md；当前robot_inverse_v1为返回1008的占位符号 |
| 耗时/终止 | 单样本预算0.2 s；独立工作进程超时回收；计时层级及统计见统一规则 | 测试执行器待实现；本轮不计求解成功率 |

C ABI布局已冻结于[ABI v1](abi-v1.md)；解析几何分支推导和奇异族代表解是下一项工作；本轮不添加伪成功占位函数。

## 冻结的测试目标

`tests/ik/targets.json` 固定seed=20260917，使用显式项目参数RTB FK生成目标，并经现有C FK逐个核对矩阵最大元素差≤1e-9。不会使用d1有差异的RTB内置UR5生成真值，也不通过运行IK筛选容易成功的案例。

| 集合 | 数量 | 生成/初值规则 | 后续统计 |
| --- | --- | --- | --- |
| normal | 100 | 在模型安全余量内均匀采q，seed逐轴扰动±0.05 rad；见证和seed均检查限位 | 既定目标100%成功；实际求解后统计 |
| wide_initial | 20 | 复用normal前20个目标，独立宽初值；与见证q最大逐轴最短角距离≥0.5 rad，避免仅差2π却误称宽初值 | 单列成功率，不与normal混合平均 |
| near_singular | 20 | 随机其余轴，q5为正负1e-7～1e-5 rad；seed扰动±0.05；用显式RTB雅可比核实最小奇异值≤1e-5，线速度行除以L=1 m后作SVD | 单列，属腕奇异附近，不代表覆盖所有肩/肘奇异族 |
| anchors | 3 | 零位、演示初态、非对称已知角；见证q作为显式seed | 冒烟锚点，排除在上述140样本分母之外 |

固定目标、q_witness、q_seed、分组和最小奇异值均保存。成功数、失败数和超时数必须等到求解器运行后填写；当前ik_execution_status=not_run，solver_success_rate=null。误差有效数、RMSE、p95等继续遵循[统计规则](statistics-rules.md)。

当前只准备可达目标。不可达、非法输入、容量不足与精确奇异退化案例须在IK接口冻结后另建，不把本轮可达集合称为完整IK测试集。

## 生成、校验与修改管理

```bash
# 显式生成（仅维护者有意更新固定集合时）
.venv-baseline/bin/python scripts/prepare_ik_targets.py --write-fixture
# 日常只校验，不覆盖固定文件
.venv-baseline/bin/python scripts/prepare_ik_targets.py
.venv-baseline/bin/python scripts/run_tests.py --suite ik_prepare --output results/test-runs/ik-prepare
```

需先编译build/librobot_contract.so。固定集合与模型、脚本和C库哈希写入报告；跨运行环境的float64微差允许1e-12，字段、样本顺序、数量、分组和模型哈希必须一致。改模型或输入规则必须显式重新生成、审查差异并提交，不能由CI自动覆盖。

集合只验证几何可达性，不验证碰撞或运动安全约束；目标不得直接作为设备运动指令。
