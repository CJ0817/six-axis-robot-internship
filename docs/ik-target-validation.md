# 典型可达、不可达与近限位目标验证

2026-09-25。新增固定输入 `tests/ik/reachability_cases.json`，入口 `scripts/verify_ik_target_groups.py`，统一套件 `--suite ik_targets`。不改模型、C算法或冻结ABI。

## 分组及独立参考

| 分组 | 数量 | 条件 | 预期 |
|---|---:|---|---|
| reachable | 6 | 非对称、正负混合、不同肩肘腕角；seed与见证角不同 | 返回0，至少一个有效候选 |
| near_limits | 24 | J1～J6分别接近上下限；距边界1e-3与1e-6 rad；seed向区间内移至距边界0.02 rad | 返回0，候选逐项限位和回代通过 |
| unreachable | 9 | ±X/±Y/±Z方向10 m六例；肩域内侧距d4为1e-3、1e-6、1e-10 m三例 | 返回2001，不改输出 |
| limit_excluded | 1 | 目标几何可达，但把软件允许运动区间收窄到seed±0.001 rad | 返回1005，不误报2001 |

可达与近限位目标由Robotics Toolbox 1.1.1的显式标准DH参数生成，不用内置UR5的旧d1值；求解器仅收到目标和明示seed。q_witness只用于生成参考目标和说明可达性。软件限位排除测试只临时改变该例模型副本的限位，不改UR5几何或仓库模型。

近限位的含义是生成目标的见证关节角位于限位附近；同一位姿可以有其他更远离边界的解，不强制逆解恢复见证角。每个返回解必须满足其自身限位及误差约束。

## 推导与判断链整理

统一采用米、弧度、J1～J6、标准DH：

`A_i=Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)`，`theta_i=sign_i*q_i+offset_i`。

本模型a2=-0.425、a3=-0.39225 m；d1=0.089159、d4=0.10915、d5=0.09465、d6=0.0823 m。base不是base_link，tool0不是flange。先将目标转换为 `T06=inv(B) T_base_tool inv(G) inv(F)`，再用解析分支公式，不能颠倒右乘逆矩阵的顺序。

1. 移除d6沿末端Z轴平移，得到p05；肩约束 `X sin(theta1)-Y cos(theta1)=d4`。若rho=hypot(X,Y)<d4-1e-12 m，有严格肩域不可达证据。
2. 对每个肩解求 `hR=[sin(theta5)cos(theta6),-sin(theta5)sin(theta6),cos(theta5)]`，求两腕分支；腕退化按已有2003/保持策略处理。
3. 精确消去A1、A6、A5得到 `U=A2 A3 A4`，保留非零d5。肘余弦 `c3=(x²+y²-a2²-a3²)/(2*a2*a3)`，求两肘分支，再求theta2和theta4；a2/a3的负号不能丢。
4. 将DH角还原为逻辑关节，选合法2π表示，逐解C FK验证后去重。没有几何根为2001；几何根被限位全部排除为1005；数值或回代无法得到有效解为2002。

外侧10 m目标还满足保守工作空间排除：末端到DH0距离大于所有DH平移长度绝对值之和1.192509 m。肩域三例使用合法单位旋转，移除d6后rho=d4-eps，各eps均大于1e-12 m域容差；不是把浮点裁剪范围内的目标强行称为不可达。

完整依据分别见[模型与来源](ur5-model-conventions.md)、[正运动学](forward-kinematics.md)、[解析推导和退化条件](ur5-analytic-ik.md)、[候选筛选](ik-candidate-filtering.md)、[状态选解与奇异过渡](ik-stateful-selection.md)、[冻结ABI](abi-v1.md)。

## 验证结果和统计

在恢复的锁定环境Python3.11.9、NumPy1.26.4、RTB1.1.1下，40/40预期结果通过。

- 6个普通目标返回48个解，24个近限位目标返回184个解，共232条候选明细。
- 每个候选分别由C FK和RTB FK回代。记录目标ID、候选索引、q_rad、C与RTB位置/姿态误差、限位距离及通过状态，记录数与solution_count一致。
- C位置最大误差9.17e-16 m，姿态最大误差1.45e-15 rad；两参考均满足1e-5 m及1e-4 rad。实际数值与分组统计见报告。
- 9个不可达正确返回2001，1个软件限位排除正确返回1005；输出缓冲区保持原字节。无有效输出的误差数量为0，均值/RMSE/p95/最大值为null，不能填零误差。

`expected_outcome_rate`表示预期结果符合率；不可达组的100%表示正确拒绝率，不是逆解求解成功率。误差汇总按有效候选记录统计，p95采用h=(n-1)p线性插值。每个C调用预算0.2 s，外层进程预算120 s；本轮无进程超时。这里保存单次调用耗时以便追溯，不作为预热/重复采样后的性能基准。

明细：`results/ik-targets-verified/ik_targets/report.json`，统一入口摘要：`results/ik-targets-verified/summary.json`。旧140目标、52专项与100状态选解案例也在锁定环境复验通过，摘要见`results/ik-closeout/regression-summary.json`；完整既有逐候选报告保留在先前提交，未以新版结果覆盖历史性能证据。

## 复现与限制

```bash
# 按既有setup.sh/setup_baseline.sh准备锁定环境并构建C库
.venv-baseline/bin/python scripts/run_tests.py --suite ik_targets --output results/test-runs/ik-targets
.venv-baseline/bin/python scripts/verify_ik_target_groups.py --library build/librobot_contract.so --output results/ik-targets
```

已接入available、acceptance与CI基准作业。几何可达不代表碰撞可行或实机安全；近限位IK返回有效候选不表示综合选解层一定允许下一步运动。精确奇异族全局选解、C解析雅可比和原FK FFI长尾问题保留为明确待办，见本轮[收尾记录](../reports/week02/ik-target-closeout.md)。
