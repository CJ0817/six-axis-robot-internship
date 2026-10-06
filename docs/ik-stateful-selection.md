# 相对上一关节状态的综合选解

2026-09-25 新增；2026-10-06 补充局部阻尼退化。C ABI v1 的nearest_seed仍表示未折返欧氏距离最近；本功能在Python适配层调用C的all候选，实施综合评分和连续性约束，不改变C签名、结果布局或错误码。不实现控制器，也不向设备发送指令。

## 输入输出

```python
from adapters.ik_selection import StatefulIKSelector, SelectionConfig
selector = StatefulIKSelector("build/librobot_contract.so", SelectionConfig())
result = selector.select(profile, T_base_tool, state)
# 仅code==0时，调用方接受result["next_state"]作为下一次的state
```

| 输入 | 类型/形状 | 约束 |
|---|---|---|
| profile | 项目模型dict | 标准DH、UR5 CB、m/rad、base→tool0；含有限正值qd_max_rad_s[6] |
| T_base_tool | 有限浮点4×4数组 | 合法齐次刚体变换，通过模型pose_validation_tol |
| state.q_rad | float[6] | 上一已接受关节状态，原始未折返角；位于收缩限位内 |
| state.step_index | int | 必填、非负，bool不接受 |
| config | SelectionConfig | 所有数值有限；约束见下表 |

成功返回code=0、data.q_rad、mode、selected_candidate_index、两项回代误差、sigma_min；next_state含新q_rad及step_index+1。details.candidates逐项保留原C候选索引、q_rad、回代误差、行程、限位距离、奇异指标、成本、准入状态和拒绝原因。失败data和next_state均为null，传入state不修改；调用方不能使用失败返回的诊断候选推进状态。

状态归调用方持有。本模块不假定真实机器人已经到达所选角度；接入设备前应由反馈状态替代纯仿真的上一命令状态。

## 配置、成本和硬限制

| 字段 | 默认值 | 约束/用途 |
|---|---:|---|
| travel_weight / limit_weight / singular_weight | 1 / 0.05 / 0.05 | ≥0，总和>0；三项成本权重 |
| max_step_rad | 0.15 | >0，所有轴的单步角变化上限 |
| dt_s | 0.02 | >0，调用方指定的目标采样周期，非实际测得运行周期 |
| joint_margin_rad | 0 | ≥0，收缩所有关节限位；收缩后区间非空 |
| limit_scale_rad | 0.2 | >0，限位成本尺度 |
| sigma_soft / sigma_hard | 0.02 / 1e-5 | 0<hard<soft，归一化雅可比阈值 |
| near_step_scale | 0.25 | (0,1]，近奇异时限步缩放 |
| characteristic_length_m | 1 | >0，将雅可比线速度行除以此尺度 |
| difference_step_rad | 1e-6 | (0,1e-3]，C FK数值差分步长 |
| position_tol_m / orientation_tol_rad | 1e-5 / 1e-4 | >0，姿态阈值≤π |
| timeout_s | 0.2 | >0，整个选择过程协作式墙钟预算；C调用使用剩余预算 |
| dls_enabled | true | bool；仅解析返回2003或全部候选因奇异保护被拒时尝试局部阻尼退化 |
| dls_max_iterations / dls_line_search_steps | 24 / 8 | 正整数；阻尼迭代数及每次线搜索试探上限 |
| dls_damping_min / dls_damping_max | 0.001 / 0.1 | 0<min≤max，归一化雅可比的自适应阻尼区间 |

这些阈值是本项目仿真初始参数，不是厂家实机安全等级或硬实时保证。基准环境已有NumPy依赖，不增加新的C库依赖。

设上一状态q_prev，候选q，原始差值delta=q-q_prev（不按2π折返）；每轴基本限步

`cap_i = min(max_step_rad, qd_max_i * dt_s)`。

收缩后的限位为L,H，最小余量 `d=min_i(min(q_i-L_i,H_i-q_i))`，归一化雅可比最小奇异值为sigma。评分：

```
travel_cost   = mean((delta_i / cap_i)^2)
limit_cost    = (limit_scale_rad / max(d, 1e-12))^2
singular_cost = (sigma_soft / max(sigma, 1e-12))^2
score = travel_weight*travel_cost + limit_weight*limit_cost + singular_weight*singular_cost
```

先排除违反FK、限位、奇异保护或连续性约束的候选，再选score最小者，同分按q字典序。不能用较低成本抵消硬约束。限位成本使用最危险一轴的余量；行程是实际未折返关节运动，不以跨±π时的数值折返制造跳变。原C候选已按seed=上一状态选好每个几何分支的合法2π表示。

## 奇异估计与基础连续性

每个状态逐轴调用C FK做中心差分；接近限位无法双侧扰动时采用合法区间内单侧差分。平移差分除以characteristic_length_m；姿态以 `dR * R^T` 的反对称部分计算空间角速度，组合成6×6矩阵并经NumPy SVD得到sigma_min。这是上层数值估计，不代表C解析雅可比（KIN-04）已实现。

- 当前或候选sigma低于soft：每轴限步缩小至near_step_scale×cap；仍必须满足原FK误差阈值。
- 候选sigma≤hard：拒绝新运动候选。若当前已在hard区域且当前FK已满足目标，返回singular_hold，q不变。
- 对端点筛选通过的候选，检查关节线性插值的1/4、1/2、3/4处sigma；中间点触及hard则拒绝，进入soft则继续收紧限步。
- UR5腕奇异满足theta5=kπ。根据符号与零偏，检查当前至候选theta5区间内是否有任何严格内部kπ交点，有则拒绝。因此两个非奇异端点也不能跳过腕奇异点。
- 若从精确奇异点小步离开，当前端点不作为“严格内部交点”，允许在候选和中间点合格时退出。它不等于求解精确奇异目标的连续解族。

一般肩/肘奇异仍只做上述有限采样，不能宣称覆盖连续路径每一点。限步提供基础离散连续性和平均速度限制，不等于已验证完整轨迹的瞬时速度、加速度、碰撞或动力学约束；没有声称C1/C2轨迹连续性。下一阶段规划器应补齐连续轨迹约束。

解析返回2003或全部候选因奇异保护被拒时，默认尝试**Python层局部阻尼最小二乘**。数值雅可比采用上述缩放；对空间位置与旋转对数误差求 `Δq=Jᵀ(JJᵀ+λ²I)⁻¹e`，`λ` 随 sigma 向 hard 区域靠近而从0.001增至0.1，回溯线搜索只接受残差下降的试探。每次试探均检查关节限位、上一状态±本周期步长、腕部严格内部 `theta5=kπ` 穿越和路径采样 sigma>hard。最多24次迭代，每次最多8次回溯；位置与姿态误差必须同时通过原阈值，最终及采样点必须在 hard 区域之外，才返回 `mode=dls_near_singular`、实际回代误差、`selected_candidate_index=null` 和迭代/阻尼诊断。它可能在用户设定的位姿容差内选到附近的非奇异代表，不承诺精确等于奇异目标位姿。

退化无可接受解时仍返回2003，预算耗尽返回2002，`data` 和 `next_state` 为null，上一状态不推进；C证明不可达2001、输入/限位/模型错误原样保留，普通连续性跳变仍为2002。只有 C 原生解析候选失败且符合上述条件时才启用退化，**C ABI 中的 method=2 仍未实现（1008）**。此局部方法不能保证穿越精确奇异、规避碰撞、解析无限连续族全局最优或实机可用；规划器收到非零码应停止推进并另行重规划。循环及输出前检查预算，无法打断正在执行的SVD/FK调用，故为协作式预算。

## 入口、证据与范围

```bash
# 先按现有构建入口生成build/librobot_contract.so；使用锁定基准环境
.venv-baseline/bin/python examples/ik_selection_demo.py
# 也可输入JSON形式4x4目标及上一状态
.venv-baseline/bin/python examples/ik_selection_demo.py --target target.json --previous .3 -1 .9 -.7 .8 .2
.venv-baseline/bin/python scripts/run_tests.py --suite ik_selection --output results/test-runs/ik-selection
```

原始报告 `results/ik-selection-verified/ik_selection/report.json` 保存初版100个案例；[新增退化回归逐例摘要](../results/ik-selection-dls/summary.json)保存103个案例各自的预期/实际码与通过状态、6项综合检查及3个新增案例的完整输入输出；运行上述测试命令会另外生成完整的 `report.json`，其中包含全部候选评分、回代误差和状态。新增例覆盖精确奇异目标附近安全近似、关闭退化时2003及较远目标限步拒绝。示例输出保存于 `results/ik-selection-demo.json`。统一入口 `ik_selection` 已纳入 CI。

初版云端实测环境Python3.12.14 / NumPy2.3.5，C库与上一提交一致；锁定的Python3.11.9 / NumPy1.26.4环境由既有CI基准步骤运行，本报告不冒称在该版本上实测。100/100案例、5/5补充检查通过：40步普通序列、30步跨π序列、15步近奇异序列，以及保护区拒绝、精确奇异保持、小步退出、两个有效端点间腕奇异穿越拒绝、限位附近、跳变、错误输入、预算及权重比较。权重比较刻意放宽步幅，只验证评分可改变分支，不作为常规运动配置。

同一目标的纯行程、纯限位、纯奇异成本分别选中索引2、7、4，证明三项指标实际参与选解；综合默认权重选中2。步长h和h/2的sigma估计差在普通、近腕奇异、关节端点三例均≤1e-6。测试未包含实机运行，FK FFI单次耗时超限和C奇异族通用选解待办仍保留；完整正式测试状态见[阶段报告](../reports/week02/kinematics-stage-test-report.md)。

## 连续多节点与失败后重规划回归（2026-10-06）

[逐节点证据](../results/ik-selection-sequences/sequence-report.json)覆盖55个连续节点：接近奇异区、解析2003后局部阻尼、重复目标保持、离开、精确奇异原位保持及小步离开；另有1个2002限步失败。每个节点记录目标、上一/保留状态、回代位置和姿态误差、sigma_min、返回码、state_advanced及q_changed。失败没有有效解，解误差和sigma为null，同时单列保留状态的实际误差和sigma；原位保持可推进step_index但不改变q。

失败节点之后，离线测试编排明确停止当前段、丢弃原段尚未执行的1个节点，从保留状态将原目标重新细分为34个最大关节增量≤0.005 rad的新目标，逐个重试后到达原目标。全55节点符合预期（54成功、1预期失败），原103案例及全部10项附加检查通过。此处重规划为可复现的关节途经点细分测试编排；完整笛卡尔规划、加速度/碰撞验证和设备停止仍待后续实现，不能从这组测试推断任意奇异路径均可通行。统一ik_selection入口和CI会运行这组序列。
