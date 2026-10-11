# 子目标势场、轨迹平滑与缩时同场景对照

本轮加入确定性中间子目标、几何平滑及固定周期缩时，并复用已冻结的整轨迹验收。普通案例轨迹运行时长由5.00s降至2.32s，减少53.6%；完整规划计算时间有所增加，二者分开统计。冻结运动学ABI和模型参数没有变更。本轮全部是PyBullet DIRECT仿真，不新增Windows GUI验收要求。

## 统一参数与接口

新增算法参数：[guided-apf.v1.json](../config/guided-apf.v1.json)，版本1.0.0、simulation_only、项目实验来源。运动上限仍使用[共享限值](../config/planning-limits.v1.json)，关节速度/加速度及TCP线/角速度和加速度都取配置缩放后的实际上限；TCP为tool0原点，按base系向量欧氏模长验收。新算法不修改运动限值、不解除限步或自碰撞检查。

| 字段 | 类型/形状/单位 | 本轮值与约束 |
|---|---|---|
| guide_fractions | float[N]、无量纲、必填 | [0.5,0.33,0.67]，各项在(0,1) |
| guide_offsets_rad | float[N,6]、rad、必填 | 零偏移及J1/J2/J3/J5的±0.25rad；有限值，生成节点另验关节范围 |
| maximum_candidates | int、必填 | 27，正整数；按fraction再offset顺序 |
| smoothing_passes | int、必填 | 2，正整数 |
| smoothing_weight | float、必填 | 0.3，在(0,1) |
| edge_maximum_subdivisions | int、必填 | 4096，正整数；证明不足拒绝3001 |
| compression_factor | float、必填 | 0.8，在(0,1)；所有实际时长向上取0.02s整数倍 |

[方法/入参/返回值注释](../src/planning/guided_apf.py)：`guided_apf(planner,start,goal,scene,config_path)`找几何途经点；`smooth_waypoints(...)`处理N×6 rad几何路径；`parameterize_smoothed(...)`生成认证五次段；`retime_trajectory(planner,trajectory,scene,factor)`重验缩时结果；`improved_plan(...)`是可执行结果入口。所有输出遵循统一Result，失败data/next_state=null；仅返回waypoints不代表可执行轨迹。端点不修改，角度不自动折返。

## 引导与平滑方法

先调用原势场，返回3001时从原起点尝试子目标 `q_sub=(1-f)q_start+f*q_goal+offset`。子目标限位和整臂/自碰撞合格后，分别用同一APF参数求起点→子目标和子目标→终点；两段成功才拼接。非法参数、端点堵塞等错误不当作可恢复局部极小。所有候选和每段APF迭代都归档，失败段不能推进执行状态。直接APF只调用一次，改进方案允许最多27个子目标、各两次APF；每次参数一致，但总计算预算更多，不能声称是等总预算算法比较或全局完备搜索。

平滑先验输入各边，再按固定顺序做两轮拉普拉斯移动 `q_i'=(1-w)q_i+w(q_(i-1)+q_(i+1))/2`，两条替换边都通过连续净距才接受。随后从当前节点尝试最远可认证捷径。所有接受、拒绝及证明不足事件保留。边检查不只测端点：用URDF全臂运动半径R和细分关节增量界 `R*sum(abs(delta_q))/N` 减去离散最小距离，证明任意中间配置净距>0.003m；预算不足不自动认为安全。

时间平滑采用静止端点五次段，归一化u下 `s=10u³−15u⁴+6u⁵`。先根据关节和TCP保守导数下界选时长，给0.8缩时预留10%裕度。五次时间标度的关节速度/加速度界分别为1.875|Δq|/T和(10/√3)|Δq|/T²；TCP使用冻结DH剩余半径及Jdot保守界，见[公共认证说明](cartesian-and-static-apf.md)。仍保留途经点停顿，未实现不停顿最优时间规划；平滑和捷径在简单案例可去掉全部内部节点。

## 缩时后的验收

不能只改时间戳：按新总时长固定0.02s采样原轨迹，重拟合归一化五次Hermite段，重验全部位置/速度/加速度实根极值、DH保守TCP速度/加速度界、数值TCP导数、整臂/非父子自碰撞及连续净距界。关节拼接C2、端点速度/加速度零及固定周期另外核对。只排除原报告列出的直接父子装配对，其余自碰撞对继续检查。固定周期取整后的actual_factor写入原始结果，不假定正好等于请求值。

错误码：缺配置1004、非法参数1001、关节越界1005、超速度/加速度3002、碰撞3003、候选/证明/时长/节点预算不足3001。成功前任何阶段失败都无可执行输出，原输入和原已认证轨迹不变。

## 同场景逐次结果

[正式汇总与逐次耗时](../results/guided-apf-run/guided_apf/report.json)，[统一入口/进程实际等待时间](../results/guided-apf-run/summary.json)，[全部运行日志](../results/guided-apf-run/guided_apf.log)，[路径对照SVG](../results/guided-apf-run/guided_apf/comparison.svg)。每案例成对使用相同起终、障碍物、模型、共享限值和APF单次参数；normal/short-stage/self-blocked均使用原场景static-apf-v1。upperarm-blocked只增一个上臂封堵球，两方案使用完全相同的变体，TCP距球>0.1m。短阶段案例把双方单次APF预算都设为4，未改运动限值。

| 案例 | 方案 | 成功/应执行 | 返回码 | 轨迹时长(s) | 计算均值(s) | 计算P95(s) | 计算最大(s) |
|---|---|---|---|---|---|---|---|
| nominal | baseline | 3/3 | 0 | 5.00 | 0.829013 | 0.842488 | 0.842935 |
| nominal | improved | 3/3 | 0 | 2.32 | 0.968285 | 1.003423 | 1.008481 |
| short-stage-budget | baseline | 0/3 | 3001 | — | 0.008008 | 0.008591 | 0.008666 |
| short-stage-budget | improved | 3/3 | 0 | 2.32 | 1.110482 | 1.231358 | 1.252485 |
| self-blocked | baseline | 0/3 | 3003 | — | 0.000727 | 0.000894 | 0.000921 |
| self-blocked | improved | 0/3 | 3003 | — | 0.001286 | 0.001480 | 0.001499 |
| upperarm-blocked | baseline | 0/3 | 3003 | — | 0.000497 | 0.000513 | 0.000513 |
| upperarm-blocked | improved | 0/3 | 3003 | — | 0.000974 | 0.001134 | 0.001158 |

普通案例基线关节路径长0.08124068rad、TCP路径长0.03924454m；改进为0.08124038rad和0.03924275m。路径长为固定周期关节欧氏折线长、FK TCP欧氏折线长，不伪装成连续精确弧长。几何路径差异很小，主要节拍改善来自去掉内部停顿。改进平滑定时原为2.90s，0.8缩时后2.32s；连续认证净距下界0.00739733m>0.003m。关节/TCP导数界、每个验证点、路径及q/qd/qdd CSV/SVG在对应原始JSON和曲线中，汇总登记每项SHA256。

每方法预热1次，不计入统计；正式3次按基线/改进顺序交替。成功率分母是该案例应执行的3次，成功须返回0且完整限值/连续碰撞认证通过；预算耗尽/异常/堵塞为失败，不填零路径或零误差。每个正式失败保留完整原因和阶段结果；成功重复路径SHA256一致，并归档首次完整轨迹。预期拒绝符合率另外统计，不把故意堵塞称为成功规划。3次统计仅为本轮观测，不是正式延迟验收或一般成功率估计。

计时使用perf_counter：调用完整规划流水线前开始、返回后结束，含全轨迹认证，排除场景构造、FK路径长汇总、写文件与画图。C函数和Python→C时间未单独测量，保持null。平均/中位数/P95/最大值包含失败调用，分位采用h=(n−1)p线性插值。script_work_wall_s覆盖main入口至构造汇总，不含import/最后写报告；真正的启动至退出进程墙钟、120s预算、超时数量和实际等待记录在统一入口summary。summary中的commit/dirty是本地Git元数据；最终源文件、配置和C库由SHA256绑定，可从本轮仓库提交复现。

## 失败专项及范围

- [速度超限缩时](../results/guided-apf-run/guided_apf/excessive-speed-compression.json)：实际0.06s，J1峰速2.19213rad/s>1.57080rad/s，返回3002，无轨迹。
- [仅加速度超限缩时](../results/guided-apf-run/guided_apf/excessive-acceleration-compression.json)：实际0.08s，J1峰速0.02305rad/s合格，峰加速度1.31851rad/s²>0.5rad/s²，返回3002，无轨迹。
- [两端安全、中间自碰撞](../results/guided-apf-run/guided_apf/safe-endpoints-colliding-interior.json)：平滑输入边内部碰撞，返回3003，保留碰撞分数位置、最近对象和拒绝证明。
- 子目标/上臂/非父子自碰撞堵塞、非法缩时比例、缺配置和证明预算均有独立回归。失败误差/可执行路径为null，不当成零误差。
- [默认160迭代长路线几何恢复](../results/guided-apf-run/guided_apf/long-route-geometric-guided.json)：直接APF预算耗尽3001，子目标两段找到几何路线；但[完整请求](../results/guided-apf-run/guided_apf/long-route-duration-budget-failure.json)仍因30s/2001节点总预算不足3001，无可执行轨迹。这是保留的失败案例，未放宽约束掩盖。

本轮50/50预期检查通过：24次正式请求，8项重复性检查，加上引导、碰撞/缩时/配置/连续性专项。RRT/PRM、动态障碍物、全局完备性、实机余量、通用改善率和正式延迟验收未纳入本轮；PLAN-05已接入功能对照，通用改善门槛仍待冻结。

## 重现命令

```bash
bash scripts/setup.sh
.venv/bin/python scripts/run_tests.py --suite guided_apf --output results/guided-apf-run
.venv/bin/ctest --test-dir build --output-on-failure
```

单独入口支持`--library`、`--planning-library`和`--output`；详细命令保留在summary.json。已纳入available/acceptance、CTest和环境CI，单个测试进程预算120s，不宣称每个候选有硬实时抢占超时。

集成验证：统一入口通过50/50检查，CTest 8/8通过，测试入口失败传播专项2/2通过。原始日志见results/guided-apf-run/ctest.log与guided_apf.log。
