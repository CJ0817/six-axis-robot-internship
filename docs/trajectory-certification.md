# 统一轨迹认证与执行入口

`JointPlanner.plan_joint(start, goal, options, scene=scene, kinematics_library=...)`
现在是完整认证入口。`plan_candidate` 仅生成候选轨迹，其 `data` 显式包含
`trajectory_status="candidate"`、`executable=false`，不能送入执行入口。

完整入口直接认证 C 生成的原多项式和分段时间；不会用另一条拟合曲线替代原轨迹。
三次、五次、梯形曲线均检查连续位置、速度、加速度极值，TCP 线／角速度及
线／角加速度保守连续上界，同时记录雅可比导数计算的采样值。
碰撞使用固定 URDF 整臂几何、自碰撞非相邻连杆对和相邻采样之间的运动包络。
模型、配置、场景和动态库均绑定到认证结果。认证范围为现有静态场景仿真。

```python
from planning.execution import execute_trajectory

result = joint.plan_joint(q0, q1, options, scene=scene,
                          kinematics_library=kinematics_library)
dispatch = execute_trajectory(result, scene, simulation_consumer)
```

`simulation_consumer(data)` 只在门禁通过后调用一次，接收独立数据副本。
执行门禁拒绝候选、失败 Result、伪造认证标志、认证后修改的数据及变化后的配置、
模型、库或场景。认证凭证仅在当前 Python 进程有效；从磁盘加载的历史 JSON 应
重新认证。此入口是仿真调度入口，不授予实体机器人执行资格。

| 情况 | 错误码及行为 |
|---|---|
| 完整入口关节位置、速度、加速度或 TCP 约束超限 | `3002`，`data=null`、`next_state=null` |
| `duration_policy="stretch"` 且导数超限 | 逐次延长到上一候选时长的 1.5 倍，每次重新生成并完整认证；最多 16 次且受最大时长约束 |
| 整臂或自碰撞 | `3003`，禁止用延时绕过碰撞 |
| 连续包络证明不足、轨迹不连续、执行门禁拒绝 | `3001` |
| 缺少限值配置、认证设置、模型、FK 库或场景 | `1004` |

失败时保留 `details.failure_log` 与 `details.attempts`，后者含候选阶段、认证阶段、
时长和已完成的检查节点。返回值不包含可执行部分轨迹。
候选生成器和冻结 C 接口沿用原始输入关节越限码 `1005`；完整入口统一映射为 `3002`。

回归命令：

```bash
python scripts/verify_trajectory_certification.py
```

脚本保存每个成功及失败 Result 和汇总报告，并检查执行回调在拒绝时未被调用。
CI 的 `Trajectory certification` 工作流同时重跑原关节候选、笛卡尔、APF 平滑及缩时回归。
原有工作流继续保留，不增加避障场景开发。
