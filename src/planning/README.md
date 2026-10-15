# planning

负责关节/笛卡尔路径及时间轨迹，依赖kinematics/common，不下发设备命令。

已实现C11静止到静止、六轴同步的三次/五次/梯形（含三角形退化）关节轨迹。独立`librobot_planning.so`，Python入口`JointPlanner.plan_joint`；调用方式、系数约定、导数及检查见[关节规划说明](../../docs/joint-planning.md)。方法/接口前均说明用途、参数和返回值。

规划器与测试共同调用`load_planning_limits()`读取唯一`config/planning-limits.v1.json`。缺项必须拒绝规划；请求缩放默认1，在加载时一次应用，位置限位读取绑定模型，不再重复缩放。

`JointPlanner.plan_joint` 与 `CartesianPlanner.plan` 共用完整多项式认证，检查关节约束、TCP 导数界和全 URDF 网格整臂／自碰撞运动界。笛卡尔入口支持直线／圆弧、姿态、固定周期连续 IK 及 C2 段。`static_apf` 提供静态势场及失败诊断，详见[笛卡尔与势场](../../docs/cartesian-and-static-apf.md)。

[参数表](../../docs/planning-limits.md) · [工程总方案](../../docs/engineering-contract.md) · [97项证据](../../results/joint-planning/report.json)。


## 完整认证入口

`JointPlanner.plan_joint` 现要求场景和运动学库，认证关节约束、TCP 导数及整臂／自碰撞后才返回可执行轨迹。低层公式试验使用 `plan_candidate`，其结果明确不可执行。仿真执行统一经过 `planning.execution.execute_trajectory`。详见[统一认证说明](../../docs/trajectory-certification.md)。历史 97 项仅为候选生成器回归，不代表完整认证。
