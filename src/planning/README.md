# planning

负责关节/笛卡尔路径及时间轨迹，依赖kinematics/common，不下发设备命令。

已实现C11静止到静止、六轴同步的三次/五次/梯形（含三角形退化）关节轨迹。独立`librobot_planning.so`，Python入口`JointPlanner.plan_joint`；调用方式、系数约定、导数及检查见[关节规划说明](../../docs/joint-planning.md)。方法/接口前均说明用途、参数和返回值。

规划器与测试共同调用`load_planning_limits()`读取唯一`config/planning-limits.v1.json`。缺项必须拒绝规划；请求缩放默认1，在加载时一次应用，位置限位读取绑定模型，不再重复缩放。

只完成关节限位/速度/加速度和端点检查；TCP限值虽必需加载但尚未做曲线导数检查，返回`cartesian_limits_checked=false`。碰撞为unchecked。非零边界导数、笛卡尔轨迹/避障和控制仍待后续，不可宣称完成全规划验收。

[参数表](../../docs/planning-limits.md) · [工程总方案](../../docs/engineering-contract.md) · [97项证据](../../results/joint-planning/report.json)。
