# planning

负责关节/笛卡尔路径及时间轨迹，依赖kinematics/common，不下发设备命令。

已实现C11静止到静止、六轴同步的三次/五次/梯形（含三角形退化）关节轨迹。独立`librobot_planning.so`，Python入口`JointPlanner.plan_joint`；调用方式、系数约定、导数及检查见[关节规划说明](../../docs/joint-planning.md)。方法/接口前均说明用途、参数和返回值。

规划器与测试共同调用`load_planning_limits()`读取唯一`config/planning-limits.v1.json`。缺项必须拒绝规划；请求缩放默认1，在加载时一次应用，位置限位读取绑定模型，不再重复缩放。

静止关节入口仍标记TCP/碰撞未检查。新`CartesianPlanner.plan`支持直线/圆弧、姿态、固定周期连续IK及C2段；使用DH保守TCP导数界和全URDF网格碰撞运动界。`static_apf`提供静态势场及失败诊断，详见[笛卡尔与势场](../../docs/cartesian-and-static-apf.md)。非零边界、动态避障、比较指标/性能和控制仍待后续。

[参数表](../../docs/planning-limits.md) · [工程总方案](../../docs/engineering-contract.md) · [97项证据](../../results/joint-planning/report.json)。
