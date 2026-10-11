# 笛卡尔直线 / 圆弧与静态势场基线（2026-10-11）

本轮在冻结运动学ABI和统一仿真限值上实现 **C11几何插值 + Python姿态/连续IK/时间段认证**。保存直线、圆弧、势场三条成功轨迹及堵塞、限步、超限、预算和缺参数失败案例。仅适用本工程UR5仿真模型、固定基座及静态场景；没有执行用户Windows GUI或实机运动。

## 公共入口与坐标

- 运动限值仍唯一读取`config/planning-limits.v1.json`：默认六轴速度π/2 rad/s、加速度0.5 rad/s²；TCP线/角速度0.125 m/s、0.25 rad/s，线/角加速度0.25 m/s²、0.5 rad/s²。
- `config/cartesian-planning.v1.json`只存算法周期、容差与预算，不重复运动限值；`scenes/static-apf.v1.json`记录固定障碍物和世界→base关系。
- 位姿为base→tool0，TCP是tool0原点，位置m、角度rad。碰撞世界为URDF base_link，`T_world_base`与冻结模型逐项校验；33项测试含C FK与PyBullet tool0对齐。
- 磁盘模型不改写。连续选解的临时profile只把速度字段换成共享配置的**实际使用上限**，避免沿用较高的上游速度；失败配置1004、非法数值1001。

## 几何和姿态规则

直线：`p(s)=p0+s*(p1-p0)`，s∈[0,1]。圆弧由起点、途经点和终点确定唯一平面/圆心，选择**包含途经点**的弧，支持大于π的长弧；平面基决定带符号旋转，不强制取短弧。共线、任意重合、半径异常或起终点相同的整圆请求返回1001，禁止静默退化直线。

C入口`robot_path_position`输出位置及对s的一、二阶导数，声明见[独立规划头文件](../include/robot_planning.h)。不增加/改变运动学ABI v1.0.0。

姿态有两种明确规则：

| rule | 定义 / 失败 |
|---|---|
| fixed | 起终旋转必须相同；否则1007，不忽略目标姿态 |
| slerp_shortest | 单位四元数[w,x,y,z]，点积<0翻转终点符号；沿SO(3)最短角路径SLERP，角度≤π；π处用确定性四元数符号约定，极小角用归一化线性极限 |

圆弧位置与姿态共用s，无“自动切向朝向”假设。旋转始终合法，角速度为空间角速度而非欧拉角导数。0、近0、近π、π、四元数正负等价及大小圆弧均有独立几何检查。

## 固定周期、连续逆解和关节段

`sample_period_s=0.02`，请求总时长向上取整到周期整数倍，不追加短末步。沿五次静止时间标度`s=10u³-15u⁴+6u⁵`采样；从种子FK核对起点，上一成功关节状态作为下一节点选解状态，复用近奇异规避和有界阻尼。每个节点保存目标、q、回代误差、sigma_min、模式及状态推进；失败误差为null，不能填零。

节点之间用归一化局部五次Hermite段连接，内节点速度/加速度中心差分、首末端速度/加速度零；相邻段q/qd/qdd匹配为C2。系数、固定周期位置/速度/加速度和细分验证节点全部保存。不要把固定周期节点之间当成未定义线性插值。

失败`data=null`、`next_state=null`；details保留最后逆解候选状态、原输入q和停止/重规划标记。内部已求解节点只作诊断，不作为可执行前缀；不能在失败后继续原段。

## 整段限值与碰撞认证

1. **关节连续约束**：每段求所有相关多项式实根，检查位置、速度、加速度极值；位置容差1e-9 rad、速度1e-8 rad/s、加速度1e-7 rad/s²。不是只查周期节点。
2. **TCP连续保守界**：标准DH全转动模型中，第i轴到TCP距离上界`R_i=sum(k>=i,hypot(a_k,d_k))+固定末端变换平移范数之和`。以全轨迹逐轴速度/加速度解析峰值V_i、A_i（加数值裕度）给出：
   - 线速度≤ΣR_i V_i；角速度≤ΣV_i。
   - 线加速度≤ΣR_i A_i + 2(ΣR_i V_i)(ΣV_i)；角加速度≤ΣA_i+(ΣV_i)²。
   - 上界须满足共享TCP限值，否则3002。另保存C FK雅可比/Jdot数值观测，作为交叉检查，不用数值差分冒充连续证明。
3. **完整机器人碰撞**：PyBullet 3.2.7 DIRECT加载仓库URDF碰撞网格，检查全体碰撞连杆与球/盒静态障碍物，以及全部非父子碰撞连杆对。仅父子装配对允许接触并明确列出；不排除祖孙对，也不只检查TCP。没有地面或未声明物体；不得推及另一场景。
4. **采样间隙覆盖**：至少每周期2次细分，且按解析速度保证单轴变化≤0.01 rad。由全部URDF关节平移长度之和、最大缩放STL顶点半径/碰撞原点偏置和0.01 m引擎裕度得到每个连杆点的保守运动半径R（本模型1.731423 m）。点速度≤RΣ|qd|，自碰撞距离变化≤2倍点位移。每个相邻验证时刻的半间隔覆盖，用`min(d0,d1)-R*max(ΣV_i)*Δt`给出整段最小距离下界；必须>0.003 m。间隙证据不足返回3001，不宣称安全；直接碰撞/不足净距返回3003。

该证明覆盖**声明的URDF/PyBullet碰撞几何、父子排除规则与静态场景**。笛卡尔路径形状误差仍为细分测量（位置≤1e-3 m、姿态≤1e-3 rad），没有给出连续路径形状误差证明。碰撞、限值、输入场景和源码哈希均写入轨迹JSON。

## 静态势场基线

在关节空间使用吸引势`Uatt=0.5*k*||q-q_goal||²`和最近全机器人几何距离斥力：

`Urep=0.5*eta*(1/d-1/rho)²*||q-q_goal||²`（0<d<rho，否则0），d为最近距离扣除净距。

目标距离衰减让斥力在目标处消失，减少目标邻近障碍时的不可收敛问题；仍不保证不存在局部极小。数值中心差分求梯度，限制每步0.02 rad，回溯要求降势能且细分碰撞通过。成功后保存精确终点，各途经段按静止五次时间标度放慢，再用同一个`certify`进行连续关节/TCP和网格运动界检查。基线有停顿，不宣称全局最优或节拍改进。

| 势场参数 | v1实验值 |
|---|---:|
| 吸引/斥力增益 | 1 / 2e-6 |
| 作用距离 | 0.12 m |
| 数值差分 | 1e-4 rad |
| 最大步长 / 目标容差 | 0.02 / 0.002 rad |
| 迭代 / 回溯预算 | 160 / 12 |

无进展、局部极小或预算耗尽3001；堵塞起终点3003，失败不返回部分轨迹。势场与其时间化均读取统一运动限值。本轮实际成功迭代5次，斥力最大非零；保存预算只允1次的失败轨迹诊断。

## 编译、调用和验收入口

```bash
# 已按锁定依赖安装，Python3.11.9、NumPy1.26.4、PyBullet3.2.7。
export PATH="$PWD/.venv/bin:$PATH"
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build
ctest --test-dir build --output-on-failure
.venv/bin/python scripts/run_tests.py --suite cartesian_planning \
  --output results/test-runs/cartesian-planning
# 单库GCC替代构建（运动学库按原构建说明）
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic -fPIC -shared \
  -Iinclude src/planning/joint.c src/planning/cartesian.c -lm -o build/librobot_planning.so
```

Python入口`CartesianPlanner(kinematics_library,planning_library).plan(start,goal,q_seed,scene,method='line'|'arc',via=None,orientation='fixed'|'slerp_shortest',duration_s=4)`。start/goal有限刚体4x4，q_seed rad(6,)，arc的via为m(3,)，scene为已验证`StaticScene`，duration须≥2周期且≤预算。`StaticScene.close()`释放DIRECT连接。姿态未知/方法未知1008，起点与种子不匹配1001；缺参数1004，逆解沿用2001/2002/2003，时间/节点预算3001，非法时间1006。

算法配置：最大30秒、2001节点；节点IK预算0.5秒；位置/姿态回代阈值1e-5 m/1e-4 rad；检查导数步长1e-4秒。统一测试整个进程预算120秒，不提供可抢占的整规划单次timeout接口。

## 本轮逐例报告和曲线

[33项汇总](../results/cartesian-planning/report.json)全部通过，干净CMake/CTest 7/7。汇总索引包含源码/配置/场景/网格/库及全部证据SHA256；原始JSON可逐节点重算。这里是验收预期通过数，不把预期失败案例称为成功规划。

| 成功轨迹 | 固定节点 / 验证节点 | 证明的最小净距离下界 | 原始证据与曲线 |
|---|---:|---:|---|
| 直线fixed | 152 / 303；请求3.001→3.02 s | 0.008228 m > 0.003 m | [JSON](../results/cartesian-planning/line-success.json) · [CSV](../results/cartesian-planning/line-success.csv) · [SVG](../results/cartesian-planning/line-success.svg) |
| 圆弧SLERP（姿态变化0.04 rad） | 151 / 301；3.00 s | 0.007611 m > 0.003 m | [JSON](../results/cartesian-planning/arc-success.json) · [CSV](../results/cartesian-planning/arc-success.csv) · [SVG](../results/cartesian-planning/arc-success.svg) |
| 静态势场时间化 | 251 / 501；5.00 s | 0.008324 m > 0.003 m | [JSON](../results/cartesian-planning/apf-success.json) · [CSV](../results/cartesian-planning/apf-success.csv) · [SVG](../results/cartesian-planning/apf-success.svg) |

[TCP投影/放大图](../results/cartesian-planning/paths.svg)。APF没有指定笛卡尔几何路径，路径误差字段为null，不填零；另用最终FK与目标q的FK独立验收端点。

| 失败案例 | 预期码 | 证据 |
|---|---:|---|
| 固定姿态目标改变 | 1007 | [JSON](../results/cartesian-planning/fixed-orientation-failure.json) |
| 共线圆弧 | 1001 | [JSON](../results/cartesian-planning/collinear-arc-failure.json) |
| 连续逆解限步无法到达 | 2002 | [JSON](../results/cartesian-planning/ik-step-failure.json) |
| 过短时长连续关节速率超限 | 3002 | [JSON](../results/cartesian-planning/rate-failure.json) |
| 上臂堵塞、TCP远离障碍 | 3003 | [JSON](../results/cartesian-planning/blocked-upperarm-failure.json) |
| 工具起点/势场起点堵塞 | 3003 | [直线](../results/cartesian-planning/blocked-line-failure.json) · [势场](../results/cartesian-planning/blocked-apf-failure.json) |
| 势场迭代预算耗尽 | 3001 | [JSON](../results/cartesian-planning/apf-budget-failure.json) |
| 缺公共限值 | 1004 | [直线](../results/cartesian-planning/missing-limits-failure.json) · [势场](../results/cartesian-planning/missing-apf-limits-failure.json) |

未完成项：非零边界/不停顿路径策略、动态障碍、RRT/PRM对比及成功率/节拍改善正式验收、正式重复性能统计、控制/跟踪及本机新GUI验证。没有由一个静态基线推断全场景或实机可执行性。
