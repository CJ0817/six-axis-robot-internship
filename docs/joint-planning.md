# 关节三次、五次与梯形轨迹 v0.1（2026-10-10）

本轮实现 **C11 静止到静止、六轴同步关节轨迹**，Python负责统一参数闸门、序列化与验收。独立 `librobot_planning.so` 和 [robot_planning.h](../include/robot_planning.h) 不更改冻结的运动学 ABI v1.0.0。

## 仓库及公共参数入口盘点

| 内容 | 当前入口 / 状态 |
|---|---|
| 几何、轴序、位置限位 | `models/ur5/kinematics.json`；UR5 CB、标准DH、base→tool0、J1..J6、m/rad |
| 唯一仿真速度/加速度配置 | `config/planning-limits.v1.json` v1.0.0，模型字节SHA绑定 |
| 公共参数加载 | `planning.limits.load_planning_limits`；规划器每次调用，测试也直接调用；缺项/null1004，不使用无限制 |
| 基础实际使用上限 | 六轴速度π/2 rad/s、加速度0.5 rad/s²；请求缩放默认1，只允许进一步降低 |
| TCP参数 | tool0原点、base系欧氏模长；参数必须完整，但本轮关节规划**未验证TCP导数** |
| 运动学/选解 | C FK/普通解析IK；Python连续选解与局部阻尼，沿用原基准 |
| 新关节曲线 | `src/planning/joint.c`、`planning.joint.JointPlanner.plan_joint` |
| 统一测试 | `scripts/run_tests.py --suite joint_planning`；CMake增加`joint_trajectories` |
| 待实现 | 非零边界速度/加速度、跨段不停顿、笛卡尔直线/圆弧及约束、碰撞、控制器；不能由本轮抵扣 |

## 同步与解析推导

设`d_i=q_goal_i-q_start_i`，`q_i(t)=q_start_i+d_i*s(t)`。所有轴共享一个s和时长T，零位移轴恒定；不进行角度折返。活动轴定义归一化限值：

`v=min(vmax_i/abs(d_i))`，`a=min(amax_i/abs(d_i))`。

三次，`u=t/T`：

- `s=3u²-2u³`，`sd=6u(1-u)/T`，`sdd=(6-12u)/T²`。
- 速度峰值`1.5/T`（u=0.5），加速度绝对峰值`6/T²`（两端）。
- `T>=max(1.5/v,sqrt(6/a))`。静止速度端点，端加速度一般不为零；与静止段拼接仅C1。

五次：

- `s=10u³-15u⁴+6u⁵`，`sd=30u²(1-u)²/T`，`sdd=60u(1-u)(1-2u)/T²`。
- 速度峰值`1.875/T`（u=0.5），加速度绝对峰值`(10/sqrt(3))/T²`（u=(3±sqrt(3))/6）。
- `T>=max(1.875/v,sqrt((10/sqrt(3))/a))`。端速度/加速度零，与静止段拼接C2。

梯形归一化行程为1：

- 若`v²/a<1`，加速时长`ta=v/a`，匀速时长`tc=1/v-ta`，`T=2ta+tc`。
- 若`v²/a>=1`，短行程退化三角形：`ta=1/sqrt(a)`、`tc=0`、`T=2ta`、实际峰值`sqrt(a)`。
- 三阶段分别恒加速、恒速、恒减速；`s`和`sd`连续，`sdd`允许有限跳变。临界行程没有重复时间或零时长匀速段。
- 多轴不是分别求时长再独立起停，而是同一归一化曲线同步；因此每轴始终满足其速度/加速度上限。

公布的`minimum_duration_s`为理论下界乘`1+1e-12`的浮点裕度，strict可直接复用，避免平方根舍入使峰值微量越限。全零行程为至少一个采样周期的hold，不产生重复时间。指定较长时长按统一时间伸缩，速度按倍率反比、加速度按倍率平方反比降低。

## 可运行入口与接口

```bash
mkdir -p build
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic -fPIC -shared \
  -Iinclude src/planning/joint.c -lm -o build/librobot_planning.so
python3 scripts/run_tests.py --suite joint_planning --smoke-python "$(command -v python3)" \
  --output results/test-runs/joint-planning
# 已按锁定环境安装时，省略--smoke-python，默认.venv/bin/python。
# CMake完整工程：cmake --build build；ctest --test-dir build --output-on-failure。
```

```python
from planning.joint import JointPlanner  # PYTHONPATH=src
planner = JointPlanner('build/librobot_planning.so')
result = planner.plan_joint([0]*6, [1, -.7, .3, 0, .02, -.1], {
    'method': 'quintic', 'sample_period_s': .01,
    'duration_s': None, 'duration_policy': 'stretch', 'max_duration_s': 60,
})
# code!=0时data=None；不得继续执行轨迹。
```

| 字段 | 类型 / 形状 | 必填 / 取值 |
|---|---|---|
| q_start_rad / q_goal_rad | 有限float(6,) | 必填，位置限位内，J1..J6 |
| method | string | 必填，cubic / quintic / trapezoidal |
| sample_period_s | 有限float秒 | 必填，[1e-9,1e6]，基线0.01；验证曲线使用0.1并额外包含极值及切换点 |
| duration_s | float秒或null | 必填，null自动；数值在[1e-9,1e6]且≥sample_period_s |
| duration_policy | string | 必填，strict / stretch；自动必须stretch |
| max_duration_s | float秒 | 必填，[1e-9,1e6]且≥sample_period_s；基线60 |
| velocity_scale / acceleration_scale | 有限float | 请求缩放可选默认1，(0,1]；配置缩放0.5在共享加载器一次应用 |
| joint_margin_rad | float | 可选默认0，≥0；收缩后位置区间须非空 |
| collision_policy | string | 可选默认unchecked；required返回1008 |

非零边界导数和其他未知选项返回1008，不静默忽略。C签名、参数顺序及失败不写输出规则见头文件；C自动时间用0，Python用null。C只接受显式6轴限值，Python负责必需配置/TCP定义/来源/单位闸门。计划总采样预算100000个基础间隔，超预算3001；没有单次可抢占超时保证，统一测试进程预算120秒。

成功Result.data保存：`method`、`boundary=rest_to_rest`、`continuity`、`time_s(N,)`、`q_rad/qd_rad_s/qdd_rad_s2(N,6)`、`segment_times_s(M+1,)`、`segment_kind(M,)`、`coefficients_rad(M,6,6)`、`effective_limits`、`continuous_joint_check`、`triangular`、`collision_checked=false`、`cartesian_limits_checked=false`。系数c0..c5按**归一化局部时间u=(t-t0)/(t1-t0)**升幂，速度/加速度导数分别除段时长/平方。三次高阶补0，梯形每子段二次高阶补0；hold单段。采样包含精确终点与切换/极值，末间隔可以短于周期。

梯形切换点加速度用右极限，末端用段内左极限；CSV/SVG保留真实跳变。三次端加速度不伪造为零。无外推：C任意时刻求值只接受[0,T]。

错误码：1001参数/长度/非法数值、1004缺配置/限值、1005位置越限、1006非法时间、1008未实现方法/边界/碰撞策略、3002严格时长超速度/加速度、3001时长或采样预算不足。全部失败`data=null`；C失败保持输出不变。

## 端点、约束及证据

[97项原始报告](../results/joint-planning/report.json)含每项输入、预期/实际码、端点误差、系数/采样交叉误差、解析峰值、左右拼接、时长与失败输出。位置阈值1e-9 rad、速度1e-8 rad/s、加速度1e-7 rad/s²。整段位置单调，极值在两端；速度/加速度检查封闭公式及独立系数求导的全部极值点，梯形两侧加速度都检查，不是只看0.1秒采样。固定随机种子20261010的8组起终点分别运行三种方法。

| 代表轨迹 | 原始采样 | 位置/速度/加速度三面板 |
|---|---|---|
| 三次混合方向 | [CSV](../results/joint-planning/cubic-mixed.csv) | [SVG](../results/joint-planning/cubic-mixed.svg) |
| 五次混合方向 | [CSV](../results/joint-planning/quintic-mixed.csv) | [SVG](../results/joint-planning/quintic-mixed.svg) |
| 长行程梯形 | [CSV](../results/joint-planning/trapezoidal-long.csv) | [SVG](../results/joint-planning/trapezoidal-long.svg) |
| 短行程三角形 | [CSV](../results/joint-planning/trapezoidal-short.csv) | [SVG](../results/joint-planning/trapezoidal-short.svg) |

报告保存源码、配置、模型、库及CSV/SVG的SHA-256。耗时只统计Python完整入口（含配置加载、C调用/采样和序列化），不是C函数耗时；本轮没有建立正式100×10性能基线，不宣称性能指标已验收。非零边界与跨段连续扩展、TCP限制映射/检查、避障及控制联调在后续任务关闭。
