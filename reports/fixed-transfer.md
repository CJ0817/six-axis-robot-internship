# 固定搬运与独立 C 核心验收

已完成任务原点、抓取、携物避障中间点、放置和返回。结果为 UR5/PyBullet 动力学仿真；抓取采用理想固定约束，放置采用静态虚拟料位，未验证真实夹爪。

|验收项|结果|
|---|---:|
|共享规划／IK／插补／控制／反馈周期|20 ms|
|时长／运动区间|54 s／2700|
|阶段末速度归零命令|4 条，单列 zero-targets.csv|
|物体质量／边长|30 g／16 mm|
|直达反例|3003，data=null；t=5.67 s|
|抓取及最大附着误差|0.02930214 mm|
|落位及返回后物体保持误差|0.01519762 mm|
|返回原点最大关节误差|5.833385e-05 rad|
|实际载荷最小间隙|11.26178 mm|
|时序／控制／指令跳变错误|0／0／0|

|运动段|全局拍号|IK 节点|认证间隙下界 mm|
|---|---|---:|---:|
|pick|0–225|200|9.157385|
|avoidance_via|225–1050|800|8.508863|
|place|1050–1875|800|10.56034|
|return|1875–2700|800|8.781178|

四段均从实际反馈重新规划并完整认证关节、TCP 导数、整臂、自碰撞及载荷。实际物体间隙与约束误差另行检查。2700 个全局运动区间拍号连续为 0…2699；4 条不推进物理时间的归零指令分别位于 225、1050、1875、2700 拍。阶段边界反馈以阶段和段内拍号区分。

纯 C 核心已通过独立 CMake 构建、CTest、安装后外部 cc 编译和 /tmp 下运行。三个库动态依赖不含 Python、PyBullet、NumPy 或 libstdc++；冻结 ABI 仍限 Linux x86_64。外部 C 示例输出 code=0、ABI=65536、IK 解数 1、executable=false；C 数值参考不等于完整碰撞认证后的设备许可。

[54 秒、540 帧演示](../results/fixed-transfer/demo.mp4) · [完整日志](../results/fixed-transfer/raw-evidence.zip) · [摘要](../results/fixed-transfer/report-summary.json) · [校验清单](../results/fixed-transfer/evidence-manifest.json)

完整日志包含每段规划／IK／认证 JSON、每拍控制和物体反馈、全局指令、归零目标、异常、抓放事件、五个阶段 PNG、MP4 与全部独立 C 构建／测试／安装／依赖日志。首次失败报告及片段保存在 first-attempt/evidence.zip：实际反馈微小姿态误差被严格 fixed 规则拒为 1007；改为实际姿态到目标的 SLERP 后重新完整认证并通过。

接口与复现命令见 [场景及接口协议](../docs/fixed-transfer.md)，问题状态、负责人和关闭条件见 [集成问题表](../docs/integration-issues.md)。未关闭限制包括真实夹爪、料台接触、硬件停车／实时性及 Windows 冻结 ABI 支持。

源码提交 7e100843cae81e2aa83b9b30af5cab9a02d6ad12；[固定搬运及独立 C Actions 通过](https://github.com/CJ0817/six-axis-robot-internship/actions/runs/38841281447)。
