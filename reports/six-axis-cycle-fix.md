# 六轴同周期与异常现场修正验证

本记录取代原 20 ms 规划／10 ms 控制的多速率结果，用于本次同周期验收。全部试验为 UR5/PyBullet 3.2.7 动力学仿真，未连接真实机器人。

唯一周期配置为 `config/cartesian-planning.v1.json:sample_period_s=0.02` s，六轴配置通过 planning_config 引用它；PID、时钟和反馈由加载器注入这一值。规划／IK、插补、控制和反馈均为 20 ms。物理积分内部为 4 个 5 ms 子步。认证结果的周期、规划和插补时间网格、IK 节点时间/序号、时钟、PID 及反馈周期在执行前检查，失配返回 1006，不做自动转换。

|环节|周期 s|
|---|---:|
|planning_s|0.02|
|ik_s|0.02|
|interpolation_s|0.02|
|control_s|0.02|
|feedback_s|0.02|

固定轨迹与控制参数见 [执行协议](../docs/six-axis-integration.md)。与原任务相同的起点、笛卡尔目标、6 s 运动加 2 s 保持，重复两次。300 个 IK 节点全部成功，完整约束认证通过。两次各 401 拍反馈、400 个运动区间；共 802 拍反馈、800 个运动区间和 3200 个物理子步。两次 q、qd、参考、命令、积分和限幅记录完全一致，墙钟耗时除外。

|关节|运动幅度 rad|RMSE rad|最终绝对误差 rad|实际峰值速度 rad/s|
|---|---:|---:|---:|---:|
|J1|0.001988029|0.000002125566|0.000001635924|0.0006222782|
|J2|0.02639895|0.00001172071|0.000007395970|0.008259692|
|J3|0.02341414|0.000005707842|1.185376e-7|0.007324968|
|J4|0.007277187|0.0001047228|0.00008137237|0.002285310|
|J5|0.006074538|0.000002874348|0.000001929365|0.001900833|
|J6|0.01888327|0.00004322483|0.00003359623|0.005914136|

正常运行两次均为：时序错误 0，参考／命令跳变 0，控制异常 0。实际反馈最小间隙 0.0094673701 m，墙钟处理超过 20 ms 的次数为 0（不代表真实系统的实时调度保证）。

|拒绝或注入案例|返回码|检测 tick|此前运动写入数|验证|
|---|---:|---:|---:|---|
|duplicate_tick|1006|5|5|通过|
|skipped_tick|1006|5|5|通过|
|reversed_tick|1006|5|5|通过|
|stale_feedback|1006|5|5|通过|
|wrong_feedback_time|1006|5|5|通过|
|nonfinite_feedback|4001|5|5|通过|
|missing_feedback|4001|5|5|通过|
|feedback_overspeed|4001|5|5|通过|
|tracking_error|4001|5|5|通过|
|reference_jump|4001|5|5|通过|
|controller_invalid|4001|5|5|通过|
|controller_jump|4001|5|5|通过|
|collision_feedback|3003|5|5|通过|
|uncertified|3001|0|0|通过|
|tampered|3001|0|0|通过|
|cycle_mismatch|1006|0|0|通过|
|feedback_cycle_mismatch|1006|0|0|通过|

17 个拒绝／注入案例全部符合预期；缺配置返回 1004 另验证通过。cycle_mismatch 保持原轨迹认证有效，仅把执行／PID／时钟改为 10 ms，认证门后立即因周期失配返回 1006；feedback_cycle_mismatch 单独把反馈周期改为 10 ms，同样在读取反馈和运动写入前返回 1006。两者运动写入均为 0。运行时故障在 tick=5、0.10 s 拦截，只有此前 5 个区间，无本拍运动写入。所有失败均不返回可执行数据。

异常上下文每拍从 null 重新初始化。本次实际 NaN 注入现场：

```json
{
  "tick": 5,
  "time_s": 0.1,
  "observed_command": [
    {
      "nonfinite": "NaN"
    },
    -0.0000543601479145517,
    0.000033603247539284463,
    -0.00019831425959279336,
    -0.000012864621336694529,
    0.0001025356808174728
  ],
  "validated_command": null,
  "previous_sample": {
    "tick": 4,
    "time_s": 0.08,
    "observed_command": [
      -0.000004789773988990608,
      -0.000038387116410155465,
      0.000021791321788419358,
      -0.00016296198872716605,
      -0.000009121282294993536,
      0.00008044295501792387
    ]
  }
}
```

raw_control_outputs[0].data.output 同样为 {"nonfinite":"NaN"}；完整六轴原始返回值在校验前采集，未被上一拍命令替换。NaN/Infinity 用显式对象编码，未获取数据用 null，日志通过严格 JSON 序列化。

重复 tick 的实际现场：

```json
{
  "tick": 5,
  "time_s": 0.1,
  "observed_clock": {
    "tick": 4,
    "time_s": 0.08
  },
  "observed_feedback": null,
  "observed_reference": null,
  "raw_control_outputs": null,
  "observed_command": null,
  "validated_command": null,
  "previous_sample": {
    "tick": 4,
    "time_s": 0.08
  }
}
```

同时复核过期、错误时间、缺失、非有限和超速反馈的日志：尚未进入插补／控制的本拍字段均为 null；上一拍独立保存在 previous_sample，并标明 tick=4。原始 CSV、全部异常 JSON 与报告见 [证据包](../results/six-axis-cycle-fix/raw-evidence.zip)，[机器报告](../results/six-axis-cycle-fix/report.json) 和 [SHA256 清单](../results/six-axis-cycle-fix/evidence-manifest.json)。

[源码提交 035ea10](https://github.com/CJ0817/six-axis-robot-internship/commit/035ea10d531dbe82eca61e124f4f7b4278428df1)；[Six axis integration Actions 通过](https://github.com/CJ0817/six-axis-robot-internship/actions/runs/38738375131)。13 项时钟／周期／日志回归通过。协议中的位置 PID 外环、速度内环及仿真停止限制继续适用。
