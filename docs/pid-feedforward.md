# 单关节速度／加速度前馈与扰动对照

在原增量式 PID 上增加独立 C11 符号 `robot_pid_step_ff`。原 PID 配置、状态、结果布局及
`robot_pid_reset`、`robot_pid_step` 均保持兼容。Python 使用 `FeedforwardPID`。

```
feedback = incremental_PID(reference_position - measured_position)
velocity_ff = Kv * reference_velocity
acceleration_ff = Ka * reference_acceleration
total_raw = feedback + velocity_ff + acceleration_ff
actuator = clip(total_raw, output_min, output_max)
```

仅对合成后的总力矩限幅。条件积分抗饱和以总力矩为判据，状态中的 `raw_output` 只保存
PID 反馈累加器，前馈不逐周期累加。非法输入、溢出或损坏状态时，不修改状态及结果。
原 PID 的 230 项回归继续运行；新回归覆盖独立公式对照、零前馈等价、正负总饱和、
相反符号前馈、复位、异常输入和指标定义。

## 预先固定的参数

| 参数 | 两组共同设置 |
|---|---|
| 周期／时长／重复数 | 0.01 s／24 s／每种组合重复 2 次 |
| PID：Kp、Ki、Kd | 80、15、12；输出单位 N·m |
| 积分项限幅／执行器限幅 | ±3 N·m／±12 N·m |
| 附加负载 | 0、0.75 kg，均加在 `wrist_3_link` 原惯性中心；球体内禀惯量半径 0.04 m |
| 重力／物理步长 | [0,0,0] m/s²／控制周期内 4 个物理子步 |
| J1 黏性阻尼 | 0.4 N·m·s/rad；关闭额外线／角空气阻尼 |
| 其他关节 | 相同初始姿态、相同位置伺服和 200 N·m 力矩设置 |
| PID 组 Kv、Ka | 均为 0 |
| PID+FF 组 Kv | 0.4 N·m·s/rad，与显式模型阻尼对应 |
| PID+FF 组 Ka | 加载后初始构型质量矩阵 M[0,0]；每种负载计算一次，不根据结果调参 |

完整配置为 `config/pid-feedforward.v1.json`。每次试验重新创建物理世界、复位控制器，
两组复用同一份解析参考和扰动力矩数组，保存轨迹／扰动哈希与实际参数，并核对重跑差异。
二阶模型与真实 UR5/PyBullet 的结果分别保存，不混为物理验证。
Kp、Ki、Kd 的单位分别为 N·m/rad、N·m/(rad·s)、N·m·s/rad。

## 轨迹及扰动

- `move`：0.5 s 起用 2 s 五次曲线从 0 到 0.2 rad，之后保持。
- `sine`：0.5–8.5 s，0.12 rad、0.25 Hz 正弦乘 sin² 包络，端点为 C2，之后保持 0。
- `hold_pulse`：保持 0.15 rad，4≤t<6 s 加 +2 N·m 扰动。
- `hold_sustained`：保持 0.15 rad，t≥4 s 持续加 +2 N·m 扰动。
- `sine_pulse`：与 `sine` 完全相同，同时施加 4≤t<6 s 的 +2 N·m 扰动。

扰动是执行器限幅之后的独立等效 J1 广义力矩，不占用／放宽执行器 ±12 N·m 限幅。
日志分别保存执行器力矩、扰动力矩及两者合计。参考速度和加速度由解析表达式计算，
不从实际反馈或位置阶跃数值差分构造。每种物理对象 40 次试验，共 96,000 条逐周期记录。

## 统计规则

- `tracking_rmse_rad`：整个 0–23.99 s 记录的跟踪 RMSE；另存运动有效区间的
  `tracking_active_rmse_rad`，防止长保持段稀释运动误差。
- 稳态误差：最后 1 s 有符号平均误差和绝对误差平均值，单独保存末值。
- 超调：仅 `move`，在参考到达终点后，沿运动方向的最大越过量及其占 0.2 rad 的百分比。
  正弦／保持试验不定义阶跃超调百分比，填 null；扰动另报峰值绝对跟踪误差。
- 调节时间：`move` 从 2.5 s 起，正弦从 8.5 s 起，保持脉冲从 6 s 起，持续扰动从 4 s 起。
  取最后一次超出误差带后的首个采样点，且尾段须持续不少于 0.5 s。
  误差带为 max(0.001 rad, 2%×指令行程)。观察窗内未达到则填 null，绝不填 0。
- 扰动恢复时间：以脉冲撤除或持续扰动加入时刻为起点，基于跟踪误差计算；另存扰动前、
  期间和之后的 RMSE。动态试验不满足恢复误差带时保留 null。

输出包含每次 CSV、两组叠加 SVG、完整 `report.json`、`parameters.csv` 和 `comparison.csv`。
报告展示前馈改善及退化结果，不强制所有试验都优于 PID。静止时前馈为零，不能据此宣称
提升静态扰动抑制，也不包含重力补偿、完整计算力矩或六轴闭环验收。

## 运行

```bash
python -m unittest discover -s tests -p test_feedforward.py -v
python scripts/verify_pid_feedforward.py --library build/librobot_control.so --output results/pid-feedforward-second-order
python scripts/verify_pid_feedforward.py --library build/librobot_control.so --plant pybullet --output results/pid-feedforward-pybullet
```

`Single joint PID` CI 保留旧 PID 全部验证，并追加以上三项。测试失败保留已获得的原始记录
和失败报告，返回非零退出码。固定配置的末值误差及位置界仅为功能检查，不等同于完整控制验收。
