# 单关节增量式 PID v0.1

本轮接续云端执行者最后一项：实现 C11 单关节增量式 PID、积分/输出限幅和复位，执行阶跃、正弦、定点试验并保存参考、反馈和控制输出。独立 `robot_control` 库不修改运动学 ABI v1.0.0，不承担规划、设备 I/O 或完整故障状态机。

## 算法与状态

固定周期 dt，误差 e=r-q，矩形积分：

```
I_candidate = clip(I_previous + Ki*dt*e, I_min, I_max)
delta_P = Kp*(e-e_previous)
delta_D = Kd/dt*(e-2*e_previous+e_previous_previous)
raw = raw_previous + delta_P + (I_candidate-I_previous) + delta_D
u = clip(raw, u_min, u_max)
```

若积分增量会把 raw 继续推向输出饱和方向，本周期冻结积分贡献，再计算 raw。原始累加器与实际限幅输出分别保存，避免输出截断丢失 P/D 历史；无饱和时与位置式 PID 独立公式等价。微分针对误差，参考跳变会产生微分冲击，原始输出完整留档；本轮无微分滤波或前馈。积分限值的单位与输出一致，不是未经 Ki 缩放的误差积分。

`robot_pid_reset(config,state)` 验证配置后将积分、两次误差、原始输出和实际输出全部归零。`robot_pid_step(state,reference,actual,result)` 返回0才提交新状态/结果；1001参数错误、1006配置周期错误、4001历史状态损坏。失败不修改可用输出和输入状态。饱和是正常限幅，返回0并置诊断标志，不等同于完整控制状态机的超限故障处理。

## 冻结试验

唯一配置为 `config/single-joint-pid.v1.json`，dt=0.01s，每类8s、800条采样。记录时序为：t[k]读取反馈→计算输出→保持输出推进一个dt。末行是7.99s的反馈，不把推进后的8s状态伪记为该输出作用前的反馈。

| 试验 | 初值与参考 |
| --- | --- |
| 阶跃 | q0=0；0.5s从0跳至0.2rad |
| 正弦 | q0=0；幅值0.1rad、频率0.2Hz，5s后保持0 |
| 定点 | q0=0.1rad，固定参考0.15rad |

两种被控对象分开记录：积分模型 `q[k+1]=q[k]+dt*u[k]` 的输出为rad/s；UR5 PyBullet DIRECT 的 J1 为自研PID力矩输出，单位N m，关闭J1默认电机，其余五轴用内置位置伺服保持指定姿态。每周期4个物理子步，力矩每子步重发。重力设为0以隔离单轴基础试验，不宣称重力补偿或整机负载能力。位置/惯量/碰撞模型直接使用仓库URDF；J1名称和六个转动关节数量强制核对。

输出每周期含node/time/reference/actual/velocity/error/raw_output/output/integral、两类限幅标志与错误码。每类保存CSV、SVG；report.json保存配置、源文件/动态库/CSV/模型哈希、环境和统计。报告RMSE、最大误差、末值误差、最后1秒平均偏差、超调、10–90%上升时间、持续进入误差带的调节时间和限幅次数；未达到的响应指标为null。正弦无阶跃超调/上升时间，调节时间从5s结束正弦后计算。

本轮功能闸门仅检查有限值、800条完整数据、位置绝对值≤1rad、积分/输出不越配置限值、末值误差≤0.02rad。阈值在运行前冻结；不替代CTRL-01的20条轨迹、CTRL-02的前馈/扰动对照、CTRL-03故障状态机或CTRL-04实时周期验收。

## 运行

```bash
mkdir -p build
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic -fPIC -shared -Iinclude src/control/pid.c -lm -o build/librobot_control.so
.venv/bin/python scripts/run_tests.py --suite single_joint_pid --output results/single-joint-pid-integrator
.venv/bin/python scripts/verify_single_joint_pid.py --library build/librobot_control.so --plant pybullet --output results/single-joint-pid-pybullet
```

也可通过CMake构建独立控制库并运行 `ctest -R single_joint_pid --output-on-failure`。专用GitHub Actions自动执行严格C11编译、C接口/历史/限幅/复位回归、三类积分模型和三类UR5力矩基线，始终上传原始证据；缺依赖、缺模型或失败不跳过/伪成功。C接口功能回归含200个独立位置式对照点、正负积分/输出限幅、持续饱和后释放、reset、NULL/NaN/Inf/溢出及损坏状态不推进。
