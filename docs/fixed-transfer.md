# 固定搬运场景与接口示例

任务原点是 `[0.3,-1.2,1.1,-1.1,1.0,0.2]` rad 的已检查安全姿态，不是编码器零位或六关节全零。流程为原点→抓取→避障中间点→放置→返回。配置在 `config/fixed-transfer.v1.json`；所有位姿、关节与时间使用 m、rad、s，关节顺序 J1–J6。

唯一控制周期继续取自 `config/cartesian-planning.v1.json:sample_period_s`，规划／连续 IK／插补／控制／反馈均为 20 ms，物理内部 4×5 ms 子步。整个任务只有一个整数场景时钟；每段使用其局部时间视图进行认证轨迹求值，不另启时钟。阶段结束与开始的反馈允许在同一个全局拍号发生，日志带阶段名和段内拍号；实际运动写入必须全局拍号严格连续、每拍唯一。

抓取使用理想固定约束，载荷为 30 g、边长 16 mm 方块，工具坐标系偏移 `[0,0,0.1]` m。抓取前检查实际物体和抓取点距离不超过 3 mm；附着后实际约束位置误差不超过 3 mm。放置后移除抓取约束并将物体切换为静态虚拟料位，保留释放时实际位姿，不把物体传送到目标。落位误差与返回后物体保持误差均不超过 3 mm；关节返回误差不超过 0.001 rad。没有真实夹爪、摩擦力闭合或料台支撑模型，不能据此证明真实硬件抓取可靠性。

障碍位于抓取与放置物体中心的中点。必须先验证携物直达路径返回 `3003`、`data=null`；再走抬高 65 mm 的中间点。每段均从实际反馈起点重新规划和完整认证，执行入口继续验证证书和同周期。携物碰撞检查包含方块对障碍、方块对机器人、整臂及自碰撞；连续运动包络半径增加工具偏移及方块顶点半径。附着时只排除工具与 wrist_3 相邻装配的预期接触，列表随场景元数据保存。动力学客户端另检查实际物体间隙和约束误差。计划连续证明不等于对实际跟踪误差管道的连续证明。

六轴位置 C PID 输出限幅速度，PyBullet 执行速度内环。阶段切换时速度目标归零，后一段首条实际速度命令也检查加速度限幅。失败停止推进仿真、保留此前日志和错误现场；只成功认证的轨迹能进入执行器。

## 独立 C 编译与调用

冻结运动学 ABI 的支持平台仍为 Linux x86_64、LP64、小端 binary64。`core/CMakeLists.txt` 是仅含 C 的入口，不查找 Python、不启用 C++，三个核心库只依赖系统 C／数学库。它与原根 CMake 构建并存，不改变现有 ABI。

```sh
cmake -S core -B build-core -DCMAKE_BUILD_TYPE=Release
cmake --build build-core --parallel 2
ctest --test-dir build-core --output-on-failure
cmake --install build-core --prefix "$PWD/build-core/install"
cc -std=c11 -Ibuild-core/install/include examples/core_api_demo.c \
  -Lbuild-core/install/lib -Wl,-rpath,"$PWD/build-core/install/lib" \
  -lrobot_contract -lrobot_planning -lrobot_control -lm -o build-core/external-consumer
./build-core/external-consumer
```

`examples/core_api_demo.c` 是完整外部消费者：构造明确的标准 DH 示例模型→FK→解析 IK→关节规划／求值→PID 加速度／速度前馈接口调用，并验证缺模型返回 `1004`、输出缓冲区不变。示例配置 Ka=0、Kv=1、输出 rad/s；其模型是教学夹具，实际搬运使用审计后的 `models/ur5/kinematics.json`。C 关节规划结果本身不具备 Python 层完整碰撞／TCP 认证，示例输出 `executable:false`，不能直接下发设备。

头文件契约为 `include/robot_kinematics.h`、`robot_planning.h`、`robot_control.h`。每次先检查返回码，非零不读取旧输出；状态与输出缓冲区由调用者持有。FK 矩阵为行优先 double[16]，IK 参数长度分别为 16、6；PID 状态每段显式复位，配置时间必须与当前周期一致。

## Python 集成与演示

```sh
python -m pip install numpy==1.26.4 pybullet==3.2.7 Pillow==11.0.0 imageio-ffmpeg==0.6.0
python examples/fixed_transfer_demo.py --output results/fixed-transfer
```

这个入口调用 `scripts/verify_fixed_transfer.py`，库路径可用 `--library`、`--planning-library`、`--control-library` 显式指定。集成顺序可直接参考该脚本的 `main`：加载共享周期→场景和载荷→从反馈构造起点→`CartesianPlanner.plan`→`SixAxisExecutor.run`→抓放状态变化后重新认证下一段。`PayloadScene` 隔离碰撞模型与物理客户端，`SegmentClock` 只映射唯一时钟。

结果目录保存：每段完整规划／IK／认证 JSON、每拍控制 CSV、原始物体反馈和全局指令 CSV、异常 JSON、状态事件与验收指标 `report.json`、纯 C 构建／安装／测试及动态依赖日志、`demo.mp4` 与五个阶段 PNG。视频由实际物理状态采样，10 fps、正常仿真速度，右下角为物体与障碍近景。视频不是轨迹示意动画。所有失败也上传已有证据，不能用最终截图代替完整记录。
