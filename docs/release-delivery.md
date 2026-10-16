# v1.0.0-internship 交付说明与冻结范围

本版本冻结 UR5 CB 的 Linux x86_64 / CPython 3.11.9 / PyBullet 3.2.7 仿真软件。标签只在同一提交的既有完整回归及新的干净目录、重复和连续运行验收全部通过后建立；不覆盖已存在的其他版本标签，不改写历史。

## 交付内容

源码：src/、include/、core/、examples/、scripts/。配置与模型：config/、models/、scenes/、requirements*、environment.lock.json。报告与证据：reports/、results/，以及发布包 acceptance/ 下的本次新日志、逐拍数据和视频。接口、安装与移植说明见 docs/fixed-transfer.md、docs/abi-v1.md、docs/ur5-model-conventions.md；问题表见 docs/integration-issues.md。交付包另包含本次最终报告、源码清单和 SHA256SUMS。

独立 C 核心：

```sh
cmake -S core -B build-core -DCMAKE_BUILD_TYPE=Release
cmake --build build-core --parallel 2
ctest --test-dir build-core --output-on-failure
```

仿真依赖与单次演示：

```sh
python -m pip install -r requirements/release.lock
python examples/fixed_transfer_demo.py --output /tmp/new-transfer-evidence
```

重复与连续验收（先构建 build-core，输出目录必须不存在）：

```sh
python scripts/verify_release_cycles.py --source-commit "$(git rev-parse HEAD)" --output /tmp/new-release-evidence
```

## 验收矩阵

- 功能、边界、异常：当前提交的 Environment verification（含 RTB）、Trajectory certification、Single joint PID、Six axis integration、Fixed transfer and independent C core 均必须通过。既有超限 3002、碰撞 3003、不可达 2001、NaN、缺配置、周期失配、未认证轨迹和失败状态保留继续回归。
- 干净目录：从当前 Git 提交 archive 到新临时目录，断言构建和输出目录不存在；重新安装锁定依赖、重新编译，不使用已跟踪结果作为本次成功报告。
- 独立重复：3 个新进程，各完成一次 54 秒搬运，逐项比较完整控制、物体反馈、全局指令及归零记录；仅墙钟处理耗时排除，数值容差 1e-9。
- 连续运行：同一进程、同一物理客户端、同一机器人和物体连续执行 3 个往返周期，交替 A→B、B→A、A→B，不重置关节或物体状态。每段均从实际反馈重新认证；为远端取料，将此模式的取料段放慢到 16 秒。共 198 秒、9900 个 20 ms 区间、12 条阶段归零指令。检查跨周期状态衔接与时钟连续性，保存全部数据与视频。
- 失败：任何必需检查失败，不建立标签或发布；失败日志照常上传，修复后重跑当前提交。

## 移植要点与保留限制

冻结 C ABI 是 Linux x86_64 LP64、小端 binary64；调用前核对布局和符号，矩阵为行优先 4×4，关节顺序 J1–J6，单位 m/rad/s。依赖路径显式传入，禁止拷贝旧虚拟环境、CMake 缓存或跨机器动态库。移植后从新目录重编译并重跑接口及模型对齐，不可把 Windows 数学测试作为冻结 ABI 验收。

周期唯一来源为 config/cartesian-planning.v1.json，规划/IK/插补/控制/反馈保持一致。配置、模型、场景或库改变后必须重新认证；进程内证书不可跨进程复用。不同载荷、工具、障碍、限值或驱动器须重新验证。

夹爪为理想约束，料位为静态虚拟支撑；未证明真实摩擦抓取、料台接触、硬件急停或实时总线。C ABI DLS、C Jacobian 尚未提供。历史总指标中的正式时延协议、硬件故障锁存及完整通用业务适配器不因本次仿真冻结而变成已通过；逐项排除原因保留在 config/release-acceptance.v1.json 和最终报告。共享 CI 的性能样本保留离群值，不能作为硬实时承诺。

## 总结

项目已形成 C 运动学/规划/控制核心、Python 认证和连续选解、同周期六轴闭环、携物碰撞认证、固定搬运与独立 C 消费示例。曾发现并回归的关键问题包括候选轨迹误用、多速率混称同周期、NaN 现场混入上一拍命令、实际反馈姿态误差导致段衔接拒绝，以及阶段末归零记录遗漏。最终以可重跑测试、原始失败记录、版本化配置和内容校验清单交付；保留限制按上文单列。
