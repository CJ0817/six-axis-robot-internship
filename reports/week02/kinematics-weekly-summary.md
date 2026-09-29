# 第二周运动学阶段周报（截至 2026-09-29）

项目：六轴工业机器人运动规划与控制线上实习。目标：固定 UR5 CB 标称模型、冻结 C ABI，完成正逆运动学推导、实现、批量验证及阶段演示。阶段源码标签 `v0.2.0-kinematics` 固定指向提交 `782d0179e95386ebc840fa47fa1010b70753542f`。用户 Windows 本机录像在标签之后的证据提交 `593edf2686b1d2399b21d9ff497edb44b27d6669` 中归档，不包含在该标签快照内。本页随证据归档更新，汇总两者时区分源码版本与后续验证证据。

## 本阶段完成

| 工作 | 交付与可复核证据 |
|---|---|
| 模型与接口 | UR5 CB 标准 DH、关节符号/零偏和 base/flange/tool0 固定变换核对；ABI 1.0.0 固定函数签名与错误码，见 [模型约定](../../docs/ur5-model-conventions.md)和[ABI](../../docs/abi-v1.md)。 |
| C 正/逆运动学 | C11 FK、普通离散分支解析 IK；最多 8 个候选，等价角限位映射、C FK 回代、去重排序与相邻关节状态选解。精确腕奇异 `all` 返回 2003，连续族全局搜索未实现。 |
| 常规与特殊验证 | 固定种子 120 个工作空间目标全部成功，890 个候选经 C FK 与显式参数 Robotics Toolbox 双回代；边界 16/16、异常 11/11 符合预期。另 140 个逆解专项样本全部通过，52 项专项通过；原始逐例报告和哈希见[运动学测试报告](../../docs/ik-summary-and-derivation.md)。 |
| 汇总与问题收口 | 误差有效数、均值、RMSE、P95、最大值与 C／Python→C 分层耗时可从已提交逐例数据重新计算；上轮因缺失 140 组原报告而重跑、保存新报告，旧耗时不再引用。未复现普通/边界组的算法失败，见[排查记录](../../results/normal-boundary-triage/report.json)。 |
| 编译与演示 | 标签快照中的锁定 Python/GCC/CMake 构建、CTest 5/5 以及直接 GCC 最小 ABI 调用通过；PyBullet DIRECT 三帧演示形成 7.5 秒视频与哈希清单，见[构建和演示记录](../../docs/build-and-demo-check.md)。标签之后另归档了用户 Windows 本机 GUI 连续录屏、环境和按键报告，见[本机录像证据](../../results/local-demo/README.md)。 |

## 可量化结果

- 120 组主目标成功率 **120/120**，890 候选回代位置误差最大 **9.587×10⁻¹⁶ m**、姿态角最大 **1.946×10⁻¹⁵ rad**；该报告 C 单次调用平均／P95／最大 **0.04127／0.06461／0.24320 ms**。
- 新的 140 组逐例报告：普通 **100/100**、宽初值 **20/20**、近奇异 **20/20**。普通组最大 C 单次耗时 **2.09753 ms**；52 项专项均通过。此性能数值是单次无预热观测，不等同于正式重复测量的验收。
- 16 个边界预期状态全部符合，其中 4 个精确腕奇异按设计返回 2003；11 个异常样本的错误码及失败输出不变检查全部通过。错误返回不充作成功逆解。
- 标签内的云端 DIRECT 场景：正面／侧面 RGB 平均差 **13.15697**，物体位置最大绝对误差 **0 m**，三帧及短视频 SHA-256 保存在[清单](../../results/demo-short/manifest.json)。
- 标签后本机 GUI 证据：**94.3 秒** Windows 连续桌面录像（1920×1200，H.264），SHA-256 为 `8621b51796b404eef58f58f76080d1b836b15e0b6490a135f9df51680d00d163`。GUI 报告为 `status=passed`、`gui_keyboard_verified=true`，按键切换 side/front/side/front，中间 J1 点动目标 0.1 rad；原始报告、机器记录、事件与录像校验见[本机证据目录](../../results/local-demo/README.md)。

## 当前限制与后续安排

1. 标签内的 DIRECT 短视频只展示云端渲染关键帧；**本轮用户 Windows GUI 连续录屏已在标签之后归档**，不再列为待办。GUI 程序的 `user_windows_validation` 字段仍保留 `not_established_by_this_run`：单份程序报告不能独立证明机器归属，本机结论由 Windows 桌面录像、机器/WSL 记录和按键报告共同支持。2026-09-15 的本机 GUI 恢复记录仍是单独的历史证据。
2. 雅可比、三次/梯形与圆弧规划、增量 PID/控制闭环仍待实现；精确腕奇异连续族的通用最近解、碰撞和实机标定也未完成。下一步按指标表依次推进并分开统计正常、边界与失败目标。
3. 正式 FK/IK 性能验收仍需固定硬件条件、预热与重复次数；云端无预热功能测试的耗时和偶发长尾只作基线。当前版本不对实机安全运行作保证。

## 复现入口

```bash
# 在满足 docs/environment-setup.md 版本锁的 Linux/WSL 环境
source .venv/bin/activate
cmake -S . -B build/release-check -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/release-check
ctest --test-dir build/release-check --output-on-failure
python scripts/summarize_ik_evidence.py --workspace-report results/workspace120-batch-run/workspace120_batch/report.json --ik-report results/closeout-ik/ik/report.json --output results/ik-summary
python scripts/record_short_demo.py --output results/demo-short
```

完整构建命令及计时局限见链接文档。图表引用的是同一组已提交的逐例运动学报告；重复测试会改变耗时与报告 SHA-256，需重新汇总。本机录像是后续验证证据，不改变 `v0.2.0-kinematics` 的标签指向。
