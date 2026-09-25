# 2026-09-25 逆解测试收尾

已完成40项分组验证、推导/接口索引整理和本轮运行阻塞处理；没有改变冻结ABI或UR5模型参数。

| 验证 | 结果 | 证据 |
|---|---|---|
| 典型可达/不可达/近限位/软件限位排除 | 6+9+24+1全部符合预期；232候选C/RTB双回代通过 | results/ik-targets-verified/ik_targets/report.json |
| 既有固定逆解目标及专项 | 100普通+20宽初值+20近奇异；52专项通过 | results/ik-closeout/regression-summary.json |
| 综合选解 | 100案例及5检查通过 | 同上 |
| 新建Release构建与CTest | 5/5通过 | results/ik-closeout/ctest.log |
| 冻结ABI | 布局/符号/错误码/失败不改输出通过 | results/ik-closeout/regression-summary.json |

环境：Python3.11.9、NumPy1.26.4、Robotics Toolbox1.1.1、GCC13.3.0、CMake3.31.6。功能回归使用原-O2库，其SHA256在报告记录；新建Release构建用于CTest，未用它替换原库的测试证据。

本轮开始时缓存解释器缺失、venv入口失效，启动返回127。通过uv重新安装3.11.9，并用该解释器对既有两个venv执行`-m venv --without-pip`恢复入口，保留原有锁定依赖；版本检查及上述实际测试通过。依赖锁没有修改。此恢复方式仅适用于依赖目录仍完好的同版本缓存环境；仓库迁移仍按既有规则完整重建环境和构建缓存。

**本轮无未解决安装／路径问题。** 缓存复发问题RUNTIME-01已按本轮实测重新关闭。ALG-01旧表述已更新，普通解析IK不再列为待实现；C解析雅可比和通用精确奇异族仍未关闭。

已查验父提交d128d51的GitHub Actions运行36115381004：environment和robotics-toolbox-baseline两作业全部步骤通过，包括锁定环境综合选解。原始步骤状态摘要保存在results/ik-closeout/parent-ci.json；这不是本次新提交的CI结果，不混用提交版本。

仍保留以下非本轮阻塞项：

- PERF-01：原FK FFI单次长尾尚未定位；父提交CI性能步骤通过不能替代其关闭条件。
- IK-SING-01：精确腕奇异连续族全局搜索未实现；已有保持、小步退出与穿越拒绝策略继续有效。
- C解析雅可比、DLS、规划/控制及加速度限值确认：按后续任务推进，不混作安装故障。

推导与实验入口统一见[目标验证说明](../../docs/ik-target-validation.md)。候选生成、回代过滤、选解和控制职责保持分离，不能把IK几何通过直接当作可执行设备运动。
