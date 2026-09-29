# six-axis-robot-internship

六轴工业机器人运动规划与控制线上实习代码。

## 项目状态

已建立模块目录、冻结ABI、锁定环境与UR5仿真模型。核心算法按任务书采用C，Python负责PyBullet仿真与测试；C正运动学及UR5标称几何的普通解析逆解已实现并通过回代测试。精确腕奇异连续族全局搜索、雅可比、规划及自研控制仍待完成。当前Linux环境的构建、模型加载、点动及C/Python调用均已验证通过，详细证据见验证记录。

## 目录说明

| 目录 | 用途 |
| --- | --- |
| `src/` | 运动学、轨迹规划与控制等核心代码 |
| `examples/` | 可运行示例和实验入口 |
| `models/` | 机器人模型、参数及配置 |
| `tests/` | 后续算法验证与测试 |
| `docs/` | 环境配置、学习笔记和实验说明 |
| `reports/week01/` 至 `reports/week04/` | 每周工作记录与阶段总结 |
| `results/` | 需要保留的实验图表与结果 |

以上为初始组织方式，可随实习实际要求调整。

## 获取代码

```bash
git clone https://github.com/CJ0817/six-axis-robot-internship.git
cd six-axis-robot-internship
```

仓库当前为公开仓库，可直接克隆；向 `main` 提交仍需相应写入权限。

## 日常提交

将文件放入相应目录，检查修改后提交：

```bash
git status
git add <文件路径>
git commit -m "说明本次完成的内容"
git push origin main
```

每周总结建议记录：已完成任务、运行方法、实验结果、遇到的问题及下一步安排。

## 工程约定

开发前阅读 [模块边界、单位、关节顺序与错误处理](docs/engineering-contract.md)。
机器可读常量见 [contract.json](src/common/contract.json)。
核心代码按 `src/common/`、`src/kinematics/`、`src/planning/`、`src/control/`、`src/adapters/` 组织。

## 环境搭建

阅读 [配置与运行步骤](docs/environment-setup.md)、[版本锁](environment.lock.json) 和 [验证记录](reports/week01/environment-validation.md)。

视角切换与物体添加演示：`python examples/scene_demo.py --headless`；桌面交互使用`--gui`，键1/2切视角、J点动、Q退出。见[操作与验收说明](docs/environment-setup.md)。

## 最小 C 动态库

[编译与启动命令](docs/c-library-quickstart.md)；独立调用入口 `examples/c_library_demo.py`；[验证记录](reports/week01/c-library-validation.md)。

## 测试与参考基准

[统一测试入口与安装命令](docs/testing.md) · [量化指标表](docs/test-metrics.md) · [基准准备验证记录](reports/week01/test-baseline-validation.md)。
Robotics Toolbox 使用独立、版本及哈希锁定的环境；未实现的算法验收项明确标为待完成。

## UR5 模型核对

[型号、标准 DH、零偏及坐标变换](docs/ur5-model-conventions.md) · [机器可读配置](models/ur5/kinematics.json) · [验证结果](reports/week01/ur5-model-audit.md)。

## C 正运动学

[推导、接口与运行命令](docs/forward-kinematics.md) · [验证报告](reports/week01/c-forward-validation.md)。

## 逆解准备

[推导与接口交接、冻结目标规则](docs/ik-interface-preparation.md) · [目标文件](tests/ik/targets.json)。已准备测试输入，C解析逆解已实现，精确奇异族全局选解仍待完成。

## ABI冻结与FK专项验证

[ABI 1.0.0](docs/abi-v1.md) · [性能与鲁棒性实测](docs/abi-v1-validation.md)。保留现有FK ABI，IK符号已实现UR5普通解析分支；性能超限与待办以实测报告为准。

## C解析逆解

[实现、运行命令与边界策略](docs/c-analytic-ik.md) · [140目标及专项证据](results/c-ik-verified/ik/report.json)。标准C数学库实现，无第三方C算法依赖，保持ABI 1.0.0。


## 候选解回代与过滤

[筛选顺序、错误状态与验证](docs/ik-candidate-filtering.md)：逐候选C FK回代，剔除越限、重复和误差不合格解；部分候选失败时保留其余有效解。统一入口`--suite ik_filter`。

## 连续状态综合选解

[评分、限步与奇异区处理](docs/ik-stateful-selection.md) · `examples/ik_selection_demo.py` · 统一测试`--suite ik_selection`。Python策略层使用C候选及C FK，冻结ABI保持不变。

## 逆解目标分组与收尾

[典型可达、不可达、近限位验证及推导索引](docs/ik-target-validation.md) · [阻塞处理与本轮收尾](reports/week02/ik-target-closeout.md)。统一入口`--suite ik_targets`；锁定环境40项目标、232条逐解双参考回代通过。

## 固定种子120组工作空间输入

[采样规则与复现](docs/workspace120-dataset.md) · [120主样本](tests/ik/workspace120.json) · [边界/异常清单](tests/ik/workspace120-special.json)。seed=20260928，24区各5组；新增输入集不替换现有IK测试集。

## 冻结120组批量验收

[120目标C/Robotics Toolbox逐项对照、边界与异常结果](docs/workspace120-batch-validation.md) · [完整报告](results/workspace120-batch-run/workspace120_batch/report.json)。120组主目标已实际执行IK，890候选双回代通过；边界和异常各自统计。

## 运动学阶段汇总与短演示

[误差、成功率、耗时与推导](docs/ik-summary-and-derivation.md) · [140组逐例来源](results/closeout-ik/ik/report.json) · [编译/运行复核与演示说明](docs/build-and-demo-check.md) · [阶段周报](reports/week02/kinematics-weekly-summary.md)。短视频是云端 PyBullet DIRECT 渲染的三个关键帧；后续的[用户 Windows 桌面录像与 GUI 证据](results/local-demo/README.md)另行归档，不在阶段标签 `v0.2.0-kinematics` 内。
