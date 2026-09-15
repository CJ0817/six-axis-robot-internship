# six-axis-robot-internship

六轴工业机器人运动规划与控制线上实习代码。

## 项目状态

已建立模块目录、接口约定、固定版本环境与UR5仿真模型。核心算法按任务书采用C，Python负责PyBullet仿真与测试；已加入C/Python调用验证，运动学/规划/自研控制算法尚未实现。当前Linux环境的构建、模型加载、点动及C/Python调用均已验证通过，详细证据见验证记录。

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

仓库为私有仓库，克隆时需要具备访问权限的 GitHub 账号。

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
