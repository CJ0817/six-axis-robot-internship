# 统一认证入口交付记录

日期：2026-10-15。基准提交：`e812b215b6bf7d6c50bbc11a8240e8756ef97fac`。

已实现：

- `plan_joint` 接入与笛卡尔路径共用的完整认证；检查原多项式的连续关节位置、速度和加速度，以及 TCP 导数界、整臂／自碰撞和连续净空运动界。
- `plan_candidate` 明确返回不可执行候选。执行入口校验认证凭证、轨迹内容、时间参数、配置、模型、动态库和当前场景；拒绝时不调用消费回调。
- 完整入口约束超限统一 `3002`。自动延时重新生成候选，并重新运行全部认证。
- 缺配置、碰撞或认证失败时 `data`、`next_state` 均为空；记录失败阶段、错误码、时间、碰撞节点和重试过程。
- 加入统一测试入口、CMake 测试及独立 `Trajectory certification` CI 工作流；继续使用已有固定碰撞反例。

已运行验证：

| 验证 | 结果 | 范围 |
|---|---|---|
| 严格 C11 关节规划编译 | 通过 | Windows 本地 Zig，`-Wall -Wextra -Werror -pedantic` |
| 原有候选轨迹回归 | 97/97 通过 | 三次、五次、梯形、边界、错误码、系数及极值 |
| 新增认证／门禁单元测试 | 12/12 通过 | 真实 C 规划与 FK；碰撞使用确定性测试替身，仅验证拒绝传播和净空证明逻辑 |
| Python 语法检查 | 通过 | 新增及修改的规划、回归、测试入口 |
| 真实 PyBullet 整臂／自碰撞集成回归 | 37/37 通过 | Linux、Python 3.11.9、NumPy 1.26.4、PyBullet 3.2.7 |
| 原有关节候选回归（云端重跑） | 97/97 通过 | 完整认证专项中的独立回归步骤 |
| 原有笛卡尔、平滑和缩时回归 | 35/35、50/50 通过 | 使用真实 URDF 几何，原有回归全部保留 |
| PID、环境验证、Robotics Toolbox 基准 | 全部通过 | 与认证修复同一提交触发 |

本地 FK 检查使用未修改的 C FK／IK 源码，额外预包含一个 Windows 计时适配头以编译
Linux 时钟调用；此次认证测试未调用 IK。该临时适配头和本地 DLL 不属于生产交付，
也不替代冻结的 Linux ABI 验证。

`results/trajectory-certification-local/report.json` 和 `unit-test-output.txt` 保存新增本地结果；
`results/joint-candidate-regression/` 保存 97 项原始报告、CSV 和曲线。

用户已明确授权本次修复与证据推送到 `main`，且禁止强制推送、删除分支和改写历史。
修复提交 `f60efc4a2c40c10490094b837db2cd6a253ed375` 以原主分支 `e812b21` 为父提交，
按普通快进发布。其完整验证结果：

- [轨迹认证专项](https://github.com/CJ0817/six-axis-robot-internship/actions/runs/38723159909)：成功。
- [PID 专项](https://github.com/CJ0817/six-axis-robot-internship/actions/runs/38723160173)：成功。
- [环境验证与 Robotics Toolbox 基准](https://github.com/CJ0817/six-axis-robot-internship/actions/runs/38723159838)：成功。

`results/trajectory-certification-cloud/` 保存四项云端报告、运行与提交绑定、原始证据包及 SHA-256。
证据包包含成功／失败 Result、原始 CSV、曲线和场景记录。早期本地报告中的“待云端验证”
是上传前的历史状态；当前验收以该目录的云端报告与上述运行结果为准。
