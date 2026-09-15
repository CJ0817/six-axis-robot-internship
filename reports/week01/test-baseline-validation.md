# 测试入口与基准依赖验证

日期：2026-09-15。环境：云端 Linux x86_64，Python 3.11.9，GCC/G++ 13.3.0。此次未执行 Windows 桌面操作；既有本地验证记录保持独立。

## 交付

统一入口 `scripts/run_tests.py`，21 项指标的机器可读 JSON 与生成的 Markdown 表，独立 Robotics Toolbox 基准虚拟环境、完整版本/哈希锁、安装脚本及 CI。安装通过 `pip check`，所有锁定包版本与安装值一致。

实际安装命令（ROBOT_PYTHON 指向本环境 Python 3.11.9）：

```bash
ROBOT_PYTHON=/root/.local/share/uv/python/cpython-3.11.9-linux-x86_64-gnu/bin/python3.11 bash scripts/setup_baseline.sh
.venv-baseline/bin/python scripts/run_tests.py --suite baseline --output results/test-runs/baseline
python3 -m unittest discover -s tests -p 'test_runner.py' -v
```

首次运行发现将虚拟环境 Python 符号链接解析到基础解释器会丢失依赖搜索路径；入口已改为保留虚拟环境路径，并成功重跑。

## 实测基准

| 检查 | 结果 |
|---|---|
| 固定版本依赖 | 36 个包全部匹配；pip check 通过 |
| UR5 FK / Jacobian | 各 100/100 组形状、有限性检查通过；FK 齐次行和旋转正交性通过 |
| IK 位置回代 | 5.39494e-15 m，阈值 ≤1e-5 m |
| IK 姿态回代 | 9.42432e-08 rad，阈值 ≤1e-4 rad |
| 参考 FK p95 | 0.123973 ms，仅信息记录 |
| 失败传播回归 | 缺失解释器返回 1，旧成功报告不能复用 |

完整版本、源码和参考数据 SHA256 见 `results/baseline-readiness/report.json`。100 组参考样本见同目录 `ur5-reference-samples.json`。

## 尚未验收

URDF 与 RTB DH 模型坐标对齐为 BASE-02 待办；C 运动学、规划、控制的指标仍待实际实现与测试。参考工具自检通过不等于项目算法通过。控制性能与碰撞规划的待定指标及确认节点在指标表中列明。

## 统一入口验证

完成基础环境安装和编译后，执行：

```bash
python3 scripts/run_tests.py --suite available --output results/test-runs/available
python3 scripts/run_tests.py --suite acceptance --output results/test-runs/acceptance
```

`available`：C ABI、DIRECT 场景、RTB 三项全部通过，退出码 0。`acceptance`：以上三项通过，17 项未来指标 blocked，退出码 2；未将尚未实现的算法标为通过。汇总原始记录保存于 `results/baseline-readiness/available-summary.json` 与 `acceptance-summary.json`；子日志由再次运行或 CI artifact 获取。
