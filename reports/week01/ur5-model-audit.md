# UR5 名义模型核对结果

日期：2026-09-17。本轮属于第1周周四模型准备；输出是仿真模型约定与核对证据，不是 C FK、解析 IK 或实机标定验收。

- 型号确认：UR5 CB 通用模型，非 UR5e。
- 标准 DH：参数、逻辑 J1～J6 对应关节、sign=+1 和 theta_offset=0 已写入 `models/ur5/kinematics.json`。
- 坐标：工程 base=URDF base，工程 tool=tool0；base_link 与 base 相差 Rz(π)。DH6 与 tool0 名义一致，flange 另有固定旋转。
- 来源：ROS-Industrial 固定提交、厂商参数页、安装的 RTB 1.1.1 源码均已核对；URDF/来源 YAML/核对脚本 SHA256 见原始报告。

## 检查结果

| 检查 | 实测 |
| --- | --- |
| 固定/逐轴扰动/随机位形 | 3 + 12 + 100 = 115 |
| 名义 DH 对 URDF 成功率 | 115/115 = 100%，失败0，无无效输出 |
| 位置误差最大值 | 2.11727994531e-10 m；门槛1e-9 m |
| 姿态矩阵元素误差最大值 | 5.71722350018e-10；门槛1e-9 |
| 姿态角误差最大值（信息记录） | 6.00605553655e-10 rad |
| 显式项目参数 RTB 与 DH 最大元素差 | 0 |
| 原 RTB 内置 UR5 位置差最大值 | 0.000300000039267 m；发现 d1 差0.0003 m |
| 错误 d1 负向回归 | 故意改为0.089459 m 后115组均失败，检测通过 |

位置/姿态的有效数、均值、RMSE、p95、最大值见 `results/model-audit/report.json`；全部输入与逐样本结果见 samples.json；负向回归输出见 regression.log。115 个都是 FK 模型核对样本，没有 IK 初值分组或求解成功率含义。未测算法性能。

## 重跑与处理

```bash
.venv-baseline/bin/python scripts/verify_model_alignment.py
.venv-baseline/bin/python -m unittest discover -s tests -p test_model_alignment.py -v
.venv-baseline/bin/python scripts/run_tests.py --suite model --output results/test-runs/model
```

本轮实际复用了同一锁定版本的 `../test-baseline-task/.venv-baseline/bin/python`。统一入口通过 `--baseline-python` 指定该路径并成功运行；CI 使用仓库标准 `.venv-baseline`。

模型对齐只对显式项目参数参考模型有效：原 RTB 内置 UR5 及历史参考数据保持原样，不作为项目名义模型真值。URDF 小数精度导致约1e-10量级差异，未放宽原1e-9门槛。

尚待处理：上游不提供加速度上限，规划/控制启动前须确定仿真限值；实机编码器零偏、序列号校准和真实 TCP 仅在实机范围明确后核对。ALG-01 中模型核对部分已完成，C 运动学实现仍待办；安装/路径问题状态不变。
