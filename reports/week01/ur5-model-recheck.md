# UR5 模型重新核对记录

日期：2026-09-17。按用户要求重新执行，使用新克隆的仓库，基于提交 `617bceddb10368c903f30f04adab9438ede73781`；执行前工作区干净。复用已锁定的 Python 3.11.9 / RTB 1.1.1 基准环境。本次未更改模型参数或算法。

## 重新确认的约定

| 项目 | 结论 |
| --- | --- |
| 型号 | UR5 CB 系列通用仿真模型，非 UR5e，不代表具体实机校准 |
| DH | 标准 DH：RotZ(theta)·TransZ(d)·TransX(a)·RotX(alpha)；a=[0,-0.425,-0.39225,0,0,0] m；d=[0.089159,0,0,0.10915,0.09465,0.0823] m；alpha=[π/2,0,0,π/2,-π/2,0] rad |
| 关节方向与零偏 | J1～J6 对应 URDF 六个可动关节，sign 全+1，名义 theta_offset 全0；实机编码器偏置未由此确定 |
| 基座 | 工程 base 对应 URDF base；T_base_link_base=Rz(π)，T_base_dh0=I |
| 工具 | 无附加工具时工程 tool=tool0；DH6 与 tool0 名义一致；T_dh6_flange 与 T_flange_tool 互逆且各自非单位旋转 |
| 世界位姿 | 本演示 T_world_base_link=I，因此 T_world_base=Rz(π) |
| RTB 差异 | 内置UR5 d1=0.089459 m，比项目多0.0003 m；使用显式项目参数 DHRobot，不改第三方包、不伪造基座平移 |

## 本次实测

- 3个命名位形、12个逐轴正负扰动、100个固定种子随机位形：115/115通过。
- 最大位置误差：2.1172799453134693e-10 m，阈值1e-9 m。
- 最大旋转矩阵元素差：5.717223500179003e-10，阈值1e-9。
- 故意把d1改成RTB内置值的负向回归：115组均被判失败，回归测试通过。
- 来源核对：将两份source-config YAML与上游提交 `39ad110d8f2e8f66856a201cca88aa7a7025e3eb` 的对应文件逐字节比对，均一致；详情见source-check.json。模型/资产哈希另由核对脚本逐项校验。

## 实际执行入口与证据

从本次新克隆目录执行：

```bash
../test-baseline-task/.venv-baseline/bin/python scripts/run_tests.py --suite model --baseline-python ../test-baseline-task/.venv-baseline/bin/python --output results/model-recheck
../test-baseline-task/.venv-baseline/bin/python -m unittest discover -s tests -p test_model_alignment.py -v
```

标准安装后可以将解释器路径改为仓库 `.venv-baseline/bin/python`。来源逐字节核对使用 `git show <固定提交>:ur_description/config/ur5/<文件名>` 的原始字节，与仓库快照比较。

本次完整证据保存在 [results/model-recheck](../../results/model-recheck/)：summary.json、model_alignment/report.json、samples.json、执行日志、回归日志及source-check.json；原有核对结果保留不覆盖。

结论与[模型约定](../../docs/ur5-model-conventions.md)一致。加速度限值、实机标定及实际附加TCP仍按原问题表处理；本次结果不代表C运动学算法验收或实机验证通过。
