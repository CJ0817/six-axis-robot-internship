# 2026-09-17 阶段收尾：推导、接口与逆解输入准备

## 本轮交付

- 整理标准DH、FK链、移除基座/工具变换及恢复逻辑关节角的关系，链接实际C/Python接口；文档见 `docs/ik-interface-preparation.md`。
- 冻结普通初值100组、宽初值20组、腕近奇异20组，以及分母外3个锚点；数据见 `tests/ik/targets.json`。
- 目标由显式项目参数RTB FK生成，并逐个通过C FK交叉核对；未执行IK、未按求解结果筛选样本。最大矩阵元素差 3.33066907388e-16。
- 新增生成/校验入口及CI；默认只校验，必须显式传--write-fixture才能更新固定输入。

## 重跑证据

本次从新克隆仓库编译C库（GCC13.3.0，C11，O2），复用同锁定版本的基准虚拟环境。运行命令：

```bash
.venv-baseline/bin/python scripts/prepare_ik_targets.py --write-fixture
.venv-baseline/bin/python scripts/run_tests.py --suite available --output results/day-close
.venv-baseline/bin/python -m unittest discover -s tests -p test_ik_preparation.py -v
```

实际基准解释器路径为相邻 `../test-baseline-task/.venv-baseline/bin/python`，通过--baseline-python指定；smoke使用对应.venv路径。日常无需重复--write-fixture。

结果：C ABI、场景、RTB依赖、模型对齐、C FK、已知角对比、IK目标准备7项均通过；入口失败传播2项、Python FK边界1项、冻结目标误改检测1项回归均通过。IK准备的solver_success_rate为null，KIN-03仍pending。

目标SHA256：`be4a50a143d2acf1fbb03f10ce2f964a68d0a8e2515523cb0f306d0a8f4e19bf`。准备报告和负向回归日志见 `results/ik-preparation/`；现有功能汇总见同目录available-summary.json。子报告路径对应完整运行目录，重跑可重建；CI保存完整运行artifact。

## 缓冲排查与处理

开始时检查主分支提交31b142f的GitHub Actions（run 35205126770），状态success。

本轮云端运行时缓存的Python3.11.9目标文件丢失，旧虚拟环境符号链接不可执行；恢复同版本解释器后成功运行全部检查，未改变仓库依赖锁或模型参数。这是本轮云端运行时恢复，不代表用户Windows出现相同故障。问题登记见issue-register.md。

新回归验证：将冻结目标位置改动1 mm后，校验器返回失败且不覆盖该文件，防止将意外变更自动接受为新真值。

本轮无未解决安装／路径问题。后续待办单列：解析IK分支推导、C ABI容量/布局、奇异处理、逆解测试执行器，以及非法/不可达案例。C雅可比、性能和规划控制仍按原计划推进。
