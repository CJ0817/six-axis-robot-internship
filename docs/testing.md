# 测试入口与 Robotics Toolbox 基准

从仓库根目录执行。当前支持已验证的 Linux x86_64 环境；Windows 可在已配置的 WSL 中运行。此次入口为无界面测试，桌面操作证据继续使用已有本地验证记录。

## 安装与运行

基础环境沿用 `docs/environment-setup.md`：Python 3.11.9、GCC/G++ 13，执行 `bash scripts/setup.sh` 创建 `.venv` 并编译 C 动态库。

```bash
# 单独创建基准环境；不改动 PyBullet 使用的 .venv
ROBOT_PYTHON=python3.11 bash scripts/setup_baseline.sh

# 已实现的 C ABI 和无界面场景检查
.venv/bin/python scripts/run_tests.py --suite smoke --output results/test-runs/smoke
# Robotics Toolbox 依赖、UR5 FK/Jacobian/IK 可运行性
.venv-baseline/bin/python scripts/run_tests.py --suite baseline --output results/test-runs/baseline
# 所有已实现检查
.venv/bin/python scripts/run_tests.py --suite available --output results/test-runs/available
# 完整验收入口：尚未实现的指标会明确 blocked
.venv/bin/python scripts/run_tests.py --suite acceptance --output results/test-runs/acceptance
```

可用 `--smoke-python`、`--baseline-python`、`--library` 指定路径。虚拟环境解释器路径保留符号链接，避免误用基础 Python。安装需联网；脚本先校验 Python 版本与编译器，再安装带 SHA256 的构建依赖，使用 `--no-build-isolation` 编译 Robotics Toolbox，最后执行 `pip check` 和基准检查。

退出码：0 表示所选检查通过；1 表示失败（含依赖缺失、超时、无有效报告）；2 表示完整验收仍有待实现项。每个子进程限时 120 秒，日志与子报告、汇总 `summary.json` 写入输出目录。旧的子报告会先移除，避免启动失败时误读旧成功结果。`available` 通过不代表所有实习算法验收通过。

## 指标维护

[指标表](test-metrics.md) 由 `tests/metrics.json` 生成，包含测试条件、单位、阈值、确认节点、实现状态和入口映射。修改 JSON 后执行：

```bash
python3 scripts/render_metrics.py
python3 -m unittest discover -s tests -p 'test_runner.py' -v
```

未来 C 运动学、规划、控制接口尚未实现的项目保留 pending；实现后需添加实际检查并更新映射。已实现的环境检查和 Robotics Toolbox 自检不能替代这些算法指标。

## 基准版本与数据

直接依赖为 `requirements/baseline.in`；完整版本和包哈希为 `requirements/baseline.lock`；构建引导依赖为 `requirements/baseline-build.lock`。主要选型：Robotics Toolbox 1.1.1、NumPy 1.26.4、SciPy 1.11.4、SpatialMath 1.1.10。完整依赖安装于独立 `.venv-baseline`，避免改变现有仿真版本。

维护者主动更新依赖时使用 `uv pip compile requirements/baseline.in --python-version 3.11 --generate-hashes -o requirements/baseline.lock`，再运行 `python3 scripts/extract_baseline_build_lock.py`；随后在全新虚拟环境执行安装及 CI，审核全部传递依赖变化后提交。普通使用者直接安装已提交的锁文件，无须重新解析。

参考实现采用 `rtb.models.DH.UR5()`，固定随机种子 20260915、100 组弧度关节向量，输出 FK 4×4、基坐标系雅可比 6×6 和 DH/base/tool 元数据。另做一组已知目标位姿的 IK 回代。耗时仅记录，不用于宣称 C 性能达标。该样本集用于依赖准备，不代表任务空间覆盖或关节限位验收。

2026-09-17 已完成 BASE-02 名义模型对齐；因内置 UR5 存在 d1 差异，项目比较须使用显式项目参数参考模型，不能直接使用旧内置 UR5 样本。保存的初次运行证据见 `results/baseline-readiness/` 和[验证记录](../reports/week01/test-baseline-validation.md)。

参考：[官方源码](https://github.com/petercorke/robotics-toolbox-python)、[1.1.1 发布页](https://pypi.org/project/roboticstoolbox-python/1.1.1/)。

统计与报告字段统一遵循 [统计规则 v1](statistics-rules.md)，包括误差有效样本、失败计数、分组成功率、分位数及分层计时。

2026-09-17 更新：BASE-02 名义模型对齐入口已接入 `--suite model`，也随 available/acceptance 运行；详见[UR5核对说明](ur5-model-conventions.md)。此入口不替代 C 算法验收。

C正运动学功能验收：先编译，再运行 `scripts/run_tests.py --suite fk`；使用基准解释器对照C/URDF/显式RTB，并执行独立C矩阵及已知位形测试。也随available/acceptance运行。见[正运动学说明](forward-kinematics.md)。

已知角度FK专项对比：`--suite fk_known`，也随available/acceptance运行；原内置RTB与显式项目模型分别统计，见[对比报告](../reports/week01/fk-rtb-known-poses.md)。

逆解目标准备：`--suite ik_prepare` 校验冻结的100普通+20宽初值+20近奇异目标和3个锚点，也随available/acceptance运行。它不执行IK、不关闭KIN-03，见[阶段接口整理](ik-interface-preparation.md)。
