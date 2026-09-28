# 冻结120组的 C／Robotics Toolbox 批量验证

2026-09-28。执行 `tests/ik/workspace120.json` 的120个目标，并单列 `workspace120-special.json` 的16个边界与11个异常输入。输入文件、固定种子20260928、UR5 CB模型和冻结C ABI均未更改。完整逐目标、逐候选和错误码证据在 [批量报告](../results/workspace120-batch-run/workspace120_batch/report.json)。

## 测试方法

使用Python3.11.9、NumPy1.26.4、Robotics Toolbox1.1.1及已编译的 `librobot_contract.so`。RTB按项目标准DH显式建模：`d1=0.089159 m`，不是RTB内置UR5的0.089459 m。比较单位m/rad、J1～J6、base→tool0，并包含固定基座、法兰/工具变换。

1. 对每个冻结见证q调用C `robot_forward`，分别和文件目标、独立RTB FK比较。C与RTB结果矩阵最大元素差≤1e-12；C FK返回0，目标误差位置≤1e-5 m、姿态角≤1e-4 rad。
2. 为IK另用固定种子**20260929**给每个见证q逐轴添加[-0.05,+0.05] rad均匀扰动并按模型限位夹取，把计算出的seed明确保存进逐项报告。见证q不作为隐藏的solver初值；测试目标和主组也不重新抽样。用冻结 ABI `analytic_ur5`/`all`、0.2 s、限位余量0及1e-5 m/1e-4 rad调用C逆解。
3. 对每个返回候选分别调用C FK和显式参数RTB FK，保存目标ID、候选索引、q、两套位置/姿态误差及限位判定。候选数必须等于有效记录数；核对排序、重复解、所选解与返回码。普通主组按冻结规则要求每例code=0且至少一个有效候选。
4. 对16边界例均验证C FK；其中零位、腕π、J5上下限4例为精确腕奇异，`all`按既定策略应返回2003，其他12例应返回0并逐候选双回代。边界分母独立于主组。
5. 解码异常清单数值位置的`NaN`/`+Inf`标记；执行11个C FK/IK调用，按清单核对1001、1005或2001，并核对失败时输出缓冲区逐字节不变。异常输入不喂给RTB当作正常位姿。

位置/姿态误差用平移差的L2范数及 `atan2`旋转角。有效结果统计有效数、均值、RMSE、线性插值p95及最大值；失败或无有效输出不填零误差。调用前已经封装指针和输入数组；报告中的`python_to_c_call_ns`只包围一次ctypes调用，`c_function_s`来自C接口返回；该执行没有预热与重复次数，不能当正式耗时性能验收。统一入口另设120 s进程超时并记录实际等待。

## 实测结果

| 组别 | 样本结果 | 返回候选 | 解释 |
|---|---:|---:|---|
|120主组|120/120，code=0|890|每个候选均经C/RTB回代和限位验证|
|边界组|16/16|按返回码保留|4个精确腕奇异返回2003，12个普通边界返回0；FK均返回0|
|异常组|11/11|不读取失败输出|7例1001、2例1005、2例2001，失败输出不变|

24个空间区各有5例，均通过；主组**实际IK成功率100%**，分母固定120。C FK与RTB正解矩阵最大元素差为`2.220446049250313e-16`；主组890候选对目标的C FK最大位置误差`9.586729210015212e-16 m`、最大姿态误差`1.9463071719580273e-15 rad`。报告另含RTB逐候选误差、各区结果、原始错误码、单次耗时、模型/输入/库哈希；这些是当前云端Linux x86_64条件下的数值结果，不代表实机运行。

原输入文件内`expected_rules.ik.execution_status=not_run`记载**生成输入时尚未执行**的历史状态；本轮新报告另记`ik_execution_status=run`及真实成功率，不覆盖冻结输入的元数据。精确奇异返回2003单列，不能把4例加进120主组成功数或谎称全分支已实现。

## 复现

```bash
# 按原仓库安装/编译步骤准备 .venv-baseline 和 build/librobot_contract.so
.venv-baseline/bin/python scripts/run_tests.py --suite workspace120_batch --output results/test-runs/workspace120-batch
# 直接入口亦可传库和输出目录
.venv-baseline/bin/python scripts/compare_workspace120.py --library build/librobot_contract.so --output results/workspace120-batch
```

套件已接入available/acceptance及GitHub Actions基准作业。原始数据准备校验 `--suite workspace120` 仍单独保留。先前FK FFI长尾和精确奇异连续族全局选解不由本次批量通过而关闭；几何解也不代表碰撞可行或可向设备执行。
