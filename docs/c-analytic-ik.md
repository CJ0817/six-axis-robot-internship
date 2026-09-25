# 标准 C 解析逆解实现与验证

2026-09-25。实现文件 `src/kinematics/inverse.c`；入口 `robot_inverse_v1`，ABI 1.0.0 的签名、参数顺序、结构布局和错误码编号保持不变。普通 UR5 CB 解析逆解已实现；DLS未实现，精确奇异连续族的通用最近解搜索未实现。

## 算法及支持范围

沿用[解析推导](ur5-analytic-ik.md)的肩×腕×肘枚举。运算只使用 C11 数学库 sin/cos/atan2/asin/acos/hypot/remainder 等；链接 `-lm`。不依赖 NumPy、RTB 或第三方 C 求解库。Linux ABI 平台上的预算计时使用 POSIX CLOCK_MONOTONIC，这部分不是纯 ISO C 时钟。

先验证模型、目标、选项和 seed，再剥离 `inv(B) target inv(G) inv(F)`；枚举最多8个离散分支、还原符号/零偏、选取合法2π表示、经已有 C FK逐候选回代、去重排序并选解。保留非零 d5，使用带符号 a2/a3。候选顺序为关节向量字典序，branch_ids 是本次索引；nearest_seed输出一个候选，all输出全部经过检查的普通离散代表。

支持模型为已审定标准 DH UR5 CB：a、d、alpha与标称值逐元素差≤1e-12；固定基座/法兰/工具变换可以非单位，joint_sign可以±1，零偏可以非零。限位和零偏绝对值超过1e6 rad的模型返回1008，避免承诺超出float64可靠相位/端点分辨率的模型。该限制是当前解析实现支持域，不改变FK模型ABI。支持模型结构检查之前仍先执行通用有效性与seed限位校验。

## 不可达、数值边界和失败原子性

| 情形 | 返回/处理 |
|---|---|
| 空必要指针、非法16/6长度、NaN/Inf、非刚体目标或非法选项 | 1001；仅模型为空1004 |
| seed不在收缩后的限位内 | 1005 |
| 不支持的几何/方法/策略 | 1008；合法DLS请求目前也为1008 |
| 末端距DH0大于所有DH平移长度绝对值之和 | 2001；三角不等式提供保守不可达证明 |
| rho<d4-1e-12 m，或全部普通分支违反肘几何域 | 2001 |
| 有几何分支，但所有等价表示都在限位外 | 1005 |
| 预算耗尽，或数值运算/回代无法可靠完成 | 2002；不冒充不可达 |
| 不能可靠完成奇异请求 | 2003 |
| 内部不变量破坏（如非等价候选超过8） | 9000 |

肩域容差1e-12 m，肘余弦域容差1e-12，无量纲腕阈值hypot(u,v)≤1e-12，候选去重1e-9 rad。acos仅在越界≤容差时裁剪；端点关节仅允许≤1e-12 rad的舍入偏差吸附至合法限位，仍须FK回代。角等价的最近代表按未折返角差选择，不能直接把所有输出限制在[-π,π]。腕角用atan2(hypot(u,v),c)，避免近奇异acos损失精度。

2026-09-25过滤任务更新：候选已完成枚举但回代不合格时剔除该候选，保留其余合格解；枚举本身无法完成仍返回2002。all表示所有通过限位/误差约束的离散代表，详见[候选过滤](ik-candidate-filtering.md)。输出在栈上构造，仅成功时一次复制；所有失败保持调用方有效输出缓冲区逐字节不变。C无法验证悬空指针、虚报分配长度；调用方必须遵守ABI内存契约，IK输出不得与输入重叠。

## 精确腕奇异策略

检测到任一肩分支的腕退化时，all返回2003，避免遗漏连续族。nearest_seed仅在seed自身回代误差同时≤1e-12（m/rad）且满足用户阈值时返回seed；它在允许误差下是距离为零的有效代表。否则返回2003，不随意固定q6后声称找到了全局最近解。该策略也会保守拒绝某些存在普通替代分支的目标；通用连续族搜索后续补齐。

近奇异20组（q5绝对值1e-7～1e-5）走普通解析分支并通过。本轮没有将精确奇异处理限制隐藏在这20组成功率中。

## 编译与运行

```bash
mkdir -p build
gcc-13 -std=c11 -O2 -Wall -Wextra -Werror -pedantic -fPIC -shared \
  -Iinclude src/common/abi_probe.c src/common/abi_v1.c \
  src/kinematics/forward.c src/kinematics/inverse.c -lm \
  -o build/librobot_contract.so
python3 scripts/verify_c_ik.py --library build/librobot_contract.so --output results/c-ik
python3 scripts/run_tests.py --suite ik --smoke-python "$(command -v python3)" \
  --output results/test-runs/ik
python3 -m unittest discover -s tests -p test_abi_v1.py -v
```

既有CMake构建已包含inverse.c及c_analytic_ik CTest，CI也接入统一ik测试入口。上述Python验证脚本仅使用标准库，适用于项目3.8+要求。测试过程外层120 s超时；C每调用预算默认0.2 s，由枚举循环和返回前协作检查，不是操作系统抢占式硬实时保证。

## 本轮结果

证据目录 `results/c-ik-verified/`。GCC13.3.0、-O2，Linux x86_64、Python3.12.14云端；模型、冻结目标没有改动。冻结目标先前已与显式项目参数RTB及C FK交叉核对，脚本不读取其见证q作为求解初值。

| 分组 | 通过/应执行 | 最大位置误差(m) | 最大姿态误差(rad) |
|---|---:|---:|---:|
| normal |100/100|7.37e-16|2.05e-15|
| wide_initial |20/20|6.93e-16|1.54e-15|
| near_singular |20/20|7.75e-16|1.27e-15|

每个目标同时核验all和nearest结果一致、所有候选的有限性/限位/FK误差、顺序、去重、填零槽、无效branch_ids、所选索引与距离最优。成功率分母固定100/20/20，140组无失败、无超时；无输出不能记零误差。误差统计取每目标候选的最大误差，报告含有效数、均值、RMSE、p95及最大值。p95按h=(n-1)p线性插值。

另52项专项全部通过：空指针、长度、NaN/Inf、非法旋转/齐次行、非法选项、不支持模型/DLS、不可达（含1e200 m有限值）、全分支限位排除、预算耗尽、精确/阈值附近腕奇异、肩相切、肘伸展/折叠、关节端点、2π选解、非单位固定变换与符号/零偏。特殊预算测试故意设置1e-15 s，按预期返回2002；不算正常140组超时。

C接口耗时含校验和候选验证，从入口CLOCK_MONOTONIC到成功复制输出前；FFI耗时为Python调用前后perf_counter差值。两者分列，未预热、每目标一次all测量，是功能测试随附观测而非正式性能基准。普通组C平均0.021904 ms、p95 0.048565 ms、最大0.158453 ms。外层进程实际等待0.114173 s（含启动、解析、全部普通/nearest及专项测试、报告写入）；不是单次求解耗时。没有据此关闭已有FK FFI性能问题。

CMake Release构建与CTest 4/4通过。ABI布局/符号/错误码回归、Python FK失败契约、C FK矩阵/已知位形、C++ ABI、测试执行器失败传播均通过。额外用 `-fsanitize=undefined -fno-sanitize-recover=all` 构建，同一140组+52专项全部通过，无UBSan中断，摘要见regressions.json。本轮没有执行Windows桌面或实机操作。

KIN-03本轮普通/近奇异目标验收已通过；DLS、精确奇异连续族全局最近解、碰撞检查及实机安全不在本次完成范围。
