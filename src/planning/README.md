# planning

负责路径和时间轨迹及连续约束检查；依赖kinematics/common，不下发设备命令。轨迹算法尚未实现，已建立必需的[限值加载闸门](limits.py)。

规划器与测试共同调用 `load_planning_limits()` 读取唯一配置 `config/planning-limits.v1.json`。加载失败必须返回其错误码并拒绝正式规划，不能采用无限制或另设默认参数。未来规划接口须接收已加载的 `PlanningLimits`，使用其中实际上限；options中的请求缩放在加载时一次应用，后续不得重复缩放。位置限位仍使用绑定模型。

[参数表、TCP定义与缩放](../../docs/planning-limits.md) · [工程总方案](../../docs/engineering-contract.md)。
