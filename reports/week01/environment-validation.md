# 环境验证记录

初次环境配置记录：**当时Linux x86_64环境已通过环境冒烟验证**。本记录证明本工程锁定组合在当前工作环境可用；没有在用户Windows电脑上执行安装，也没有验证桌面GUI交互。

| 检查 | 实测结果 |
| --- | --- |
| Python | 3.11.9 |
| PyBullet / NumPy | 3.2.7 / 1.26.4 |
| CMake | 3.31.6 |
| GCC / G++ | 13.3.0 / 13.3.0 |
| Ninja程序 | 1.11.1.git.kitware.jobserver-1（包1.11.1.3） |
| xacro / PyYAML / pip | 2.1.1 / 6.0.2 / 24.3.1 |
| CMake/Ninja构建 | C11共享库与C++17程序均成功 |
| CTest | 1/1通过 |
| Python ctypes | 六双精度数往返、长度错误、NaN错误均通过 |
| UR5模型 | 固定上游提交；URDF+7个STL的SHA-256全部匹配 |
| 六轴与末端 | J1…J6映射及tool0检查通过 |
| 仿真 | DIRECT、重力-9.81m/s²、dt=1/240s、480步，状态有限 |
| 内置伺服点动 | J1目标0.1rad；六轴最终最大位置差约1.59e-13rad |
| 渲染 | TinyRenderer 128×128，机器人占370像素 |

点动误差只是当前固定目标、PyBullet内置伺服的环境冒烟结果，不能推导为实际机器人精度或自研控制指标。未执行FK/IK/规划/PID算法验收。

机器可读完整结果见 [environment-check.json](../../results/environment-check.json)。同一套安装和验收命令是 `ROBOT_PYTHON=<Python3.11.9可执行文件> bash scripts/setup.sh`。本次首次安装通过uv完成固定依赖下载，随后该setup脚本在相同虚拟环境完成pip一致性检查、CMake构建、CTest与全链路环境验证。

验证中发现并修复：

- Ninja分发包1.11.1.3的实际程序版本带kitware.jobserver后缀，锁文件按实测完整字符串修正。
- 上游URDF的base_link/base/flange/tool0无实体坐标链接省略惯量，PyBullet会默认赋予1kg；生成器显式设置零质量与零惯量并增加检查，实际机械连杆惯量未改。
- 当前宿主Python3.12.14不作为仿真基线，独立安装Python3.11.9并建立项目虚拟环境。

已提供GitHub Actions持续验证流程，后续推送会重跑相同脚本并保留证据。CI状态以Actions页面为准，本次“已通过”依据上述本地实际运行，不冒充CI结果。

## 方案检查者反馈补充：视角、物体与本机证据

原始环境检查仅创建物体，没有对物体ID、位置和可见性作断言，因此不能以原始报告认定“物体添加验收通过”。新增examples/scene_demo.py单独覆盖这些条件，并由scripts/setup.sh及CI统一执行。

- 正面与侧面预设yaw分别40°/130°，pitch=-25°，距离1.8m；自动序列为正面→点动→侧面→正面。
- 物体为显式带visual/collision的橙色静态立方体，中心[0.45,0.25,0.12]m，边长0.16m；必须成功注册到场景。
- 位置最大绝对误差阈值1e-9m；在每个保存画面中物体和机器人各至少20个可见分割像素。
- 两预设RGB平均绝对差必须>0.5；PNG文件SHA-256及执行脚本/模型SHA-256写入JSON。
- GUI入口支持1/2切换、J点动和Q退出。报告必须收到两种切换键事件，才将gui_keyboard_verified设为true。

### 用户Windows电脑验收：历史待办（最新进展见文末）

本轮用户已要求先搁置本地会话连接问题，本会话也没有本机控制通道。因此此项继续标为未完成，不使用云端DIRECT、CI或渲染截图冒充桌面交互证据。

待本机接通后，按docs/environment-setup.md执行安装及--gui入口，保存以下证据才能关闭该验收项：

1. 本机setup.sh成功日志及environment-check.json；记录Windows、WSL发行版、工具链版本和代码提交。
2. 实际在PyBullet窗口按2、1切换，按J点动并按Q退出；归档local-gui-demo/report.json与对应PNG。
3. Windows桌面截图/录屏展示真实GUI、两预设切换、物体画面；由操作者确认运行机器归属。
4. 复核文件哈希、版本一致性和GUI事件记录后，由验收者将本机状态更新为通过。

即使云端演示验收通过，若完整交付目标包含用户电脑，则整体验收仍是“部分通过，本机GUI待验收”。

### 新增验收实测结果（2026-09-15）

执行环境：GitHub Actions Ubuntu24.04，Python3.11.9 / PyBullet3.2.7，DIRECT + TinyRenderer。测试提交aeb20353292028abaa3693dcdac6a355a64b8521。

| 项目 | 实测 | 结论 |
| --- | --- | --- |
| 物体创建 | body_id=1，存在于场景；visual/collision创建成功 | 通过 |
| 物体中心 | [0.45,0.25,0.12]m，创建后/仿真后/每次截图误差均0m | 通过（≤1e-9m） |
| 首次正面画面 | 物体2057像素、机器人6101像素 | 通过（各≥20） |
| 侧面画面 | 物体3237像素、机器人7054像素 | 通过（各≥20） |
| 切回正面 | 物体2057像素、机器人5914像素 | 通过（各≥20） |
| 正面/侧面RGB平均绝对差 | 13.1569672309 | 通过（>0.5） |
| 单关节点动 | J1到0.1rad；六轴最大末态误差1.59e-13rad | 内置伺服冒烟通过 |
| 真实GUI键盘切换 | 本轮未执行，gui_keyboard_verified=false | 待验收 |
| 用户Windows安装与桌面交互 | 无本机证据 | 未完成 |

证据：[通过的CI运行](https://github.com/CJ0817/six-axis-robot-internship/actions/runs/34952737641)；[environment-evidence附件](https://github.com/CJ0817/six-axis-robot-internship/actions/runs/34952737641/artifacts/10389783410)包含00-front.png、01-side.png、02-front.png、原始report.json及环境检查结果。当前附件保留至2026-12-14，不能把临时附件链接当永久归档。

[仓库内JSON记录](../../results/scene-demo/report.json)由该次CI stdout精确提取，字段值未改写；已核对脚本SHA-256匹配测试源码。PNG哈希已包含在JSON中。当前下载通道返回403，未将PNG另存入仓库，且未对PNG进行人工视觉复核；可见性验收依据CI渲染器分割掩码与非零退出码检查。图像仍可通过上述CI附件查看/下载。

总评：云端视角自动切换和物体添加验证通过；GUI入口已实现，但GUI键盘操作及用户Windows本机安装仍未验证。


## 用户电脑本机补充验收（2026-09-15）

**环境安装及本机自动测试通过；GUI 显示连接阻塞，整体验收仍为部分通过。**

实际在用户 Windows 11（10.0.26200.9457）、WSL 2.7.14.0、WSLg 1.0.73.2、Ubuntu 24.04.5 LTS 上执行，测试提交 98e2ffe9247ef696131501b379de5e7c7e047c1a。Python 3.11.9、PyBullet 3.2.7、CMake 3.31.6、GCC/G++ 13.3.0 及全部锁定版本通过；setup.sh 退出 0，CTest 1/1、模型校验、C/Python 接口与本机 DIRECT 场景通过。

真实 GUI 已启动，但 Windows 端窗口不可见/激活失败，重选及重启会话未恢复。用户也反馈看不见。未完成 2/1/J/Q 操作，gui_keyboard_verified=false；无有效桌面截图/录屏，不能标记本机 GUI 通过。以 SIGINT 结束不可见进程并保留原始失败报告。

[本机归档及阻塞详情](../../results/local-validation/README.md) · [环境报告](../../results/local-validation/environment-check.json) · [安装测试日志](../../results/local-validation/setup.log) · [GUI 原始失败报告](../../results/local-validation/gui/report.json)。既有云端证据保留，以上更新不修改历史 CI 结论。
