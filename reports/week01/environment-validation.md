# 环境验证记录

状态：**当前Linux x86_64环境已通过完整验证**。本记录证明本工程锁定组合在当前工作环境可用；没有在用户Windows电脑上执行安装，也没有验证桌面GUI交互。

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
