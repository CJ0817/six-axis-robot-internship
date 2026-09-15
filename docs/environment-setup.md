# 开发与仿真环境配置

任务书第2页第1阶段要求 Python 3.8+、PyBullet 3.2.x、开源 UR5 URDF、CMake/GCC 及 C 库与 Python 调用通路。本工程针对 Linux x86_64 锁定如下版本；Windows 使用 WSL2 Ubuntu 24.04。Windows 原生和 macOS 未列入已验证平台。

| 组件 | 固定版本 / 来源 |
| --- | --- |
| Python | CPython 3.11.9（满足3.8+） |
| PyBullet | 3.2.7 |
| NumPy | 1.26.4 |
| CMake | 3.31.6（Python wheel） |
| GCC / G++ | 13.3.0，程序名 gcc-13 / g++-13 |
| Ninja | Python分发包1.11.1.3，程序版本1.11.1.git.kitware.jobserver-1 |
| xacro / PyYAML | 2.1.1 / 6.0.2（重新生成模型时使用） |
| pip | 24.3.1 |
| UR5 URDF | ROS-Industrial universal_robot 提交39ad110d8f2e8f66856a201cca88aa7a7025e3eb |

版本与型号的唯一配置来源是 [environment.lock.json](../environment.lock.json)，Python依赖以 [requirements.lock](../requirements.lock) 安装。程序会拒绝与锁文件不一致的环境，不会悄悄改用新版本。此处是应用/工具版本锁，不是完整操作系统二进制镜像锁；Ubuntu基础镜像和编译器发行包修订可能变化，实际环境由验收报告记录。

选择3.11是因为 [PyBullet 3.2.7 的 PyPI 发布文件](https://pypi.org/project/pybullet/3.2.7/) 明确提供 CPython3.11/Linux x86_64 wheel，可避免在3.12环境编译庞大源码。`--only-binary=:all:` 在缺少匹配wheel时直接失败。

## 安装与运行

先在 Ubuntu24.04/WSL2 中准备 Git、gcc-13、g++-13 和精确 Python3.11.9（含venv）。GCC/G++可通过 `sudo apt-get install git gcc-13 g++-13` 安装，版本须通过脚本检查。Python可用已有版本管理器安装3.11.9；GitHub Actions使用setup-python自动安装同版。不要替换操作系统的python3。若 apt 已提供不同编译器版本，必须恢复锁定环境或显式更新锁文件并重跑验收。

```bash
git clone https://github.com/CJ0817/six-axis-robot-internship.git
cd six-axis-robot-internship
ROBOT_PYTHON=python3.11 bash scripts/setup.sh
```

这会建立独立 `.venv`，安装固定依赖、检查版本，用CMake/Ninja编译C11共享库及C++17测试程序，运行CTest，再用Python ctypes调用C库、加载UR5并仿真。成功时退出码0，证据写入 `results/environment-check.json`。失败时保留日志，不能将安装成功等同于全部验收通过。

已有环境可重复验证：

```bash
source .venv/bin/activate
ctest --test-dir build --output-on-failure
python scripts/verify_environment.py --report results/environment-check.json
```

## 模型和验证边界

URDF、7个STL网格、许可与SHA-256清单已随工程保存，加载不依赖ROS和网络。模型为UR5 CB系列通用参数，非UR5e、非某台设备的校准模型。保留上游关节/惯量/限位，视觉使用碰撞STL简化外观，详情见 [模型说明](../models/ur5/NOTICE.md)。J1…J6映射为肩部旋转、肩部抬升、肘部、腕1、腕2、腕3，末端为tool0。

如需重新生成模型，在锁定环境中执行：

```bash
git clone https://github.com/ros-industrial/universal_robot.git ../universal_robot
git -C ../universal_robot checkout --detach 39ad110d8f2e8f66856a201cca88aa7a7025e3eb
python scripts/prepare_ur5.py --source ../universal_robot
```

生成脚本验证上游提交和工作树未修改，解析xacro时只替换包定位语法，生成相对网格路径；SHA-256清单随之更新。对更新后的模型必须重跑环境检查。

验收门槛：版本一致；8个模型文件校验通过；PyBullet中仅有预期6个可动关节和tool0；480步、1/240s重力仿真状态有限；内置位置伺服完成J1=0.1rad点动且最大逐轴误差<0.02rad；TinyRenderer画面含机器人像素；C/C++编译、C/Python六双精度数往返及非法输入返回1001通过。

此处点动使用PyBullet内置伺服，仅验证环境；不是已完成实习要求的自研PID/前馈控制。DIRECT和TinyRenderer检查不代表桌面GUI窗口交互已验证。图形界面视角切换等演示留待后续任务。

## 验证记录

参见 [环境验证记录](../reports/week01/environment-validation.md)。CI见仓库Actions中的Environment verification，推送main后运行；证据上传到environment-evidence附件。当前Linux环境完整运行已通过，详情及局限见验证记录；CI状态单独以Actions页面为准。

PyBullet适配补充：对base_link、base、flange、tool0四个无实体坐标链接显式写入零质量/零惯量，防止导入器自动赋予1kg虚假质量；实际机械连杆的上游惯量保持不变。
