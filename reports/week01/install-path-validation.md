# 安装与路径修复验证

日期：2026-09-15；云端 Linux x86_64。保持 Python 3.11.9、PyBullet 3.2.7、CMake 3.31.6 与 GCC/G++ 13.3.0 锁定版本。

修复前：从仓库外传相对 ROBOT_PYTHON 路径安装，退出码 127；限制 PATH 为 /usr/bin:/bin 后用虚拟环境 Python 检查版本，因找不到 cmake 退出码 1。错误日志分别保存为 before-relative-python.log、before-tool-path.log。

修复后：相同 Python 路径的基础/基准安装预检查均通过；含空格的相对路径回归通过；未激活虚拟环境的版本检查通过。基础完整安装、CMake 构建、CTest 2/2、URDF 校验、C/Python 调用及 PyBullet DIRECT 演示均通过。完整安装从仓库外执行：

```bash
ROBOT_PYTHON=./test-baseline-task/.venv/bin/python bash path-fix-task/scripts/setup.sh
```

上述路径是本次工作目录中的实际路径，复现方式见 docs/install-path-troubleshooting.md。基准安装只复验共用路径预检查，本次未重新编译 Robotics Toolbox 全部依赖。

证据位于 results/path-validation：setup.log 为完整安装日志，regression.log 为路径回归，environment-report.json 和 scene-report.json 为实际检查结果，front.png、side.png 是本次 PyBullet DIRECT 两视角画面。它们不代表用户 Windows 桌面 GUI 操作截图；既有本地证据未改动。

问题闭环、使用限制及后续算法待办见[第1周问题表](issue-register.md)。本轮无未解决安装／路径问题。
