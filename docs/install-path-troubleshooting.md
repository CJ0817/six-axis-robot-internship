# 安装与路径问题修复

两个已复现的问题：

1. 从仓库外运行安装脚本时，`ROBOT_PYTHON=./某目录/bin/python` 原来在脚本切换工作目录后解析，导致 No such file or directory。现在共用预检查在切换目录前转为绝对路径，保留虚拟环境符号链接，支持含空格路径，并给出缺少 Python/编译器的错误说明。
2. 用虚拟环境 Python 直接运行环境检查、但未执行 activate 时，原检查依赖 PATH 中的 CMake/Ninja，可能缺失或误用系统版本。现在明确使用当前解释器同目录下的工具，仍校验锁定版本，不回退到未知系统版本。

从任意目录调用（将路径替换为实际值，含空格时保留引号）：

```bash
ROBOT_PYTHON="/path/to/python3.11" bash "/path/to/repo/scripts/setup.sh" --check
ROBOT_PYTHON="/path/to/python3.11" bash "/path/to/repo/scripts/setup.sh"
ROBOT_PYTHON="/path/to/python3.11" bash "/path/to/repo/scripts/setup_baseline.sh" --check
"/path/to/repo/.venv/bin/python" "/path/to/repo/scripts/verify_environment.py" --versions-only
```

`--check` 仅检查 Python 3.11.9、可执行路径及 GCC/G++ 命令存在，不代表所有依赖已安装。完整 setup 才安装锁定包、检查版本、编译、运行 CTest 和场景。此修复不迁移已创建的虚拟环境或旧 CMakeCache；移动仓库后应在新路径重新创建环境及构建目录，不能复制旧缓存使用。

验证环境为云端 Linux x86_64，非用户 Windows 桌面。本次两张 PNG 是 PyBullet DIRECT 实际渲染画面，不是 Windows GUI 截图。此前本地 GUI 证据保持独立。

验证证据见 `results/path-validation/`，完整安装命令及结果见 `reports/week01/install-path-validation.md`。
