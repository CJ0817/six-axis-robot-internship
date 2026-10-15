# control

独立 C11 单关节增量式 PID：`pid.c`、公开头文件 `include/robot_control.h`、Python ctypes入口 `pid.py`。

用途、参数、返回值和状态历史见头文件及方法注释；算法、配置、试验范围和复现命令见 [单关节PID](../../docs/single-joint-pid.md)。

本模块无设备I/O、不规划路径。完整六轴控制状态机、前馈、扰动恢复和实时调度仍待后续任务。
