# 用户 Windows 本机验证（2026-09-15）

状态：环境安装与自动验收通过；真实桌面 GUI 验收阻塞，整体验收部分通过。

实际执行位置为用户 Windows 11 电脑的 Ubuntu-24.04 / WSL2。系统版本见 windows-wsl.txt 和 ubuntu-release.txt；测试提交为 98e2ffe9247ef696131501b379de5e7c7e047c1a。沿用已有工程及 Python 安装，没有重新安装发行版。

## 已通过

ROBOT_PYTHON=/opt/python-3.11.9/bin/python3.11 bash scripts/setup.sh 完整执行成功（退出码 0）。setup.log 保存本机依赖安装检查、CMake/Ninja、CTest 1/1、C/Python ABI、UR5 模型和仿真结果。environment-check.json 中所有版本与锁文件一致。

headless/ 保存本机 DIRECT 场景演示（非云端）：物体中心 [0.45,0.25,0.12]m，位置误差 0m；正面、侧面及切回正面均满足物体和机器人可见像素阈值。两视角 RGB 平均绝对差 13.156967230902778。PNG 属于 TinyRenderer 导出，不能作为 Windows 桌面交互证据。

## GUI 阻塞

实际启动 python examples/scene_demo.py --gui --output results/local-validation/gui。Windows 枚举到 msrdc.exe 承载的 Bullet Physics ExampleBrowser 窗口（标题含 WARN:COPY MODE），但窗口无法正常展示/激活。控制通道一次返回 failed to activate captured window；其余捕获到被遮挡的聊天或启动终端画面。用户在会话中明确反馈“看不见”。重选窗口、从可见 Windows 进程启动、重启 Ubuntu 会话后仍未恢复。尚不能判定底层根因。

未向未确认可见的窗口发送 2/1/J/Q；没有完成桌面视角切换或点动确认。为保留报告，结束本次 GUI 进程时发送 SIGINT，程序 finally 保存 status=failed、gui_keyboard_verified=false。这不是 Q 键正常退出。gui/report.json 保留原始内容；gui-attempt-1/ 和 gui-attempt-2/ 保留此前失败尝试。

GUI 初始渲染报告表明物体创建成功、位置误差 0m；仅生成初始正面 PNG，不能据此认定真实窗口可见或两个桌面视角通过。没有有效的 Windows 桌面截图/录屏，未将聊天/终端截图归档，避免包含无关信息并防止误认成 GUI 证据。

## 证据与隐私复核

原始云端 results/environment-check.json 和 results/scene-demo/report.json 保持仓库版本不变。本机证据单独归档于本目录。prior/ 为上一轮本机安装记录。日志仅含安装版本、工程路径、编译和仿真输出；系统记录仅含与验收相关版本字段。未归档账户凭据、浏览记录或其他应用画面。文件完整性见 SHA256SUMS.json。

## 后续关闭条件

恢复可见的 WSLg PyBullet 窗口后，实际按 2、1、J、Q；保存 GUI 成功 report.json、PNG 和经隐私检查的真实 Windows 窗口截图/录屏，再更新本机 GUI 状态。当前没有以云端或本机 DIRECT 结果替代此项。
