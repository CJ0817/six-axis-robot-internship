# 用户 Windows 本机验证（2026-09-15）

最新状态：本机环境及真实 GUI 验收通过。以下阻塞记录为修复前历史；恢复过程见文末及 gui-recovery/RECOVERY.md。

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

## GUI 显示恢复及真实操作通过（2026-09-15）

已在用户 Windows 电脑恢复可见 PyBullet 窗口，并通过 Computer Use 实际按 2→1→J→Q 完成验证。原始 GUI 报告 status=passed、gui_keyboard_verified=true；J1 点动至 0.1rad，最大误差约 1.59e-13rad；物体中心 [0.45,0.25,0.12]m，误差 0m。侧面机器人/物体像素 2049/684，切回正面 2332/1077；视角 RGB 平均绝对差 4.339656032986111。

故障定位：WSLg 系统日志记录 rdp_allocate_shared_memory Failed to open /mnt/shared_memory/... Input/output error，随后 use_gfxredir=0，窗口标题出现 WARN:COPY MODE。wsl --update 确认已是最新；完整执行 wsl --shutdown 后重新启动，use_gfxredir=1 且错误/警告消失，真实机器人窗口立即恢复。先前 wsl --terminate Ubuntu-24.04 只重启发行版，未能恢复整个 WSL 虚拟机的显示共享内存通道。本次未修改机器人源代码、版本锁或显卡配置。

微软同类问题记录：https://github.com/microsoft/WSL/issues/40618 。本机故障归因依据实际日志和修复前后行为，未把他人问题记录当成本机验证证据。

证据：gui-recovery/report.json 为未改写的程序报告；00/01/02 PNG 为 TinyRenderer 导出；windows-side.png、windows-front.png、windows-jog.png 为 Windows Computer Use 捕获的真实 WSLg 窗口（含 Windows 窗口边框），不是模拟桌面图。已视觉复核，仅含机器人、方块及任务相关背景片段，无无关私人信息。操作者为本会话 Codex，通过用户本机 Windows 控制通道完成；用户此前反馈窗口不可见，修复后的窗口可见性由操作者实际观察确认。

report.json 的 user_windows_validation 字段仍保留 not_established_by_this_run：程序自身不判断主机归属；本机通过结论由系统版本记录、实际 Windows 窗口截图和操作记录共同建立。原失败尝试及云端证据保留用于追溯。

本机环境与 GUI 验收现已通过。此结论不包含自研控制器或真实硬件验证。
