## GUI 显示恢复及真实操作通过（2026-09-15）

已在用户 Windows 电脑恢复可见 PyBullet 窗口，并通过 Computer Use 实际按 2→1→J→Q 完成验证。原始 GUI 报告 status=passed、gui_keyboard_verified=true；J1 点动至 0.1rad，最大误差约 1.59e-13rad；物体中心 [0.45,0.25,0.12]m，误差 0m。侧面机器人/物体像素 2049/684，切回正面 2332/1077；视角 RGB 平均绝对差 4.339656032986111。

故障定位：WSLg 系统日志记录 rdp_allocate_shared_memory Failed to open /mnt/shared_memory/... Input/output error，随后 use_gfxredir=0，窗口标题出现 WARN:COPY MODE。wsl --update 确认已是最新；完整执行 wsl --shutdown 后重新启动，use_gfxredir=1 且错误/警告消失，真实机器人窗口立即恢复。先前 wsl --terminate Ubuntu-24.04 只重启发行版，未能恢复整个 WSL 虚拟机的显示共享内存通道。本次未修改机器人源代码、版本锁或显卡配置。

微软同类问题记录：https://github.com/microsoft/WSL/issues/40618 。本机故障归因依据实际日志和修复前后行为，未把他人问题记录当成本机验证证据。

证据：gui-recovery/report.json 为未改写的程序报告；00/01/02 PNG 为 TinyRenderer 导出；windows-side.png、windows-front.png、windows-jog.png 为 Windows Computer Use 捕获的真实 WSLg 窗口（含 Windows 窗口边框），不是模拟桌面图。已视觉复核，仅含机器人、方块及任务相关背景片段，无无关私人信息。操作者为本会话 Codex，通过用户本机 Windows 控制通道完成；用户此前反馈窗口不可见，修复后的窗口可见性由操作者实际观察确认。

report.json 的 user_windows_validation 字段仍保留 not_established_by_this_run：程序自身不判断主机归属；本机通过结论由系统版本记录、实际 Windows 窗口截图和操作记录共同建立。原失败尝试及云端证据保留用于追溯。

本机环境与 GUI 验收现已通过。此结论不包含自研控制器或真实硬件验证。
