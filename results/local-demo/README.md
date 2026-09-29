# 2026-09-29 用户 Windows 真实桌面录像

已完成本机指定命令 `python examples/scene_demo.py --gui --output results/local-demo/gui`。
实际通过 Windows Computer Use 依次按 **2、1、J、2、1、Q**；GUI 程序正常退出，退出码 0。

- GUI report.json：status=passed，gui_keyboard_verified=true。
- 环境报告：status=passed，全部锁定版本一致。
- 桌面录像：desktop-recording.mp4，94.3 秒，1920×1200，H.264；实际帧率见 video-metadata.json。
- 录像为本机 Windows FFmpeg gdigrab 连续桌面采集，不是渲染图拼接。因截图工具选区浮层无法自动操作，采用自动化桌面录屏替代；没有使用截图工具生成此 MP4。
- 隐私处理：裁去 GUI 出现前 23.7 秒等待画面；Q 退出后的桌面用明确标注的黑色遮罩覆盖。机器人视角切换及点动过程连续，无加速、补帧或合成替代。
- 截图：screenshots/ 为真实 Windows 窗口截图；gui/ 下的 PNG 为程序导出，两者分开保存。
- 机器/WSL 版本及操作者：machine.json、ubuntu-release.txt；代码版本见 tested-commit.txt、upstream-commit.txt。
- 事件时刻：key-events.json；录制命令、时刻和隐私裁切信息：recording-session.json。
- 初次启动出现 WSLg COPY MODE，完整重启 WSL 后恢复；失败启动证据保留为 gui-startup-failed/。
- 原始报告 user_windows_validation 字段保留程序原值；本机归属由机器记录和真实桌面录像补充证明。

核验：键盘切换事件依次 side/front/side/front，期间只有一次 J1 点动（目标 0.1rad，最大误差 1.5876189252139739e-13rad）；物体位置误差 0m。

未验证真实硬件或自研控制器。本次证据另存于工程 results/local-demo/，未覆盖此前云端或 9 月 15 日本机证据。
