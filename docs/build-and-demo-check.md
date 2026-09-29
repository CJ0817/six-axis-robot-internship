# 编译、运行与简短演示复核（2026-09-29）

本记录针对 [环境安装](environment-setup.md)、[最小动态库](c-library-quickstart.md)和[测试入口](testing.md)中的关键命令。测试发生在云端 Linux x86_64，使用锁定的 Python 3.11.9、GCC/G++ 13.3.0、CMake 3.31.6、Ninja 1.11.1.3、PyBullet 3.2.7、NumPy 1.26.4。结构化摘要见 [build-check.json](../results/demo-short/build-check.json)。

## 编译与调用

从仓库根目录，在已安装锁定依赖并激活虚拟环境后：

```bash
source .venv/bin/activate
cmake -S . -B build/release-check -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13 \
  -DPython3_EXECUTABLE="$(command -v python)"
cmake --build build/release-check
ctest --test-dir build/release-check --output-on-failure
python examples/c_library_demo.py --library build/release-check/librobot_contract.so \
  --report results/c-library/release-check.json
```

本轮使用临时虚拟环境和 `/tmp/ur5-cmake-check` 同参数执行，**CTest 5/5 通过**，CMake 产物的 Python ABI 示例通过。最小 GCC 路径 `gcc-13 -std=c11 -O2 -Wall -Wextra -Werror -pedantic -fPIC -shared -Iinclude src/common/abi_probe.c -lm -o build/c-demo/librobot_contract.so` 及相应 Python 示例也通过。后一命令只编译连通性探针；运行 FK/IK 必须使用完整 CMake 构建或[完整四源码 GCC 命令](ik-summary-and-derivation.md#复现)。首次直接调用虚拟环境内的 `cmake` 却未将其 `bin` 目录加入 PATH，CMake 无法找到 Ninja；依照上面的 `source` 步骤激活环境后构建成功。未因此修改构建文件。

Python 测试入口 `scripts/run_tests.py` 的完整 `acceptance` 还含待实现项，不以 5/5 CTest 代替其验收。用户 Windows GUI 与实机未在本轮云端复测。

## 可复现的短视频

本轮直接运行[场景入口](../examples/scene_demo.py)，新的 DIRECT 场景报告与三张原始图像保存在 [frames/](../results/demo-short/frames/)；固定相机依次为正面、J1 点动后侧面、点动后正面。图像由 PyBullet TinyRenderer 生成，脚本检查物体 ID、位置、像素可见性和两视角画面差异，然后编码成 [7.5 秒 MP4](../results/demo-short/ur5-direct-demo.mp4)。[清单](../results/demo-short/manifest.json)包含视频与各帧 SHA-256、准确的 ffmpeg 命令及场景状态；视频画面明确标明 **DIRECT 渲染，不是桌面录屏**。

```bash
source .venv/bin/activate
python scripts/record_short_demo.py --output results/demo-short
ffprobe -v error -show_entries format=duration -show_entries stream=width,height,r_frame_rate \
  -of json results/demo-short/ur5-direct-demo.mp4
```

使用系统 `ffmpeg`（含 libx264）；复现需要锁定的 PyBullet/NumPy。视频为三张**本次运行中生成的关键帧**各持续 2.5 秒，用于展示程序执行结果，不作为连续物理运动、桌面键盘操作或硬件实时性证据。本轮结果：`report.status=passed`，两视角 RGB 平均差 13.15697，物体位置最大误差 0 m，视频 640×480、24 fps、7.5 秒。

## 用户 Windows GUI 录屏（阶段标签之后归档）

用户电脑上的 WSL2/Ubuntu 已运行 `python examples/scene_demo.py --gui --output results/local-demo/gui`，并由 Windows FFmpeg gdigrab 连续采集 **94.3 秒、1920×1200、H.264** 桌面录像。实际按键顺序为 **2、1、J、2、1、Q**；[GUI 报告](../results/local-demo/gui/report.json)记录 `status=passed`、`gui_keyboard_verified=true`，程序退出码为 0。环境检查通过，物体位置误差为 0 m。录像及截图属于真实 Windows GUI 证据，与上节 7.5 秒 DIRECT 关键帧视频分开保存。

[本机证据索引](../results/local-demo/README.md)列出机器和 WSL 信息、键盘事件、截图、录制会话、失败启动日志及复核结论；[SHA-256 清单](../results/local-demo/SHA256SUMS.json)中 `desktop-recording.mp4` 的摘要为 `8621b51796b404eef58f58f76080d1b836b15e0b6490a135f9df51680d00d163`。首次启动遇到 WSLg COPY MODE，完整重启 WSL 后恢复，原始失败证据仍保留。原 GUI 程序中 `user_windows_validation` 字段维持其程序级原值；机器记录与桌面录像共同证明此次用户电脑上的交互运行。

阶段标签 [`v0.2.0-kinematics`](https://github.com/CJ0817/six-axis-robot-internship/tree/v0.2.0-kinematics) 指向较早的运动学阶段提交；Windows 录像在后续提交中归档，应以 `main` 的 [本机证据目录](../results/local-demo/) 查阅。未进行真实机器人硬件或自研控制器验证。
