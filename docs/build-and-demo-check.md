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

## 给“本地执行者”的 Windows 桌面录制任务

请在用户电脑已配置的 WSL2/Ubuntu 与可见的 PyBullet 窗口上完成 **真实 GUI 录屏**，不要使用本页 DIRECT 视频充当本机证据。

1. 在项目根目录激活环境，先运行 `python scripts/verify_environment.py --report results/local-demo/environment-check.json`，保存执行状态；随后运行 `python examples/scene_demo.py --gui --output results/local-demo/gui`。
2. 使用 Windows 截图工具的屏幕录制，开始录制后展示 PyBullet 窗口；点击窗口取得焦点，依次按 **2、1、J、2、1、Q**，使侧面/正面切换和 J1 点动过程确实在录屏中出现。
3. 检查 `results/local-demo/gui/report.json` 的 `status=passed`、`gui_keyboard_verified=true`，保存 GUI PNG 和 Windows 桌面录屏（建议 `results/local-demo/desktop-recording.mp4`）；记录 Windows 与 WSL 版本、录制时间、操作者、视频 SHA-256。
4. 将这些文件提供给本项目执行者复核归档；若窗口无法启动或脚本失败，保留原始错误日志和失败报告，不填写“通过”。

本地录屏是单独的证据节点；只有收到并核对本机文件后才能在周报中写明用户电脑 GUI 已验证。
