# 最小 C 动态库：编译与 Python 调用

本步骤复用已有 `src/common/abi_probe.c` 和 `include/robot_contract.h`，新增独立入口 `examples/c_library_demo.py`。它仅依赖 Python 标准库 ctypes，不启动 PyBullet，也不需要 NumPy。目标平台为已配置的 Linux/WSL2；生成 `.so`，不把它作为 Windows 原生 `.dll` 使用。

## 接口

```c
int robot_copy_joints(const double *input, size_t count, double *output);
```

input/output分别指向调用方拥有的6个连续double，按J1…J6排列，角度单位rad；count必须为6。函数不分配、不释放调用方内存，也不保存指针。输入输出使用不同缓冲区；指针必须指向足够大小的有效内存，C接口不能检测任意伪造地址或缓冲区真实长度。

成功返回0并复制6个有限数；非法长度、NaN/Inf或空指针返回1001。失败时可写输出缓冲区保持原值，不能将旧输出作为成功结果使用。此函数是ABI连通性探针，尚不是运动学算法。

Python调用显式设置argtypes为两个double指针和c_size_t，restype为c_int，使用ctypes.CDLL载入绝对路径。示例自动检查一次成功调用及5种错误调用；库缺失/符号缺失/校验失败均非零退出。

## CMake 编译与启动（项目根目录）

先完成既有环境配置，再执行：

```bash
source .venv/bin/activate
cmake -S . -B build/c-demo -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=gcc-13 \
  -DCMAKE_CXX_COMPILER=g++-13 \
  -DPython3_EXECUTABLE="$(command -v python)"
cmake --build build/c-demo
ctest --test-dir build/c-demo --output-on-failure
python examples/c_library_demo.py \
  --library build/c-demo/librobot_contract.so \
  --report results/c-library/cmake-python-report.json
```

产物为 `build/c-demo/librobot_contract.so`。CTest包含cxx_abi_smoke和python_abi_smoke，后者通过目标文件路径载入刚刚构建的动态库。上述目录独立于原环境检查使用的build输出。

## 直接用 GCC 编译（最小路径）

无需CMake或PyBullet，也可验证C/Python通路：

```bash
mkdir -p build/c-demo
gcc-13 -std=c11 -O2 -Wall -Wextra -Werror -pedantic \
  -fPIC -shared -Iinclude src/common/abi_probe.c -lm \
  -o build/c-demo/librobot_contract.so
python3 examples/c_library_demo.py \
  --library build/c-demo/librobot_contract.so \
  --report results/c-library/gcc-python-report.json
```

两种构建方式任选一种；正式环境使用锁定的Python3.11.9。无需改LD_LIBRARY_PATH，示例将库路径解析为绝对路径。Python与动态库必须使用匹配的操作系统和架构。

成功输出JSON：status=passed；six_joint_roundtrip的code=0、输出等于输入[0.1,-0.2,0.3,-0.4,0.5,-0.6]；其余5个错误用例code=1001。进程退出码0。

验证记录见 [本步骤报告](../reports/week01/c-library-validation.md)。现有setup.sh的CTest也会执行新增Python测试；CI另存独立JSON报告。不要把本步骤云端运行当作新一轮用户Windows本机操作。
