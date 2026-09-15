# 最小 C 动态库与 Python 调用验收

本轮复用既有C ABI探针，新增独立Python入口与CTest用例；没有新增FK/IK算法。工作基线为仓库提交cbdd0a3ee20750c9ae8eed8ff04de45c302d9dd5，保留本地执行者已提交的全部Windows/WSL证据。

## 本轮实际运行

环境：云端Linux x86_64、GCC/G++13.3.0、CMake3.31.6、Ninja分发包1.11.1.3、Python3.12.14（此处只验证标准库ctypes兼容性，不变更工程Python3.11.9版本锁）。正式3.11.9组合由现有CI执行新增CTest和独立入口，结果以对应Actions运行为准。本轮未操作用户Windows电脑。

| 验收项 | 结果 |
| --- | --- |
| GCC直接编译 | 成功生成build/librobot_contract.so |
| CMake/Ninja构建 | 成功生成build/c-demo/librobot_contract.so及C++测试程序 |
| CTest | cxx_abi_smoke与python_abi_smoke均通过，2/2 |
| Python→C→Python | 六个不同double返回一致，code=0 |
| 长度错误、NaN、Inf、空输入指针 | code=1001，输出缓冲区保持原值 |
| 空输出指针 | code=1001，无崩溃 |
| 动态库文件缺失 | 清楚提示先编译，退出码1 |

机器可读证据：[GCC路径](../../results/c-library/gcc-python-report.json)、[CMake路径](../../results/c-library/cmake-python-report.json)。报告记录实际解释器、平台、源码/头文件/示例/动态库SHA-256。动态库位于build中，不提交平台相关二进制。

## 本轮命令记录

以下命令均在本步骤工作目录运行；.build-tools仅隔离安装CMake/Ninja，不替换工程正式虚拟环境：

```bash
uv venv .build-tools
uv pip install --python .build-tools/bin/python cmake==3.31.6 ninja==1.11.1.3
mkdir -p build
gcc-13 -std=c11 -O2 -Wall -Wextra -Werror -pedantic -fPIC -shared \
  -Iinclude src/common/abi_probe.c -lm -o build/librobot_contract.so
python3 examples/c_library_demo.py --report results/c-library/gcc-python-report.json
export PATH="$PWD/.build-tools/bin:$PATH"
cmake -S . -B build/c-demo -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13 \
  -DPython3_EXECUTABLE="$(command -v python3)"
cmake --build build/c-demo
ctest --test-dir build/c-demo --output-on-failure
python3 examples/c_library_demo.py --library build/c-demo/librobot_contract.so \
  --report results/c-library/cmake-python-report.json
```

正式配置下的可复制命令、接口类型与预期输出见[编译与启动说明](../../docs/c-library-quickstart.md)。
