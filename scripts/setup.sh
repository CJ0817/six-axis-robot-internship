#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
ROBOT_PYTHON="${ROBOT_PYTHON:-python3.11}"
"$ROBOT_PYTHON" -c 'import platform; assert platform.python_version() == "3.11.9", "Need Python 3.11.9; see docs/environment-setup.md"'
command -v gcc-13 >/dev/null
command -v g++-13 >/dev/null
"$ROBOT_PYTHON" -m venv .venv
.venv/bin/python -m pip install pip==24.3.1
.venv/bin/python -m pip install --only-binary=:all: -r requirements.lock
.venv/bin/python -m pip check
export PATH="$PWD/.venv/bin:$PATH"
python scripts/verify_environment.py --versions-only
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13
cmake --build build
ctest --test-dir build --output-on-failure
python scripts/verify_environment.py --report results/environment-check.json
