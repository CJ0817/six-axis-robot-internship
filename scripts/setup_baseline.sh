#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
ROBOT_PYTHON="${ROBOT_PYTHON:-python3.11}"
"$ROBOT_PYTHON" -c 'import platform; assert platform.python_version()=="3.11.9", "Need Python 3.11.9"'
command -v gcc-13 >/dev/null
command -v g++-13 >/dev/null
"$ROBOT_PYTHON" -m venv .venv-baseline
export CC=gcc-13 CXX=g++-13
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 MPLBACKEND=Agg
.venv-baseline/bin/python -m pip install --require-hashes --only-binary=:all: -r requirements/baseline-build.lock
.venv-baseline/bin/python -m pip install --require-hashes --no-build-isolation -r requirements/baseline.lock
.venv-baseline/bin/python -m pip check
.venv-baseline/bin/python scripts/run_tests.py --suite baseline --output results/test-runs/baseline
