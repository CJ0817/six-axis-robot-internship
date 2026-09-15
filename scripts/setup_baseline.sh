#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/setup_common.sh"
cd "$(dirname "${BASH_SOURCE[0]}")/.."
"$ROBOT_PYTHON" -m venv .venv-baseline
export CC=gcc-13 CXX=g++-13
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 MPLBACKEND=Agg
.venv-baseline/bin/python -m pip install --require-hashes --only-binary=:all: -r requirements/baseline-build.lock
.venv-baseline/bin/python -m pip install --require-hashes --no-build-isolation -r requirements/baseline.lock
.venv-baseline/bin/python -m pip check
.venv-baseline/bin/python scripts/run_tests.py --suite baseline --output results/test-runs/baseline

