#!/usr/bin/env bash
# Source before changing directory: relative ROBOT_PYTHON belongs to caller cwd.
ROBOT_PYTHON="${ROBOT_PYTHON:-python3.11}"
if [[ "$ROBOT_PYTHON" == */* ]]; then
  ROBOT_PYTHON="$(cd -- "$(dirname -- "$ROBOT_PYTHON")" && pwd)/$(basename -- "$ROBOT_PYTHON")"
else
  ROBOT_PYTHON="$(command -v -- "$ROBOT_PYTHON")" || {
    echo 'Python not found. Set ROBOT_PYTHON to Python 3.11.9; see docs/environment-setup.md' >&2
    exit 1
  }
fi
[[ -x "$ROBOT_PYTHON" ]] || { echo "Python is not executable: $ROBOT_PYTHON" >&2; exit 1; }
# Do not use readlink -f: resolving the executable symlink loses venv identity.
"$ROBOT_PYTHON" -c 'import platform,sys; assert platform.python_version()=="3.11.9", "Need Python 3.11.9; set ROBOT_PYTHON explicitly"; print("Python:",sys.executable)'
for robot_tool in gcc-13 g++-13; do
  command -v "$robot_tool" >/dev/null || { echo "Missing $robot_tool; see docs/environment-setup.md" >&2; exit 1; }
done
if [[ "${1:-}" == --check ]]; then
  echo 'Installation prerequisites and Python path: passed'
  exit 0
fi
[[ $# == 0 ]] || { echo 'Usage: bash scripts/setup[_baseline].sh [--check]' >&2; exit 1; }
