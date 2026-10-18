#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$ROOT/keti_m80_mujoco_20261013"
if ! command -v python3.13 >/dev/null && ! command -v uv >/dev/null; then
    echo 'Install Python 3.13 (with venv) or uv, then rerun this script.' >&2
    exit 1
fi
if [ -d "$PROJECT/.venv" ] && [ ! -d "$PROJECT/.venv-source-snapshot" ]; then
    mv "$PROJECT/.venv" "$PROJECT/.venv-source-snapshot"
fi
if command -v python3.13 >/dev/null; then
    python3.13 -m venv "$PROJECT/.venv"
elif command -v uv >/dev/null; then
    uv venv --python 3.13 "$PROJECT/.venv"
else
    echo 'Install Python 3.13 (with venv) or uv, then rerun this script.' >&2
    exit 1
fi
if "$PROJECT/.venv/bin/python" -m pip --version >/dev/null 2>&1; then
    "$PROJECT/.venv/bin/python" -m pip install --upgrade pip
    "$PROJECT/.venv/bin/python" -m pip install -r "$ROOT/docs/requirements-main-snapshot.txt"
elif command -v uv >/dev/null; then
    uv pip install --python "$PROJECT/.venv/bin/python" -r "$ROOT/docs/requirements-main-snapshot.txt"
else
    echo 'Install pip or uv into the new Python environment, then rerun.' >&2
    exit 1
fi
"$PROJECT/.venv/bin/python" - <<'PY'
import mujoco,numpy,scipy,fcl
print('mujoco',mujoco.__version__,'numpy',numpy.__version__,'scipy',scipy.__version__)
PY
