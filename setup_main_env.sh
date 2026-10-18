#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$ROOT/keti_m80_mujoco_20261013"
command -v python3.13 >/dev/null || { echo 'Python 3.13 is required for the main snapshot.' >&2; exit 1; }
if [ -d "$PROJECT/.venv" ] && [ ! -d "$PROJECT/.venv-source-snapshot" ]; then
    mv "$PROJECT/.venv" "$PROJECT/.venv-source-snapshot"
fi
python3.13 -m venv "$PROJECT/.venv"
"$PROJECT/.venv/bin/python" -m pip install --upgrade pip
"$PROJECT/.venv/bin/python" -m pip install -r "$ROOT/docs/requirements-main-snapshot.txt"
"$PROJECT/.venv/bin/python" - <<'PY'
import mujoco,numpy,scipy,fcl
print('mujoco',mujoco.__version__,'numpy',numpy.__version__,'scipy',scipy.__version__)
PY
