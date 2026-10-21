#!/usr/bin/env bash
set -euo pipefail
GPU_PACKAGE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export M80_PROJECT_ROOT="${M80_PROJECT_ROOT:-$(cd -- "$GPU_PACKAGE/../../../.." && pwd)}"
export M80_GPU_PLANNING_HOME="${M80_GPU_PLANNING_HOME:-$M80_PROJECT_ROOT/.local/curobo-gpu2}"
export M80_CUROBO_SOURCE="${M80_CUROBO_SOURCE:-$M80_PROJECT_ROOT/keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/vendor/curobo}"
mkdir -p "$M80_GPU_PLANNING_HOME/site"
python3 -m pip install --target "$M80_GPU_PLANNING_HOME/site" --no-deps -r "$GPU_PACKAGE/requirements-gpu.txt"
export PYTHONPATH="$M80_GPU_PLANNING_HOME/site:$M80_CUROBO_SOURCE${PYTHONPATH:+:$PYTHONPATH}"
python3 "$GPU_PACKAGE/export_model.py"
echo "Dependencies/model ready. Start ik_server.py with this PYTHONPATH and the same M80_* environment."
