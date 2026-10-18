#!/usr/bin/env bash
set -euo pipefail
CONTACT_STAMP=$(date -u +%Y%m%dT%H%M%SZ)
CONTACT_OUTPUT="/workspace/project/mujoco/isaac_port/runs/contact-${CONTACT_STAMP}-$$"
exec isaac-run --workspace jake --gpu auto -- /bin/bash -c '
set -euo pipefail
python -c "import mujoco, scipy, ruckig; assert mujoco.__version__ == '\''3.11.0'\'' and scipy.__version__ == '\''1.18.1'\'' and ruckig.__version__ == '\''0.12.2'\''" || python -m pip install mujoco==3.11.0 scipy==1.18.1 ruckig==0.12.2
exec python /workspace/project/mujoco/isaac_port/contact_pipeline.py --output "$1" "${@:2}"
' contact-task "$CONTACT_OUTPUT" "$@"
