#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME="$ROOT/robotics_agent/mujoco_duo_fr3/runtime"
BASE_PYTHON="${SAM31_PYTHON_BASE:-python3.11}"
ENV="$RUNTIME/sam31_venv"
if [ ! -x "$ENV/bin/python" ]; then
 "$BASE_PYTHON" -m venv --system-site-packages "$ENV"
fi
if ! "$ENV/bin/python" -c 'import torch, torchvision' >/dev/null 2>&1; then
 echo 'Install a CUDA-compatible PyTorch and torchvision for AIDAS into the selected Python 3.11 base environment, then rerun this script.' >&2
 exit 2
fi
"$ENV/bin/python" -m pip install --upgrade pip setuptools wheel
"$ENV/bin/python" -m pip install -e "$RUNTIME/sam31_source" opencv-python-headless
PYTHONPATH="$RUNTIME/sam31_source" "$ENV/bin/python" - <<'PY'
import torch
from sam3 import model_builder
from sam3.model.sam3_multiplex_detector import Sam3MultiplexDetector
from sam3.model.sam3_image_processor import Sam3Processor
from sam3.model.vl_combiner import SAM3VLBackboneTri
print('SAM3.1 imports OK; PyTorch',torch.__version__,'CUDA available:',torch.cuda.is_available())
PY
