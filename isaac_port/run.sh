#!/usr/bin/env bash
set -euo pipefail
PORT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$PORT_ROOT/../.." && pwd)"
SCENE=exchange
GPU=auto
STEPS=480
VIDEO=1
FRESH=0
while (($#)); do
 case "$1" in
  --scene) SCENE="$2"; shift 2;;
  --gpu) GPU="$2"; shift 2;;
  --steps) STEPS="$2"; shift 2;;
  --no-video) VIDEO=0; shift;;
  --fresh) FRESH=1; shift;;
  -h|--help) echo 'Usage: run.sh [--gpu auto|0] [--steps 480] [--no-video] [--fresh] [--scene exchange|m80]'; exit 0;;
  *) echo "Unknown argument: $1" >&2; exit 2;;
 esac
done
case "$SCENE" in exchange|m80) ;; *) echo 'Scene must be exchange or m80' >&2; exit 2;; esac
[[ "$STEPS" =~ ^[0-9]+$ ]] && ((STEPS>=120)) || { echo 'Steps must be an integer >=120' >&2; exit 2; }
SOURCE_PYTHON="$PORT_ROOT/../keti_m80_mujoco_20261013/.venv/bin/python"
[[ -x "$SOURCE_PYTHON" ]] || { echo 'Run setup_main_env.sh first' >&2; exit 1; }
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-$$"
RUN_DIR="$PORT_ROOT/runs/$RUN_ID"
mkdir -p "$RUN_DIR"
"$SOURCE_PYTHON" "$PORT_ROOT/prepare.py" --scene "$SCENE" | tee "$RUN_DIR/preparation.log"
CONTAINER_PORT="/workspace/project/$(realpath --relative-to="$WORKSPACE_ROOT" "$PORT_ROOT")"
CONTAINER_OUTPUT="$CONTAINER_PORT/runs/$RUN_ID"
OPTIONS=(--scene "$SCENE" --steps "$STEPS" --output "$CONTAINER_OUTPUT")
if ((VIDEO)); then OPTIONS+=(--video); fi
if ((FRESH)); then OPTIONS+=(--fresh); fi
isaac-run --workspace "$(basename "$WORKSPACE_ROOT")" --gpu "$GPU" --detach -- python -u "$CONTAINER_PORT/pipeline.py" "${OPTIONS[@]}"
printf 'Run output: %s\n' "$RUN_DIR"
printf 'Status: %s/pipeline-status.json\n' "$RUN_DIR"
