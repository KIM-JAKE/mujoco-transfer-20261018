#!/usr/bin/env bash
set -euo pipefail
PORT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$PORT_ROOT/../.." && pwd)"
GPU="${1:-auto}"
RUN_ID="button-$(date -u +%Y%m%dT%H%M%SZ)-$$"
RUN_DIR="$PORT_ROOT/runs/$RUN_ID"
mkdir -p "$RUN_DIR"
"$PORT_ROOT/../keti_m80_mujoco_20261013/.venv/bin/python" "$PORT_ROOT/export_button_replay.py" | tee "$RUN_DIR/preparation.log"
CONTAINER_PORT="/workspace/project/$(realpath --relative-to="$WORKSPACE_ROOT" "$PORT_ROOT")"
isaac-run --workspace "$(basename "$WORKSPACE_ROOT")" --gpu "$GPU" --detach -- python -u "$CONTAINER_PORT/button_pipeline.py" --output "$CONTAINER_PORT/runs/$RUN_ID"
printf 'Replay output: %s\n' "$RUN_DIR"
