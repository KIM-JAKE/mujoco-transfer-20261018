#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT/archives"
sha256sum -c SHA256SUMS
cat mujoco-snapshot.tar.zst.part-* | zstd -dc | tar -xf - -C "$ROOT" --no-same-owner
CHECKPOINT_DIR="$ROOT/robotics_agent/mujoco_duo_fr3/runtime/sam31_checkpoint"
mkdir -p "$CHECKPOINT_DIR"
CHECKPOINT_TMP="$CHECKPOINT_DIR/sam3.1_multiplex.pt.tmp"
trap 'rm -f "$CHECKPOINT_TMP"' EXIT
cat sam31-checkpoint.zst.part-* | zstd -dc > "$CHECKPOINT_TMP"
mv "$CHECKPOINT_TMP" "$CHECKPOINT_DIR/sam3.1_multiplex.pt"
(cd "$CHECKPOINT_DIR" && sha256sum -c "$ROOT/archives/sam31_checkpoint.sha256")
trap - EXIT
python3 "$ROOT/relocate.py" --root "$ROOT"
printf 'Restored at %s\n' "$ROOT"
printf 'Read README.md before running simulations; source virtualenv snapshots need rebuilding.\n'
