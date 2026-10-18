#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT/archives"
sha256sum -c SHA256SUMS
cat mujoco-snapshot.tar.zst.part-* | zstd -dc | tar -xf - -C "$ROOT" --no-same-owner
python3 "$ROOT/relocate.py" --root "$ROOT"
printf 'Restored at %s\n' "$ROOT"
printf 'Read README.md before running simulations; source virtualenv snapshots need rebuilding.\n'
