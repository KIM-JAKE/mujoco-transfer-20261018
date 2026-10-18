#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARCH="$REPO/archives"
mkdir -p "$ARCH"
if compgen -G "$ARCH/mujoco-snapshot.tar.zst.part-*" >/dev/null; then
  echo 'Archive parts already exist; refusing overwrite.' >&2
  exit 1
fi
status() {
  python3 - "$ARCH/build_status.json" "$1" "${2:-}" <<'PY'
import json,os,sys,time
from pathlib import Path
p=Path(sys.argv[1]);tmp=p.with_suffix('.tmp')
tmp.write_text(json.dumps({'state':sys.argv[2],'pid':os.getppid(),
                           'updated_unix_s':time.time(),'detail':sys.argv[3]}))
tmp.replace(p)
PY
}
trap 'status failed "archive pipeline failed; partial parts remain for inspection"' ERR
status running 'tar | zstd | split'
tar --sparse -cf - \
  -C /home/jovyan/data-1/jake \
  keti_m80_mujoco_20261013 \
  g2_replenishment_poc_20261016 \
  fr3_cube_mujoco_handoff_20261012 \
  fr3_cube_mujoco_handoff_20261012_v2 \
  fr3_cube_mujoco_handoff_20261012.zip \
  fr3_cube_mujoco_handoff_20261012_v2.zip \
  fr3_cube_mujoco_handoff_20261012_v2.zip.sha256 \
  -C /home/jovyan/data/wjkim robotics_agent \
  | zstd -T4 -3 -q \
  | split -b 512M -d -a 3 - "$ARCH/mujoco-snapshot.tar.zst.part-"
status checking 'checksums and compressed stream'
cd "$ARCH"
sha256sum mujoco-snapshot.tar.zst.part-* > SHA256SUMS
cat mujoco-snapshot.tar.zst.part-* | zstd -tq
python3 - <<'PY'
import json,time
from pathlib import Path
p=Path('mujoco-snapshot.tar.zst.part-000')
parts=sorted(Path('.').glob('mujoco-snapshot.tar.zst.part-*'))
manifest={'created_unix_s':time.time(),'parts':[{'name':x.name,'size_bytes':x.stat().st_size} for x in parts],
          'total_compressed_bytes':sum(x.stat().st_size for x in parts),
          'source_roots':['/home/jovyan/data-1/jake/keti_m80_mujoco_20261013',
                          '/home/jovyan/data-1/jake/g2_replenishment_poc_20261016',
                          '/home/jovyan/data-1/jake/fr3_cube_mujoco_handoff_20261012',
                          '/home/jovyan/data-1/jake/fr3_cube_mujoco_handoff_20261012_v2',
                          '/home/jovyan/data/wjkim/robotics_agent']}
Path('archive_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
PY
status complete 'all archive parts SHA-256 listed; zstd integrity passed'
