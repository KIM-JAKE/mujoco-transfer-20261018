#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARCH="$ROOT/archives"
SRC='/home/jovyan/data/.cache/huggingface/hub/models--facebook--sam3.1/snapshots/daa63191845a41281374e725f4c9e51c7a824460/sam3.1_multiplex.pt'
[ -f "$SRC" ]
if compgen -G "$ARCH/sam31-checkpoint.zst.part-*" >/dev/null; then
 echo 'Checkpoint parts already exist; refusing overwrite.' >&2; exit 1
fi
status() {
 python3 - "$ARCH/sam31_build_status.json" "$1" "${2:-}" <<'PY'
import json,os,sys,time
from pathlib import Path
p=Path(sys.argv[1]);tmp=p.with_suffix('.tmp')
tmp.write_text(json.dumps({'state':sys.argv[2],'pid':os.getppid(),'updated_unix_s':time.time(),'detail':sys.argv[3]}));tmp.replace(p)
PY
}
trap 'status failed "inspect logs/sam31_build.log"' ERR
status compressing 'external SAM3.1 checkpoint'
zstd -T4 -3 -q -c "$(readlink -f "$SRC")" | split -b 512M -d -a 3 - "$ARCH/sam31-checkpoint.zst.part-"
status checking 'verify source and compressed stream'
cd "$ARCH"
sha256sum sam31-checkpoint.zst.part-* >> SHA256SUMS
sha256sum "$SRC" | awk '{print $1 "  sam3.1_multiplex.pt"}' > sam31_checkpoint.sha256
cat sam31-checkpoint.zst.part-* | zstd -tq
python3 - <<'PY'
import json
from pathlib import Path
p=Path('archive_manifest.json');d=json.loads(p.read_text())
parts=sorted(Path('.').glob('sam31-checkpoint.zst.part-*'))
d['sam31_checkpoint']={'source':'/home/jovyan/data/.cache/huggingface/hub/models--facebook--sam3.1/snapshots/daa63191845a41281374e725f4c9e51c7a824460/sam3.1_multiplex.pt',
 'restored':'robotics_agent/mujoco_duo_fr3/runtime/sam31_checkpoint/sam3.1_multiplex.pt',
 'parts':[{'name':q.name,'size_bytes':q.stat().st_size} for q in parts],
 'compressed_bytes':sum(q.stat().st_size for q in parts)}
p.write_text(json.dumps(d,indent=2)+'\n')
PY
status complete 'checkpoint parts and source hash recorded'
