#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
status() {
 python3 - "$ROOT/archives/publish_status.json" "$1" "${2:-}" <<'PY'
import json,os,sys,time
from pathlib import Path
p=Path(sys.argv[1]);tmp=p.with_suffix('.tmp')
tmp.write_text(json.dumps({'state':sys.argv[2],'pid':os.getppid(),
                           'updated_unix_s':time.time(),'detail':sys.argv[3]}));tmp.replace(p)
PY
}
trap 'status failed "inspect logs/publish.log, then rerun ./publish.sh to resume"' ERR
status checking 'verify local archive checksums'
(cd archives && sha256sum -c SHA256SUMS)
status staging 'stage main and SAM3 archive parts through Git LFS'
git add .gitattributes .gitignore README.md restore.sh relocate.py publish.sh build_sam31_checkpoint.sh setup_sam31_env.sh docs/ENVIRONMENTS.md docs/requirements-sam31-active.txt archives/SHA256SUMS archives/sam31_checkpoint.sha256 archives/archive_manifest.json archives/mujoco-snapshot.tar.zst.part-* archives/sam31-checkpoint.zst.part-*
status committing 'commit archive pointers and restore instructions'
if ! git diff --cached --quiet; then
 git commit -m 'Publish complete private MuJoCo migration archive'
fi
status uploading 'push Git commit and all LFS objects'
git push origin main
status verifying 'check remote main commit and LFS file count'
local_sha="$(git rev-parse HEAD)"
remote_sha="$(git ls-remote origin refs/heads/main | awk '{print $1}')"
[ "$local_sha" = "$remote_sha" ]
[ "$(git lfs ls-files | rg 'mujoco-snapshot.tar.zst.part-' | wc -l)" -eq "$(find archives -maxdepth 1 -type f -name 'mujoco-snapshot.tar.zst.part-*' | wc -l)" ]
[ "$(git lfs ls-files | rg 'sam31-checkpoint.zst.part-' | wc -l)" -eq "$(find archives -maxdepth 1 -type f -name 'sam31-checkpoint.zst.part-*' | wc -l)" ]
status complete "remote main $remote_sha; all main and SAM3 parts tracked by LFS"
