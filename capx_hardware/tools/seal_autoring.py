"""Developer source/assets manifest builder; no site/device operations."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1];package=root/'capx_autoring'
files={str(p.relative_to(package)):hashlib.sha256(p.read_bytes()).hexdigest()for p in sorted(package.rglob('*'))if p.is_file()and '__pycache__'not in p.parts and p.name!='engine-manifest.json'}
(package/'engine-manifest.json').write_text(json.dumps({'schema':'capx-autoring-engine-source/v1','files':files},indent=2)+'\n')
proof=root/'evidence/autoring-upstream-port.json';d=json.loads(proof.read_text())
for r in d['files']:r['packaged_sha256']=hashlib.sha256((root/r['packaged']).read_bytes()).hexdigest()
proof.write_text(json.dumps(d,indent=2)+'\n')
print('Sealed',len(files),'engine/source/asset files')
