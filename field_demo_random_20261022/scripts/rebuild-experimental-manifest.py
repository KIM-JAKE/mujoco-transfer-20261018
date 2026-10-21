#!/usr/bin/env python3
"""Seal source/data bytes of this experimental COPY; never certifies hardware.

Generated local build/SDK/cache directories are excluded: users must build from
source on the supported host. Runtime profiles are regenerated separately and
must pass stabilization check before this manifest is produced.
"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json
R=Path(__file__).resolve().parents[1]
if R.name!='field-demo-random-scenelease-integration-20261022':raise RuntimeError('Explicit experimental copy only; do not reseal upstream bundle')
files={}
for p in sorted(R.rglob('*')):
 rel=p.relative_to(R)
 if not p.is_file() or p.is_symlink() or any(x in ['build','build-state','.local-sdk','__pycache__','.venv','.pytest_cache']or x.startswith('.test-')for x in rel.parts)or rel.as_posix()in ['manifest.json','SHA256SUMS']:continue
 files[rel.as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
manifest=dict(schema_version=4,name=R.name,created_utc=datetime.now(timezone.utc).isoformat(),entry_point='field-demo',guide='docs/MODEL_STARTER.md',selected_profile='stabilization/v128/field-profile.json',experimental_not_field_ready=True,live_random_cycle_enabled=True,residual_stock_occupancy_implemented=True,usable_field_grasp_verified=False,hardware_tested=False,source_build_required=True,files=files)
(R/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(R/'SHA256SUMS').write_text(''.join(h+'  '+name+'\n'for name,h in files.items()));print(json.dumps(dict(state='experimental_manifest_written',files=len(files),hardware_tested=False)))
