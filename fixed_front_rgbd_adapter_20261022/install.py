#!/usr/bin/env python3
"""Derive a NEW camera-only overlay release from immutable608; stdlib only."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil
BASE_MAIN_SHA256='717b9aa951440d2fabcccd817bdf51bfc8a8084377a76337f5f43b3376aca690'
BASE_SUMS_SHA256='5155ab56d714e69f034c3d433ffdcaa9fecc34858fe1b30f8839f5eca47111be'
BASE_MANIFEST_SHA256='6eb077d5b373372194b7e74ec760d21a1e8e14bc9403a1b1f1867c7eb8e39076'
IMMUTABLE_ORIGINAL=Path('/home/gpuadmin/workspace/jake/mujoco-field-publish-20261022/field_demo_random_20261022')
IGNORED={'__pycache__','.venv','.git','.pytest_cache'}
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def contained(root,name):
    relative=Path(name)
    if relative.is_absolute()or '..'in relative.parts:raise ValueError('Unsafe manifest path: '+name)
    p=(root/relative).resolve()
    if not p.is_relative_to(root):raise ValueError('Escaping manifest path: '+name)
    return p

def install(base,output):
    bundle=Path(__file__).resolve().parent;base=Path(base).resolve();output=Path(output).resolve()
    if output.exists()or output==IMMUTABLE_ORIGINAL.resolve()or output==base or output.is_relative_to(base):raise ValueError('Output must be a new directory outside immutable base')
    if digest(base/'src/field_demo/__main__.py')!=BASE_MAIN_SHA256:raise ValueError('Unexpected base __main__.py; use original608 release')
    if digest(base/'SHA256SUMS')!=BASE_SUMS_SHA256 or digest(base/'manifest.json')!=BASE_MANIFEST_SHA256:raise ValueError('Unexpected base608 provenance files')
    original_sums=(base/'SHA256SUMS').read_text();rows=original_sums.splitlines()
    if len(rows)!=608:raise ValueError('Expected original608 source files')
    for line in rows:
        expected,name=line.split('  ',1);p=contained(base,name)
        if not p.is_file()or digest(p)!=expected:raise ValueError('Base source differs: '+name)
    manifest_path=bundle/'overlay-manifest.json';overlay=json.loads(manifest_path.read_text())
    for row in overlay['files']:
        p=contained(bundle,row['path'])
        if not p.is_file()or digest(p)!=row['sha256']:raise ValueError('Overlay source differs: '+row['path'])
        if row.get('target')is not None:contained(output,row['target'])
    # New output only; published source is never opened for writing.
    shutil.copytree(base,output,ignore=shutil.ignore_patterns(*IGNORED))
    for row in overlay['files']:
        if row.get('target')is None:continue
        target=contained(output,row['target']);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(contained(bundle,row['path']),target)
    main=output/'src/field_demo/__main__.py';text=main.read_text();old='MODULES = {';new="MODULES = {\n    'fixed-native-vision':'field_demo.fixed_native_vision',"
    if text.count(old)!=1 or "'fixed-native-vision'"in text:raise ValueError('Unexpected CLI command registry')
    main.write_text(text.replace(old,new,1))
    integration=output/'integration';integration.mkdir(exist_ok=True);(integration/'base608-SHA256SUMS.txt').write_text(original_sums);shutil.copy2(base/'manifest.json',integration/'base608-manifest.json')
    provenance=dict(base_release='immutable608',base_main_sha256=BASE_MAIN_SHA256,base_SHA256SUMS_sha256=BASE_SUMS_SHA256,base_manifest_sha256=BASE_MANIFEST_SHA256,base_files_checked=608,overlay_manifest_sha256=digest(manifest_path),installed_at_utc=datetime.now(timezone.utc).isoformat(),camera_only=True,robot_commands_added=False,hardware_tested=False)
    (integration/'fixed-native-vision-provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    files={p.relative_to(output).as_posix():digest(p)for p in sorted(output.rglob('*'))if p.is_file()and p.relative_to(output).as_posix()not in ['SHA256SUMS','manifest.json']and not(set(p.relative_to(output).parts)&IGNORED)}
    derived=json.loads((base/'manifest.json').read_text());derived.update(name='field-demo-fixed-native-camera-from608',created_utc=provenance['installed_at_utc'],base608_provenance=provenance,camera_only_command='fixed-native-vision',files=files,hardware_tested=False)
    (output/'manifest.json').write_text(json.dumps(derived,indent=2)+'\n');(output/'SHA256SUMS').write_text(''.join(h+'  '+n+'\n'for n,h in files.items()))
    return dict(state='new_camera_overlay_release_created',output=str(output),source_files=len(files),base_files_checked=608,base_unchanged=True,robot_command_sent=False,hardware_tested=False)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(json.dumps(install(a.base,a.output),indent=2))
if __name__=='__main__':main()
