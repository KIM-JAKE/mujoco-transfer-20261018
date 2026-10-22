"""Prepare an exact private Isaac simulation snapshot; optionally launch it."""
from pathlib import Path
import argparse,hashlib,json,os,re,shlex,shutil,subprocess
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--base-repo',type=Path,required=True)
p.add_argument('--checkpoint',type=Path,required=True)
p.add_argument('--run',required=True)
p.add_argument('--container',choices=['isaac-jake-gpu-2-capacity-ac370e58','isaac-jake-gpu-3-9aeb439d20'],default='isaac-jake-gpu-3-9aeb439d20')
p.add_argument('--host-mount',type=Path,default=Path('/home/gpuadmin/workspace/jake'))
p.add_argument('--container-mount',default='/workspace/project')
p.add_argument('--launch',action='store_true',help='Launch Isaac Sim only; no libfranka/PLC execution')
a=p.parse_args();base=a.base_repo.resolve();checkpoint=a.checkpoint.resolve();mount=a.host_mount.resolve()
if not re.fullmatch(r'native-roas-table-[A-Za-z0-9_-]+',a.run):p.error('run must be a unique native-roas-table-* name')
for path in [base/'isaac_port/runs/native-roas-table-bin-retimed14/scene-authored.usda',base/'isaac_port/native_hand_layout/roas-table/physx-cpu.kit']:
 if not path.is_file():p.error('Restored server assets required: '+str(path))
manifest=json.loads((checkpoint/'exact-source-manifest.json').read_text())
for name,want in manifest.items():
 rel=Path(name)
 if rel.is_absolute()or'..'in rel.parts:p.error('Invalid archived source path')
 if hashlib.sha256((checkpoint/rel).read_bytes()).hexdigest()!=want:p.error('Archived source changed: '+name)
root=base/'.local/fixed-checkpoint-snapshots'/a.run;out=base/'isaac_port/runs'/a.run
if root.exists()or out.exists():p.error('Use a fresh run name; existing snapshots/results are preserved')
def mirror(src,dst,descend=()):
 dst.mkdir(parents=True,exist_ok=True)
 for x in src.iterdir():
  if x.name in ['.git','.local','__pycache__']or x.name in descend:continue
  y=dst/x.name
  if x.is_file()and x.suffix=='.py':shutil.copy2(x,y)
  else:y.symlink_to(os.path.relpath(x,y.parent),target_is_directory=x.is_dir())
mirror(base,root,['isaac_port']);mirror(base/'isaac_port',root/'isaac_port',['native_hand_layout']);mirror(base/'isaac_port/native_hand_layout',root/'isaac_port/native_hand_layout',['roas-table']);h=root/'isaac_port/native_hand_layout/roas-table';mirror(base/'isaac_port/native_hand_layout/roas-table',h)
def private_parent(folder):
 if folder==root:return
 private_parent(folder.parent)
 if folder.is_symlink():
  src=folder.resolve();folder.unlink();mirror(src,folder)
 else:folder.mkdir(exist_ok=True)
for name in manifest:
 rel=Path(name)
 if rel.parts[0]=='code':target=h/Path(*rel.parts[1:])
 elif rel.parts[0]=='support-code':target=root/Path(*rel.parts[1:])
 else:p.error('Unknown source group '+name)
 private_parent(target.parent)
 if target.is_symlink():target.unlink()
 shutil.copy2(checkpoint/rel,target)
for name,want in manifest.items():
 rel=Path(name);target=(h/Path(*rel.parts[1:]))if rel.parts[0]=='code'else(root/Path(*rel.parts[1:]));assert hashlib.sha256(target.read_bytes()).hexdigest()==want
out.mkdir();cfg=json.loads((checkpoint/'trial-config.json').read_text());(out/'trial-config.json').write_text(json.dumps(cfg,indent=2)+'\n')
(out/'prepared-source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
def cp(path):return a.container_mount.rstrip('/')+'/'+str(path.resolve().relative_to(mount))
# Assets/services use the restored base repository; proof loaders still check
# model, world, source and current grasp before any certificate is reused.
script=out/'run_prepared_sim.py';script.write_text('from pathlib import Path\nimport os,sys,runpy\nH=Path('+repr(cp(h))+')\nsys.path[:0]=[str(H/"bin-picking/realtime-deps-python312"),str(H),str(H.parents[1]),str(H.parents[2])]\nos.environ["M80_ROAS_RUN"]='+repr(a.run)+'\ntry:\n runpy.run_path(str(H/"run_realtime_recipe.py"))\nfinally:\n import omni.kit.app\n omni.kit.app.get_app().post_quit(0)\n')
argv=['nice','-n','10','/isaac-sim/kit/kit',cp(h/'physx-cpu.kit'),'--no-window','--allow-root','--exec',cp(script)]
shell='exec '+shlex.join(argv)+' >'+shlex.quote(cp(out/'kit.log'))+' 2>&1'
cmd=['docker','exec','-d','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','TBB_NUM_THREADS=1',a.container,'/bin/bash','-c',shell]
if a.launch:subprocess.run(['sg','docker','-c',shlex.join(cmd)],check=True)
print(json.dumps(dict(source_files_verified=len(manifest),snapshot=str(root),results=str(out),simulation_launched=a.launch,hardware_commands=False),indent=2))
