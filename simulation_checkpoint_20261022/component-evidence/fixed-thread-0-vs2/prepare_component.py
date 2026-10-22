"""Prepare, never launch, a literal-source isolated PhysX thread component."""
from pathlib import Path
import ast, hashlib, json, os, shutil, textwrap
ROOT=Path('/home/gpuadmin/workspace/jake/mujoco')
STAGE=Path(__file__).resolve().parent
RUN=ROOT/'isaac_port/runs/native-roas-table-fixed-front-v494-balanced-small-raw-cache-nominal'
BASE=ROOT/'.local/thread-component-494'
SNAP=RUN/'code_snapshot'; ORIGINAL=ROOT/'isaac_port/native_hand_layout/roas-table'
sha=lambda b:hashlib.sha256(b).hexdigest()
plan=json.loads((STAGE/'plan.json').read_text())
for rel,want in plan['source_seals'].items():
    assert sha((RUN/rel).read_bytes())==want,rel
source=(SNAP/'run_realtime_recipe.py').read_text(); tree=ast.parse(source);lines=source.splitlines(keepends=True)
manifest=json.loads((RUN/'code-manifest.json').read_text())
for relative,want in manifest.items():
    p=SNAP/relative
    assert p.is_file() and sha(p.read_bytes())==want,relative
if BASE.exists():raise RuntimeError('Refusing existing component materialization '+str(BASE))
H=BASE/'tree/isaac_port/native_hand_layout/roas-table';H.mkdir(parents=True)
# Only private copies/symlinks. Original Python/control snapshots win; original
# non-Python assets remain read-only symlink inputs. No worker is refreshed.
for src in ORIGINAL.iterdir():
    if src.name=='__pycache__' or src.name.startswith('.'):continue
    dst=H/src.name
    if src.name=='bin-picking':
        dst.mkdir()
        for item in src.iterdir():
            (dst/item.name).symlink_to(os.path.relpath(item,dst),target_is_directory=item.is_dir())
    elif src.suffix=='.py':
        if not (SNAP/src.name).exists():continue
        dst.write_bytes((SNAP/src.name).read_bytes())
    else:dst.symlink_to(os.path.relpath(src,dst.parent),target_is_directory=src.is_dir())
for src in SNAP.rglob('*'):
    if not src.is_file():continue
    dst=H/src.relative_to(SNAP);dst.parent.mkdir(parents=True,exist_ok=True)
    if dst.is_symlink():dst.unlink()
    if not dst.exists() or src.suffix=='.py' or src.relative_to(SNAP).parts[0]=='bin-picking':dst.write_bytes(src.read_bytes())
# Keep source scene/data resolution identical; output is explicitly redirected.
(H.parents[1]/'runs').symlink_to(os.path.relpath(ROOT/'isaac_port/runs',H.parents[1]),target_is_directory=True)
# Import helpers that original HERE.parents[2] might resolve, without canonical edits.
for p in (ROOT/'isaac_port').glob('*.py'):
    (H.parents[1]/p.name).write_bytes(p.read_bytes())
for p in ROOT.glob('*.py'):
    (H.parents[2]/p.name).write_bytes(p.read_bytes())
for source,dest,excluded in [(ROOT,H.parents[2],{'isaac_port','.local','.git','__pycache__'}),(ROOT/'isaac_port',H.parents[1],{'native_hand_layout','runs','__pycache__'})]:
    for item in source.iterdir():
        target=dest/item.name
        if item.name in excluded or target.exists() or target.is_symlink():continue
        target.symlink_to(os.path.relpath(item,target.parent),target_is_directory=item.is_dir())
blocks={};seals={}
def block(name,start,end,dedent=False):
    original=source[source.index(start):source.index(end,source.index(start))]
    value=textwrap.dedent(original) if dedent else original
    blocks[name]=value
    seals[name]={'original_sha256':sha(original.encode()),'extracted_sha256':sha(value.encode()),'start_marker':start,'end_marker':end,'dedent_only':dedent}
    return value
prefix=block('setup_prefix',source[:80],"    camera=BatchRGBDCamera")
# Only thread-setting, pre-attach dispatch call, output redirect differ.
old="OUT=HERE.parents[1]/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)"
new="OUT=Path(os.environ['M80_COMPONENT_OUT']);OUT.mkdir(exist_ok=True)"
assert prefix.count(old)==1;prefix=prefix.replace(old,new)
old="set_int('/persistent/physics/numThreads',0)";new="set_int('/persistent/physics/numThreads',int(os.environ['M80_COMPONENT_THREADS']))"
assert prefix.count(old)==1;prefix=prefix.replace(old,new)
old="sim.attach_stage(ident)";new="omni.physx.get_physx_interface().set_thread_count(int(os.environ['M80_COMPONENT_THREADS']));sim.attach_stage(ident)"
assert prefix.count(old)==1;prefix=prefix.replace(old,new)
prefix += 'except BaseException:\n    raise\n'
compile(prefix,'<literal494-setup>','exec');blocks['setup_prefix']=prefix;seals['setup_prefix']['authorized_patches']=['OUT location only','numThreads requested before attach','set_thread_count before attach'];seals['setup_prefix']['generated_sha256']=sha(prefix.encode())
block('clamps_and_writeback',"    tape=[];actuators=[];peak=0.;physical=0.;active_physics_substeps=1",'    # Raw contact safety telemetry only:',True)
block('contact_callback',"    empty_donor_contact_guard={'left':False",'    def smooth(u):',True)
block('operation_settings',"    if startup_config.get('native_contact_offset_m'):",'    prepared_fixed_routes=',True)
block('native_controller_setup',"        native_grip_hold={'left':False",'        def bind_validation_target(',True)
functions={}
for name in ['scattered_step','update_motor_speed_limits','contacts','loaded_preload','set_native_grip_hold']:
    nodes=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name];assert len(nodes)==1
    node=nodes[0];original=''.join(lines[node.lineno-1:node.end_lineno]);functions[name]=sha(ast.dump(node,include_attributes=False).encode())
    if name=='scattered_step':
        blocks[name]=textwrap.dedent(original);seals[name]={'original_sha256':sha(original.encode()),'extracted_sha256':sha(blocks[name].encode()),'lines':[node.lineno,node.end_lineno],'dedent_only':True}
for name,value in blocks.items():
    compile(value,'<literal494-'+name+'>','exec')
    (BASE/(name+'.py')).write_text(value)
    (STAGE/(name+'.py')).write_text(value)
(ROOT/'.local/thread-component-494/input-source-seals.json').write_text(json.dumps({'original_manifest':manifest,'blocks':seals,'literal_function_AST_sha256':functions,'plan':plan},indent=2)+'\n')
# Copy the source field, not an approximation, and seal all original run inputs.
shutil.copy2(RUN/'executed-actuator-commands.npz',BASE/'source-command-field.npz')
shutil.copy2(RUN/'trial-config.json',BASE/'original-trial-config.json')
shutil.copy2(STAGE/'component.py',BASE/'component.py')
shutil.copy2(STAGE/'evaluation-body-paths.json',BASE/'evaluation-body-paths.json')
# Assets physically consumed by literal setup. Source USD resolves its original
# relative dependencies in its source directory, as in the full driver.
assetpaths=[ROOT/'isaac_port/runs/native-roas-table-bin-retimed14/scene-authored.usda',ROOT/'isaac_port/runs/native-roas-table-bin-retimed14/body-paths.json',ROOT/'isaac_port/runs/native-roas-table-rgbd-control05/executed-actuator-commands.npz',H/'bin-picking/pipeline-options.json',H/'bin-picking/scattered-home-robot-q.json',H/'physx-cpu.kit']
assets={str(p):sha(p.read_bytes()) for p in assetpaths}
(BASE/'asset-seals.json').write_text(json.dumps(assets,indent=2)+'\n')
# Runtime commands are DATA ONLY; never subprocess.run here.
def dockerpath(p):return '/workspace/project/'+str(p.relative_to(ROOT.parent))
import shlex
launch=[]
for n in [0,2]:
    case=BASE/('threads-'+str(n));case.mkdir();shutil.copy2(BASE/'original-trial-config.json',case/'trial-config.json')
    script=f'''import os,runpy,traceback\nfrom pathlib import Path\nos.environ['M80_COMPONENT_THREADS']={str(n)!r}\nos.environ['M80_COMPONENT_OUT']={dockerpath(case)!r}\nos.environ['M80_COMPONENT_BASE']={dockerpath(BASE)!r}\nos.environ['M80_COMPONENT_H']={dockerpath(H)!r}\nos.environ['M80_ROAS_RUN']='independent-thread-component-494'\ntry:\n runpy.run_path({dockerpath(BASE/'component.py')!r},run_name='__main__')\nexcept BaseException:\n (Path(os.environ['M80_COMPONENT_OUT'])/'error.txt').write_text(traceback.format_exc())\nfinally:\n import omni.kit.app\n omni.kit.app.get_app().post_quit(0)\n'''
    (case/'run.py').write_text(script)
    kit=['nice','-n','10','/isaac-sim/kit/kit',dockerpath(H/'physx-cpu.kit'),'--no-window','--allow-root','--exec',dockerpath(case/'run.py')]
    shell='exec '+shlex.join(kit)+' >'+shlex.quote(dockerpath(case/'kit.log'))+' 2>&1'
    command=['docker','exec','-d','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','TBB_NUM_THREADS=1','-e','CUDA_VISIBLE_DEVICES=','isaac-jake-gpu-3-9aeb439d20','/bin/bash','-c',shell]
    launch.append({'threads':n,'case':str(case),'sg_command':['sg','docker','-c',shlex.join(command)],'run_sequentially_only':True,'requires_parent_review_before_launch':True})
(BASE/'launch-commands.json').write_text(json.dumps(launch,indent=2)+'\n');(STAGE/'launch-commands.json').write_text(json.dumps(launch,indent=2)+'\n')
(STAGE/'extraction-seals.json').write_text(json.dumps(seals,indent=2)+'\n')
print(json.dumps({'prepared_only':True,'launched':False,'private_base':str(BASE),'launch_commands':str(BASE/'launch-commands.json'),'block_count':len(blocks),'source_manifest_entries':len(manifest)},indent=2))
