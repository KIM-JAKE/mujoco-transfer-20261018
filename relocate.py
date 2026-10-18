"""Patch runtime text paths in the restored copy; preserve source archive bytes."""
import argparse
import hashlib
import json
import os
from pathlib import Path

SKIP_DIRS={'.git','.venv','.venv-curobo','sam3_venv','site-packages',
           'vendor','outputs','logs','provenance','code_snapshot','__pycache__'}
TEXT_EXT={'.py','.sh','.xml','.yaml','.yml','.toml','.ini','.cfg'}
CONFIG_NAMES={'active_setup.json','task_config.json','copy_manifest.json'}
SOURCE_DIRS=['keti_m80_mujoco_20261013','g2_replenishment_poc_20261016',
             'fr3_cube_mujoco_handoff_20261012','fr3_cube_mujoco_handoff_20261012_v2',
             'robotics_agent']

def sha(b): return hashlib.sha256(b).hexdigest()

def main(root):
    mapping=[('/home/jovyan/data/.cache/huggingface/hub/models--facebook--sam3.1/snapshots/daa63191845a41281374e725f4c9e51c7a824460/sam3.1_multiplex.pt',str(root/'robotics_agent/mujoco_duo_fr3/runtime/sam31_checkpoint/sam3.1_multiplex.pt')),
             ('/home/jovyan/data-1/jake/ego-scale-compiler/.venv/bin/python',str(root/'robotics_agent/mujoco_duo_fr3/runtime/sam31_venv/bin/python')),
             ('/home/jovyan/data/wjkim/robotics_agent',str(root/'robotics_agent')),
             ('/home/jovyan/data-1/jake',str(root))]
    changed=[];links=[];skipped=[]
    for dirname in SOURCE_DIRS:
        parent=root/dirname
        if not parent.exists(): continue
        for base,dirs,files in os.walk(parent,followlinks=False):
            here=Path(base)
            for name in list(dirs):
                p=here/name
                if p.is_symlink():
                    target=os.readlink(p);new=target
                    for old,replacement in mapping: new=new.replace(old,replacement)
                    if new!=target:
                        p.unlink();p.symlink_to(new,target_is_directory=True)
                        links.append({'path':str(p.relative_to(root)),'old':target,'new':new})
                    dirs.remove(name)
                elif name in SKIP_DIRS: dirs.remove(name)
            for name in files:
                p=here/name
                if p.is_symlink():
                    target=os.readlink(p);new=target
                    for old,replacement in mapping: new=new.replace(old,replacement)
                    if new!=target:
                        p.unlink();p.symlink_to(new)
                        links.append({'path':str(p.relative_to(root)),'old':target,'new':new})
                    continue
                if p.suffix not in TEXT_EXT and name not in CONFIG_NAMES: continue
                if p.stat().st_size>20_000_000:
                    skipped.append(str(p.relative_to(root)));continue
                data=p.read_bytes()
                if b'\0' in data:continue
                try: text=data.decode('utf-8')
                except UnicodeDecodeError:continue
                new=text
                for old,replacement in mapping:new=new.replace(old,replacement)
                if new==text:continue
                b=new.encode('utf-8');p.write_bytes(b)
                changed.append({'path':str(p.relative_to(root)),
                                'before_sha256':sha(data),'after_sha256':sha(b)})
    report={'target_root':str(root),'changed_files':changed,
            'rewritten_symlinks':links,'skipped_large_text_files':skipped,
            'note':'Archived source remains byte-original; restored runtime text was relocated.'}
    (root/'relocation_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(f'Relocated {len(changed)} text files and {len(links)} symlinks')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args();main(args.root.resolve())
