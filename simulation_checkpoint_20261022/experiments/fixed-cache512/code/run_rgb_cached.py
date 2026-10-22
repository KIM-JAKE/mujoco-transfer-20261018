"""Run fresh dynamics from the identical prebuilt native-hand cell."""
from pathlib import Path
import os,sys,json,shutil,traceback
import numpy as np
from pxr import Usd,UsdPhysics
import omni.kit.app
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1])]
OUT=HERE.parents[1]/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
source=HERE.parents[1]/'runs/native-roas-table-bin-retimed14'
try:
    stage=Usd.Stage.Open(str(source/'scene-authored.usda'))
    paths=json.load(open(source/'body-paths.json'));hands=paths.pop('native_hands')
    initial=json.load(open(HERE.parents[1]/'runs/native-roas-table-bin-quick07/initial-robot-pose.json'))
    drives={p.GetName():UsdPhysics.DriveAPI(p,'angular')for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in p.GetName()}
    base=json.load(open(source/'pipeline-results.json'))
    report={k:base[k]for k in ['engine','pin_collision_approximation','scene_ring_color','starts_with_rings_already_installed']}
    report.update(part_attachments=False,full_task_pass=False,continuous_cycle_pass=False,hardware_tested=False,vision_end_to_end_pass=False,scene_cache_source=source.name)
    for name in ['arm-plan.json','body-paths.json']:shutil.copyfile(source/name,OUT/name)
    target=OUT/'scene-authored.usda'
    if not target.exists():target.symlink_to('../'+source.name+'/scene-authored.usda')
    from execute_rgb_motion import execute
    os.environ['M80_ROAS_RGB_ONLY']='1'
    execute(stage,paths,hands,drives,initial,report,OUT,HERE)
    omni.kit.app.get_app().post_quit(0)
except Exception:
    (OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
