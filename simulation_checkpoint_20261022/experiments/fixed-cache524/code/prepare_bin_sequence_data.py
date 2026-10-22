"""Assemble existing CAD/task data; motion solving/execution stays in Isaac."""
from pathlib import Path
import json,copy,shutil
import numpy as np
HERE=Path(__file__).resolve().parent;OUT=HERE/'bin-picking';PORT=HERE.parents[1];folder=OUT/'cases/sequence';folder.mkdir(parents=True,exist_ok=True)
p=json.load(open(OUT/'arm-plan.json'));p.update(continue_sequence=True,proposed_pin_holder=True,capture_camera=False,acceptance_mode='bin_pick_relocalize_after_lift',minimum_lift_m=.12)
assets=json.load(open(OUT/'asset-audit.json'));b=assets['baskets'][0];O=np.array(b['floor_origin_world_m']);B=np.array(b['basket_to_world_R']);p['scene_parts']=copy.deepcopy(p['parts']);p['scene_parts']['ring2']=dict(source_P=(O+B@[-.06,.10,.007]).tolist(),source_R=B.tolist());p['scene_parts']['pin']=dict(source_P=[.230,-.790,.91105],source_R=np.eye(3).tolist())
h=json.load(open(OUT/'handoff-screen.json'));p['handoffs']={}
for part in ['ring1','ring2','pin']:
    if part!='pin':
        r=h['parts'][part]['handoff_candidates'][0];b=r['recipient_recipe'];p['handoffs'][part]=dict(center=r['part_center'],localP=b['localP'],localR=b['localR'],gap=b['gap'],recipient_q=r['recipient_q'])
    else:
        b=next(r for r in h['parts'][part]['insertion_candidates']if r['side']=='left');p['handoffs'][part]=dict(center=[-.15,-.25,1.30],localP=b['localP'],localR=b['localR'],gap=b['gap'],recipient_q=b['q_insert'][0])
source=json.load(open(PORT/'runs/contact-trim-03/recording/config.json'))['parts'];p['goals']={r['name']:dict(P=r['goal_origin_m'],R=np.eye(3).tolist())for r in source}
(folder/'arm-plan.json').write_text(json.dumps(p,indent=2));shutil.copyfile(OUT/'initial-robot-pose.json',folder/'initial-robot-pose.json')
print('ISAAC_SEQUENCE_SCENE_DATA_READY',flush=True)
