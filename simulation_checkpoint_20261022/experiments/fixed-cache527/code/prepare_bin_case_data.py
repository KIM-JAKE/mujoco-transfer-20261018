"""Write scene-only variants; all new motion is solved inside Isaac Sim."""
from pathlib import Path
import json,copy,shutil
import numpy as np
HERE=Path(__file__).resolve().parent;OUT=HERE/'bin-picking'
template=json.load(open(OUT/'arm-plan.json'));assets=json.load(open(OUT/'asset-audit.json'));b=assets['baskets'][0];O=np.array(b['floor_origin_world_m']);B=np.array(b['basket_to_world_R'])
for label,part,lp,gap,opening in [('ring2-rim','ring2',[0,-.02735,.0035],.0075,.016),('pin-head','pin',[0,0,.006],.013,.022),('pin-body','pin',[0,0,-.003],.013,.022)]:
    folder=OUT/'cases'/label;folder.mkdir(parents=True,exist_ok=True);p=copy.deepcopy(template);r=copy.deepcopy(next(iter(p['parts'].values())))
    r['source_P']=([.190,-.790,.91605]if part=='pin'else(O+B@[-.06,.07,.007]).tolist());r['source_R']=(np.diag([1.,-1.,-1.])if part=='pin'else B).tolist();p['parts']={part:r};p['adapt_grasp']=dict(localP=lp,localR=np.diag([1.,-1.,-1.]).tolist(),gap=gap,opening=opening);p['lift_m']=.05 if part=='pin'else .145;p['proposed_pin_holder']=part=='pin';p['grasp_command_compression_m']=.004
    if part=='pin':p.update(pin_holder_origin_m=[.2075,-.85,.8843],pin_holder_yaw_deg=90);p['adapt_grasp'].update(localP=[0,0,-.006]if label=='pin-head'else[0,0,.003],localR=np.eye(3).tolist())
    (folder/'arm-plan.json').write_text(json.dumps(p,indent=2));shutil.copyfile(OUT/'initial-robot-pose.json',folder/'initial-robot-pose.json')
print('BIN_CASE_SCENE_DATA_READY',flush=True)
