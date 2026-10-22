"""Ten mixed CAD rings, collision-separated spawn, then free PhysX settling.
All physical settling, grasp selection, IK and execution occur in Isaac Sim.
"""
from pathlib import Path
import json,copy,shutil,sys,math
import numpy as np
HERE=Path(__file__).resolve().parent;OUT=HERE/'bin-picking';seed=int(sys.argv[1])if len(sys.argv)>1 else 101;folder=OUT/'cases'/('mixed'+str(seed));folder.mkdir(parents=True,exist_ok=True)
p=json.load(open(OUT/'cases/sequence/arm-plan.json'));p.update(continue_sequence=False,capture_camera=True,clutter_seed=seed,clutter_ring_count=10,clutter_counts={'ring1':5,'ring2':5})
b=json.load(open(OUT/'asset-audit.json'))['baskets'][0];B=np.array(b['basket_to_world_R']);O=np.array(b['floor_origin_world_m']);rng=np.random.default_rng(seed);kinds=['ring1']*5+['ring2']*5;rng.shuffle(kinds)
grid=np.array([[x,y]for x in [-.0475,.0475]for y in [-.127,-.0635,0,.0635,.127]]);order=rng.permutation(10);items=[]
def rz(t):return np.array([[math.cos(t),-math.sin(t),0],[math.sin(t),math.cos(t),0],[0,0,1.]])
for i in order:
    part=kinds[i];r=.0375 if part=='ring1'else .031;half=.00775 if part=='ring1'else .007;t=rng.uniform(-.035,.035);Rx=np.array([[1,0,0],[0,math.cos(t),-math.sin(t)],[0,math.sin(t),math.cos(t)]]);R=rz(rng.uniform(-math.pi,math.pi))@Rx;extent=half*math.cos(t)+r*abs(math.sin(t))
    for attempt in range(2000):
        xy=rng.uniform([-.055,-.125],[.055,.125]);z=extent+.001
        for old in items:
            if np.linalg.norm(xy-np.array(old['xy']))<r+old['radius']+.002:z=max(z,old['local_center_z']+old['vertical_extent']+extent+.002)
        if z+extent<.085:break
    else:raise RuntimeError('Cannot spawn ten rings inside the original basket without interpenetration')
    items.append(dict(id=part+'_'+str(i),part=part,source_P=(O+B@np.r_[xy,z]).tolist(),source_R=(B@R).tolist(),xy=xy.tolist(),radius=r,vertical_extent=extent,local_center_z=z,main=False))
for part in ['ring1','ring2']:
    candidates=[r for r in items if r['part']==part and r['xy'][0]<0]or[r for r in items if r['part']==part];chosen=max(candidates,key=lambda r:r['local_center_z']);chosen['main']=True;p['scene_parts'][part]=dict(source_P=chosen['source_P'],source_R=chosen['source_R']);
    if part in p['parts']:p['parts'][part].update(source_P=chosen['source_P'],source_R=chosen['source_R'])
p['scene_parts']['pin']=dict(source_P=[.190,-.790,.91605],source_R=np.diag([1.,-1.,-1.]).tolist());p.update(pin_holder_origin_m=[.2075,-.85,.8843],pin_holder_yaw_deg=90);p['clutter_items']=items
p['spawn_distribution']='Independent uniform XY, random CAD yaw/tilt, vertically separated overlapping footprints; free PhysX settling before observation'
(folder/'arm-plan.json').write_text(json.dumps(p,indent=2));shutil.copyfile(OUT/'initial-robot-pose.json',folder/'initial-robot-pose.json');print('MIXED_RING_SCENE',seed,10,max(r['local_center_z']+r['vertical_extent']for r in items),flush=True)
