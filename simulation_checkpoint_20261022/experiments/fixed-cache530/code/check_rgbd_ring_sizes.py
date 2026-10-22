"""Recorded RGB-D regression: distinguish a large bore from a small rim.

Only RGB, depth, intrinsics and fixed camera/bin calibration enter detection.
The annotated expected classes are test assertions, never controller inputs.
"""
from pathlib import Path
import json,sys,types,time,numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from rgbd_scattered_rings import detect
RUNS=HERE.parents[1]/'runs'
source=RUNS/'native-roas-table-scatter-opt-v55-decomp-403'
observations=json.loads((source/'rgb-observations.json').read_text())
cal=json.loads((HERE/'bin-picking/asset-audit.json').read_text())['baskets'][0]
rows=[]
for index in [0,6,11]:
    observation=observations[index]
    packet=dict(np.load(source/'rgb-perception'/Path(observation['work']).name/'rgbd.npz'))
    seed=np.asarray(observation.get('result',{}).get('pose',{}).get('center_m',[.4844,-.7586,1.2435]))
    calibration=dict(cal,global_plane_faces=True) if observation['mode']=='initial' else dict(floor_origin_world_m=seed-[0,0,.06],basket_to_world_R=np.eye(3),outer_bounds_world_m=[seed-.065,seed+.065],allow_sideways_faces=True)
    started=time.perf_counter()
    result,_=detect(packet,calibration,None,desired_part=observation.get('part'),seed=None if observation['mode']=='initial' else seed)
    poses=result['instances']
    if index==0:
        assert sum(p['part']=='ring2' for p in poses)==2
        assert sum(p['part']=='ring1' for p in poses)==3
    if index==6:
        confusing=np.array([.4844,-.7591,1.1214])
        assert not any(p['part']=='ring2' and np.linalg.norm(np.asarray(p['center_m'])-confusing)<.008 for p in poses)
        assert any(p['part']=='ring1' and np.linalg.norm(np.asarray(p['center_m'])-confusing)<.003 for p in poses)
        assert sum(p['part']=='ring2' for p in poses)==3
    if index==11:
        assert not poses, 'A transported large-ring bore must not pass as a small ring'
    rows.append(dict(observation=index,mode=observation['mode'],parts=[p['part'] for p in poses],processing_s=time.perf_counter()-started,passed=True))
result=dict(passed=True,source=source.name,scope='Recorded image regression only; no grasp or assembly success',simulator_body_pose_input=False,tests=rows)
destination=HERE/'bin-picking/rgbd-size-regression-20261021.json'
destination.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
