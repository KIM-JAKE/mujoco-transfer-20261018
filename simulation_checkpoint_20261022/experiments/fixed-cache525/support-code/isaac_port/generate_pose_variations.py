"""Independent-CAD synthetic camera scenes with perturbed stock poses.

Ground truth is saved separately and is not consumed by vision.detect(). This
is a software regression dataset, not validation of any physical RGB-D camera.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,json
import numpy as np
from scipy.spatial.transform import Rotation
from fieldkit.perception import ASSETS,PARTS,pose


def generate(output,case,seed=571):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);rng=np.random.default_rng(seed)
    meta=json.loads((ASSETS/'part-models.json').read_text())['parts'];surfaces=np.load(ASSETS/'part-validation-surfaces.npz')
    sign=1 if case=='shift-a'else-1
    shifts={'ring1':np.array([.004,-.003,0])*sign,'ring2':np.array([-.005,.003,0])*sign,'pin':np.array([.003,.002,0])*sign}
    angles={'ring1':4*sign,'ring2':-3*sign,'pin':3*sign}
    eye=np.array([-.2,-.8,1.9]);look=np.array([-.3,-.16,1.3]);z=(look-eye)/np.linalg.norm(look-eye);x=np.cross(z,[0,0,1]);x/=np.linalg.norm(x);y=np.cross(z,x);camera=np.eye(4);camera[:3,:3]=np.c_[x,y,z];camera[:3,3]=eye
    priors={n:pose(meta[n]['nominal_position'],meta[n]['nominal_quaternion_wxyz'])for n in PARTS};truth={};cloud=[];labels=[]
    for index,name in enumerate(PARTS,1):
        t=priors[name].copy();t[:3,:3]=Rotation.from_euler('z',angles[name],degrees=True).as_matrix()@t[:3,:3];t[:3,3]+=shifts[name];truth[name]=t
        points=surfaces[name+'_points'];normal=surfaces[name+'_normals'];world=points@t[:3,:3].T+t[:3,3];norm=normal@t[:3,:3].T;visible=np.sum(norm*(eye-world),axis=1)>0;world=world[visible];world=world[rng.choice(len(world),min(1800,len(world)),replace=False)]
        world+=rng.normal(0,.00025,world.shape);cloud.append(world);labels.extend([index]*len(world))
        clutter=rng.uniform(world.min(axis=0)-.006,world.max(axis=0)+.006,(200,3));cloud.append(clutter);labels.extend([0]*len(clutter))
    world=np.vstack(cloud);points_camera=(world-camera[:3,3])@camera[:3,:3]
    # No semantic labels in the default test: automatic spatial ROI selection.
    np.savez_compressed(output/'snapshot.npz',points_camera_m=points_camera)
    now=datetime.now(timezone.utc).isoformat();capture_id=f'SYNTHETIC-{case}-{seed}'
    (output/'snapshot.json').write_text(json.dumps(dict(capture_id=capture_id,captured_at_utc=now,camera_frame='synthetic_camera',calibration_id='SYNTHETIC_NOT_FIELD_CALIBRATION'),indent=2)+'\n')
    config=dict(calibration_id='SYNTHETIC_NOT_FIELD_CALIBRATION',camera_frame='synthetic_camera',T_cell_camera=camera.tolist(),nominal_stock_poses={n:t.tolist()for n,t in priors.items()},roi_margin_m=.035,hardware_ready=False)
    config['pin_holder_tracks_pin']=True
    (output/'camera-config.json').write_text(json.dumps(config,indent=2)+'\n')
    gt=dict(scope='Synthetic independent CAD surface samples, partial visible faces, 0.25mm Gaussian point noise and clutter; no physical camera',case=case,seed=seed,parts={n:dict(position=t[:3,3].tolist(),quaternion_wxyz=Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]].tolist(),T_cell_part=t.tolist())for n,t in truth.items()},move_pin_holder_with_pin=True,m80_machine_fixed=True)
    (output/'ground-truth.json').write_text(json.dumps(gt,indent=2)+'\n')
    return gt


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--case',choices=['shift-a','shift-b'],required=True);p.add_argument('--seed',type=int,default=571);a=p.parse_args();generate(a.output,a.case,a.seed)
