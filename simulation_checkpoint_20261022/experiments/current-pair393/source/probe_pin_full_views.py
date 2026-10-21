from pathlib import Path
import sys,os,time,json,numpy as np
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE/'bin-picking/realtime-deps-python312'),str(HERE),str(HERE.parents[2])];os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[-4:])
from ray_camera_core import NumericRayCamera
from rgbd_fast_tracker import FastRGBDTracker
from isaac_port.fieldkit.perception import CadPoseEstimator
source=HERE.parents[1]/'runs/native-roas-table-direct-v14-seed106';z=np.load(source/'physx-sequence-trace.npz');packet=np.load(source/'rgb-perception/observation-009-held/rgbd.npz');seed=packet['T_cell_camera'][:3,3]-[-.22,-.15,-.12];body=dict(zip(z['camera_body_paths'].tolist(),z['camera_bodies'][-1]));camera=NumericRayCamera(source/'camera-local-models.npz');tracker=FastRGBDTracker(CadPoseEstimator());out=source/'view-probe';out.mkdir(exist_ok=True);results=[]
for i,offset in enumerate([[-.25,0,-.12],[-.25,0,-.22],[-.18,-.05,-.18],[.25,0,-.15],[-.18,.18,-.18],[0,-.25,-.2],[0,.25,-.2],[-.3,-.05,-.05],[-.15,0,-.25],[.15,-.1,-.2]]):
 p=camera.capture(seed+offset,seed,.10,256,256,body_poses=body);start=time.perf_counter();row=dict(offset=offset)
 try:pose,mask=tracker.estimate(p,'pin',seed,np.array([0,0,1.]));row.update(success=True,pose=pose)
 except Exception as e:row.update(success=False,error=str(e));mask=np.zeros((256,256),bool)
 row['tracking_ms']=(time.perf_counter()-start)*1000;results.append(row);rgb=p['rgb'].copy();rgb[mask]=(rgb[mask]*.5+[25,120,60]).clip(0,255).astype(np.uint8);(out/('%02d.ppm'%i)).write_bytes(b'P6\n256 256\n255\n'+rgb.tobytes());print('PIN_VIEW',row,flush=True)
(out/'results.json').write_text(json.dumps(results,indent=2))
