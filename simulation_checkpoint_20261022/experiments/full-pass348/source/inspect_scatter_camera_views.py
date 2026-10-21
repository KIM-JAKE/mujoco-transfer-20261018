"""Offline camera placement inspection; recorded transforms form pixels only."""
from pathlib import Path
import sys,os,json,numpy as np
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE/'bin-picking/realtime-deps-python312'),str(HERE),str(HERE.parents[2])]
from ray_camera_core import NumericRayCamera
from rgbd_scattered_rings import detect
from isaac_port.fieldkit.perception import CadPoseEstimator
cal=json.load(open(HERE/'bin-picking/asset-audit.json'))['baskets'][0];floor=np.array(cal['floor_origin_world_m']);est=CadPoseEstimator();root=HERE.parents[1]/'runs'/sys.argv[1];z=np.load(root/'physx-sequence-trace.npz');poses=dict(zip(z['camera_body_paths'].tolist(),z['camera_bodies'][0]));camera=NumericRayCamera(root/'camera-local-models.npz')
for name,offset in [('top',[0,0,.7]),('front',[0,-.2,.65]),('side',[.2,0,.65])]:
 packet=camera.capture(floor+offset,floor+[0,0,.025],.44,640,640,body_poses=poses);r,m=detect(packet,cal,est);out=root/f'camera-placement-{name}';out.mkdir(exist_ok=True);np.savez_compressed(out/'rgbd.npz',**packet);(out/'detections.json').write_text(json.dumps(r,indent=2));rgb=packet['rgb'].copy()
 for mask in m:rgb[mask]=(rgb[mask]*.5+[20,120,40]).clip(0,255)
 (out/'preview.ppm').write_bytes(b'P6\n640 640\n255\n'+rgb.tobytes());print(name,[(p['part'],np.round(p['center_m'],3).tolist())for p in r['instances']],flush=True)
