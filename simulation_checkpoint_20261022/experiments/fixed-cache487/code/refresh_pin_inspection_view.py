"""Render the pin underside from an observable inspection angle; replay only."""
from pathlib import Path
import os,sys,json,numpy as np
os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[-12:-10])
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE/'bin-picking/realtime-deps-python312'),str(HERE)]
from ray_camera_core import NumericRayCamera
source=HERE.parents[1]/'runs'/sys.argv[1];out=HERE.parents[1]/'runs'/sys.argv[2];manifest=json.loads((out/'video-frames.json').read_text());z=np.load(source/'physx-sequence-trace.npz');paths=z['camera_body_paths'].tolist();camera=NumericRayCamera(source/'camera-local-models.npz');count=0
for row in manifest['frames']:
 j=int(np.argmin(abs(z['time_s']-row['sim_time_s'])))
 phase=str(z['pin_phase'][j]) if 'pin_phase'in z else ''
 if phase not in ['pin_tail_orient_free','pin_tail_carry_to_preinsert','pin_single_axial_insert','pin_full_depth_hold','pin_clamp_hold','pin_release_after_full_depth','pin_axial_withdraw','pin_retention','all_parts_final_retention']:continue
 row['active_part']='pin';row['event']='pin_continuous_task';target=z['parts'][j,2,:3].astype(float)
 file=out/row['file'];left=np.frombuffer(file.read_bytes().split(b'\n',3)[3],np.uint8).reshape(360,960,3)[:,:480].copy();pose=dict(zip(paths,z['camera_bodies'][j]));right=camera.capture(target+(np.array([-.22,-.15,-.12]) if target[1]>.05 else np.array([-.22,0,.12])),target,.10,480,360,body_poses=pose,far_clip=5.)['rgb'];file.write_bytes(b'P6\n960 360\n255\n'+np.concatenate([left,right],axis=1).tobytes());row['inspection_camera']='Pin underside; same relative view as live RGB-D full-depth check';count+=1
manifest['pin_inspection_frames_refreshed']=count;(out/'video-frames.json').write_text(json.dumps(manifest,indent=2));print('PIN_INSPECTION_VIEWS',count,flush=True)
