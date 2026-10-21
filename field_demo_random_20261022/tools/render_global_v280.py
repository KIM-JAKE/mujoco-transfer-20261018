"""SIM image formation ONLY. Actor snapshot never returned to field controller."""
from pathlib import Path
import sys,json,time,hashlib
import numpy as np
ROOT=Path('/workspace/project');REPO=ROOT/'mujoco';H=REPO/'isaac_port/native_hand_layout/roas-table';sys.path[:0]=[str(H),str(H/'bin-picking/realtime-deps-python312')]
from ray_camera_core import NumericRayCamera
RUN=REPO/'isaac_port/runs/native-roas-table-scatter-stable-v280-park-single-initial-gpu-all3-446';OUT=ROOT/'field-demo-random-recorded-closure-20261022/replay/recorded-closure-v280'
z=np.load(RUN/'physx-sequence-trace.npz');print(z.files,flush=True)
key=next(k for k in ['time_s','times_s','times','time']if k in z.files);index=int(np.argmin(abs(z[key]-7.020404767)))
model=RUN/'camera-local-models.npz';paths=np.load(model)['paths'].tolist();tracepaths=z['camera_body_paths'].tolist();snapshot={p:z['camera_bodies'][index,tracepaths.index(p)]for p in paths}
cam=NumericRayCamera(model);packet=cam.capture([-.20,-1.20,1.80],[.50,-.72,1.16],span=1.1,width=768,height=768,far_clip=4.,body_poses=snapshot)
np.savez_compressed(OUT/'global-rgbd.npz',**packet)
overhead=dict(np.load(OUT/'initial-rgbd.npz'));eye=overhead['T_cell_camera'][:3,3];target=eye+overhead['T_cell_camera'][:3,2];twin=cam.capture(eye,target,span=768/overhead['K'][0,0],width=768,height=768,far_clip=1.0298816669,body_poses=snapshot);np.savez_compressed(OUT/'overhead-same-snapshot-rgbd.npz',**twin)
record=dict(scope='SECOND VIRTUAL calibrated RGB-D camera, same archived SIM scene as overhead. Private actor snapshot used ONLY inside image formation, never field detection/planning.',hardware_camera_tested=False,physical_camera_count_assumption=2,scene_snapshot_time_s=float(z[key][index]),requested_overhead_capture_time_s=7.020404767,trace_snapshot_index=index,T_cell_camera=packet['T_cell_camera'].tolist(),K=packet['K'].tolist(),render_s=cam.last_capture_s,source_scene_model_sha256=hashlib.sha256(model.read_bytes()).hexdigest())
(OUT/'global-render-provenance.json').write_text(json.dumps(record,indent=2));print(json.dumps(record),flush=True)
