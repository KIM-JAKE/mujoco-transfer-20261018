from pathlib import Path
import sys,json,hashlib,time
import numpy as np
H=Path('/workspace/project/mujoco/isaac_port/native_hand_layout/roas-table');sys.path[:0]=[str(H/'bin-picking/realtime-deps-python312'),str(H)]
from warp_ray_camera import WarpRayCamera
R=H.parents[1]/'runs/native-roas-table-scatter-stable-v278-self-clear-park-camera-446'
out=Path('/workspace/project/mujoco/.local/field-fair-diagnostic565')
obs=json.loads((R/'empty-park-observation.json').read_text());trace=np.load(R/'physx-sequence-trace.npz');wall=obs['captured_wall_s'];k=int(np.argmin(abs(trace['time_s']-wall)))
poses=dict(zip(trace['camera_body_paths'].tolist(),trace['camera_bodies'][k]))
target=np.asarray(obs['camera_target_from_calibration_m']);eye=target+[0,0,1.15]
cam=WarpRayCamera(R/'camera-local-models.npz');began=time.perf_counter();p=cam.capture(eye,target,span=.65,width=1024,height=1024,body_poses=poses,far_clip=2.5)
np.savez_compressed(out/'fresh-overhead-rgbd.npz',**p)
(out/'fresh-overhead-render.json').write_text(json.dumps(dict(scope='One new virtual pinhole camera exposure of genuine recorded physically parked Isaac scene; private dynamic poses renderer-only; no robot/stock writes or physics steps',source_run=str(R),source_wall_s=wall,source_trace_wall_s=float(trace['time_s'][k]),trace_index=k,camera_eye=eye.tolist(),camera_target=target.tolist(),span_m=.65,width=1024,height=1024,physical_camera_tested=False,robot_command_sent=False,render_s=time.perf_counter()-began,source_trace_sha256=hashlib.sha256((R/'physx-sequence-trace.npz').read_bytes()).hexdigest()),indent=2))
print('CAPTURE',out,flush=True)
