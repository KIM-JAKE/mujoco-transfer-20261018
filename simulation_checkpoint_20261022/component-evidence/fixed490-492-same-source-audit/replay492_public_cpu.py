from pathlib import Path
import sys,json,hashlib
import numpy as np
ROOT=Path('/workspace/project/mujoco');H=ROOT/'isaac_port/native_hand_layout/roas-table';SOURCE=ROOT/'.local/rgbd-gpu3-fast-spread';sys.path[:0]=[str(SOURCE),str(H/'bin-picking/realtime-deps-python312'),str(H),str(ROOT)]
from isaac_port.fieldkit.perception import CadPoseEstimator
from rgbd_scattered_rings import detect
from rgbd_geometry import cloud
p=ROOT/'isaac_port/runs/native-roas-table-fixed-front-v492-width-memo-placed-broadphase-plus';observations=json.loads((p/'rgb-observations.json').read_text());est=CadPoseEstimator();rows=[]
for oi in [15,17,18]:
 ob=observations[oi];path=p/'rgb-perception'/Path(ob['work']).name/'rgbd.npz';packet=dict(np.load(path));T=packet['T_cell_camera'];seed=T[:3,3]+T[:3,2]*(packet['K'][0,0]*.10/packet['rgb'].shape[1]);cal=dict(floor_origin_world_m=seed-[0,0,.06],basket_to_world_R=np.eye(3),outer_bounds_world_m=[seed-.065,seed+.065],allow_sideways_faces=True,gpu_metric_proposals=False)
 result,masks=detect(packet,cal,est,desired_part='ring2',seed=seed);pose=result['poses'][0];axis=np.array(pose['axis']);actual=ob['result']['pose'];points=cloud(packet)[masks[0]];_,_,V=np.linalg.svd(points-points.mean(0),full_matrices=False);svd=V[-1];svd*=1 if svd@axis>=0 else -1;Htcp=np.array(ob['hand_TCP_world']);relative=Htcp[:3,:3].T@svd
 row=dict(frame=oi+1,packet_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),CPU_full_hypothesis_axis=axis.tolist(),recorded_GPU_axis=actual['axis'],axis_difference_to_recorded_rad=float(np.arccos(np.clip(axis@actual['axis'],-1,1))),CPU_mask_plane_SVD_axis=svd.tolist(),SVD_relative_axis=relative.tolist(),CPU_cap_points=len(points),mask_plane_rms_m=float(np.sqrt(np.mean(((points-points.mean(0))@svd)**2))),mask_selected_from_public_RGBD_only=True,CUDA_used=False)
 if rows:row['SVD_relative_delta_rad']=float(np.arccos(np.clip(relative@rows[-1]['SVD_relative_axis'],-1,1)))
 rows.append(row)
report=dict(scope='Offline CPU-only replay and plane SVD of three saved public RGB-D packets. No actorposes, no hardware/physics/source changes. Camera/SDKrelative tilt confirmation only, not privileged truth.',rows=rows);out=ROOT/'.local/render-current/replay492-public-cpu-result.json';out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
