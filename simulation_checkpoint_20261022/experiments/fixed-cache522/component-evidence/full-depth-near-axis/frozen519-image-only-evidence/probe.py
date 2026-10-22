"""READ-ONLY camera proposal. Trace poses are IMAGE FORMATION ONLY.

Detector receives only newly rendered RGB/depth/K/extrinsics and public seed.
"""
from pathlib import Path
import sys,json,time,numpy as np,hashlib
root=Path('/workspace/project/mujoco');run=next((root/'isaac_port/runs').glob('*v519-*'));out=root/'.local/fixed519-axis-view';sys.path[:0]=[str(run/'code_snapshot'),str(run/'code_snapshot/bin-picking/realtime-deps-python312'),str(root),str(root/'.local/render-current/frozen507/py312')]
from warp_ray_camera import WarpRayCamera
from rgbd_fast_tracker import FastRGBDTracker
from isaac_port.fieldkit.perception import CadPoseEstimator
from PIL import Image
obs=json.load(open(run/'rgb-observations.json'));prior=obs[68];final=obs[69];A=np.asarray(prior['hand_TCP_world']);H=np.asarray(final['hand_TCP_world']);P=np.asarray(prior['result']['pose']['center_m']);oldaxis=np.asarray(prior['result']['pose']['axis']);seed=H[:3,:3]@A[:3,:3].T@(P-A[:3,3])+H[:3,3];axis=H[:3,:3]@A[:3,:3].T@oldaxis;axis/=np.linalg.norm(axis);tangent=H[:3,2]-axis*(axis@H[:3,2]);tangent/=np.linalg.norm(tangent)
trace=np.load(run/'physx-sequence-trace.npz');k=int(np.argmin(abs(trace['time_s']-final['captured_wall_s'])));poses={p:v for p,v in zip(trace['camera_body_paths'].tolist(),trace['camera_bodies'][k])};camera=WarpRayCamera(run/'camera-local-models.npz');tracker=FastRGBDTracker(CadPoseEstimator());results=[]
for lateral in [0.,.02,.04,.06,-.04,-.06]:
 eye=seed-.25*axis+lateral*tangent;start=time.perf_counter();packet=camera.capture(eye,seed,span=.10,width=768,height=768,body_poses=poses,far_clip=2.);prefix=f'tail_axis_{lateral:+.3f}';np.savez_compressed(out/(prefix+'.npz'),**{key:value for key,value in packet.items()if key in ['rgb','depth_m','K','T_cell_camera','points_camera_m']});Image.fromarray(packet['rgb']).save(out/(prefix+'.png'));row=dict(lateral_m=lateral,eye=eye.tolist(),target=seed.tolist(),axis=axis.tolist(),tangent_from_public_HandZ=tangent.tolist(),render_s=time.perf_counter()-start,accepted=False)
 try:
  p,m=tracker.estimate(packet,'pin',seed,axis,robust_pin_cap_circle=True);row.update(accepted=True,pose=p,original_full_depth_goal_error_m=float(np.linalg.norm(np.asarray(p['center_m'])-np.asarray([-.30203095784,.22724999619,1.52875]))),original_axis_error_rad=float(np.arccos(np.clip(np.asarray(p['axis'])[2],-1,1))));Image.fromarray(np.where(m[:,:,None],(packet['rgb'].astype(np.float64)*.5+np.array([0,110,0])).clip(0,255).astype(np.uint8),packet['rgb'])).save(out/(prefix+'-mask.png'))
 except Exception as e:row['error']=type(e).__name__+': '+str(e)
 results.append(row);print(json.dumps(row),flush=True)
result=dict(source_run=run.name,image_formation_only_from_trace=True,trace_wall_s=float(trace['time_s'][k]),public_snapshot_wall_s=final['captured_wall_s'],seed_from_lastqualified_public_RGBD_and_same_captureSDKHand=True,no_simulator_poses_to_detector=True,no_physics_steps=True,no_runtime_changes=True,original_240deg_gate_unchanged=True,rows=results);(out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
