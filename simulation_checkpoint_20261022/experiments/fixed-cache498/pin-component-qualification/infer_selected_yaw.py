"""Read-only actualSDK endpoint comparison to original currentG yaw proposals."""
import json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
root=Path('/home/gpuadmin/workspace/jake/mujoco');cat=json.load(open(root/'isaac_port/runs/native-roas-table-scatter-stable-v251-corrected-pin-catalog/pin-native-grasp-catalog.json'))['candidates'][0];out=[]
for v in[498,499]:
 run=next((root/'isaac_port/runs').glob(f'*v{v}-*'));obs=[o for o in json.load(open(run/'rgb-observations.json'))if o.get('part')=='pin'and o['mode']=='held'and o.get('result',{}).get('pose')];tr=np.load(run/'physx-sequence-trace.npz');G=np.asarray(cat['grasp_transform']).copy();G[2,3]+=.003;selected=[];end=None
 for o in obs:
  k=int(np.argmin(abs(tr['time_s']-o['captured_wall_s'])));ph=str(tr['pin_phase'][k]);H=np.asarray(o['hand_TCP_world'])
  if ph in['pin_lift_settle','pin_lift_independent_confirmation','pin_confirmed_postlift_preload_ramp']:
   p=o['result']['pose'];axis=H[:3,:3].T@p['axis'];old=G[:3,2];cross=np.cross(old,axis);n=np.linalg.norm(cross);G[:3,:3]=Rotation.from_rotvec(cross/max(n,1e-12)*np.arctan2(n,old@axis)).as_matrix()@G[:3,:3];G[:3,3]=(np.linalg.inv(H)@np.r_[p['center_m'],1])[:3];selected.append(dict(work=Path(o['work']).name,phase=ph))
  if ph=='pin_precision_encoder_settle':end=o;break
 assert end is not None
 H=np.asarray(end['hand_TCP_world']);base=np.asarray(cat['goal_part_R']);rr=[]
 for yaw in[0,90,-90,180]:
  targetR=base@Rotation.from_rotvec([0,0,np.deg2rad(yaw)]).as_matrix()@G[:3,:3].T;err=float(Rotation.from_matrix(targetR@H[:3,:3].T).magnitude());rr.append(dict(candidate_yaw_deg=yaw,actualSDK_orientation_residual_rad=err))
 row=dict(run=run.name,scope='Infer selected branch only from originalcatalogue/freshRGBD grasp updates and actualSDKHand endpoint; noactorposes',original_preload_G=G.tolist(),Gupdate_observations=selected,orientation_comparisons=rr,inferred_selected_yaw_deg=min(rr,key=lambda r:r['actualSDK_orientation_residual_rad'])['candidate_yaw_deg']);out.append(row);print(v,json.dumps(row['orientation_comparisons']),row['inferred_selected_yaw_deg'])
Path('/tmp/fixed498-pin-component/actual498499-public-yaw-inference.json').write_text(json.dumps(out,indent=2))
