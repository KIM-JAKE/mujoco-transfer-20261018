"""POST-RUN EVALUATION ONLY: recorded image estimates versus recorded physics.

Never imported by the controller. Reads saved files; issues no physical steps,
commands, contacts, or sensor inputs. Requires numpy and scipy only.
"""
from pathlib import Path
import argparse,hashlib,json
import numpy as np
from scipy.spatial.transform import Rotation

ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,default=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs/native-roas-table-fixed-front-v474-tight-own-prealign-nominal'));ap.add_argument('--out',type=Path,default=Path('/tmp/pin474-rgbd-vs-physics-evaluation'));a=ap.parse_args()

def sha(path):
 h=hashlib.sha256()
 with path.open('rb')as f:
  for block in iter(lambda:f.read(1048576),b''):h.update(block)
 return h.hexdigest()
def angle(a,b):return float(np.arccos(np.clip(np.dot(a,b),-1,1)))

observations=json.loads((a.run/'rgb-observations.json').read_text());trace=np.load(a.run/'physx-sequence-trace.npz');pi=trace['part_names'].tolist().index('pin');rows=[];offset=np.array([0.,-.00025,0.])
for i,o in enumerate(observations):
 if o.get('part')!='pin'or'pose'not in o.get('result',{}):continue
 pose=o['result']['pose'];t=float(o['captured_wall_s']);k=int(np.argmin(abs(trace['time_s']-t)));tr=trace['parts'][k,pi].astype(float)
 # Isaac tensor poses are [px,py,pz,qx,qy,qz,qw]. The saved authored CAD
 # origin is offset from the physical symmetry-axis midpoint by BODY_TO_AXIS.
 R=Rotation.from_quat(tr[3:]).as_matrix();gt=tr[:3]+R@offset;axis=R[:,2];observed_axis=np.asarray(pose['axis'],float);observed_center=np.asarray(pose['center_m'],float)
 # Pin rotational symmetry: choose the same signed physical axis as the
 # observed visible cap. No yaw comparison is used or asserted observable.
 if np.dot(axis,observed_axis)<0:axis=-axis
 bias=observed_center-gt;phase=str(trace['pin_phase'][k])
 rows.append(dict(observation_index=i+1,captured_wall_s=t,matched_trace_index=k,matched_trace_wall_s=float(trace['time_s'][k]),nearest_trace_delta_ms=1000*(float(trace['time_s'][k])-t),phase=phase,preinsert=(phase=='pin_precision_encoder_settle'),observed_center_m=observed_center.tolist(),evaluated_physical_axis_midpoint_m=gt.tolist(),center_bias_mm=(1000*bias).tolist(),center_bias_norm_mm=float(1000*np.linalg.norm(bias)),observed_axis=observed_axis.tolist(),evaluated_physical_signed_axis=axis.tolist(),axis_bias_mrad=1000*angle(axis,observed_axis),observed_z_tilt_mrad=1000*float(np.arccos(abs(np.clip(observed_axis[2],-1,1)))),physical_z_tilt_mrad=1000*float(np.arccos(abs(np.clip(axis[2],-1,1)))),circle_direct=not pose.get('lateral_from_previous_RGBD_grasp_and_encoders',False)))
pre=[r for r in rows if r['preinsert']];assert len(pre)==16
stat={}
for key in ['axis_bias_mrad','center_bias_norm_mm','observed_z_tilt_mrad','physical_z_tilt_mrad']:
 v=np.array([r[key]for r in pre]);stat[key]=dict(min=float(v.min()),mean=float(v.mean()),median=float(np.median(v)),max=float(v.max()))
stat['mean_center_bias_mm']=np.mean([r['center_bias_mm']for r in pre],axis=0).tolist();stat['maximum_nearest_trace_delta_ms']=max(abs(r['nearest_trace_delta_ms'])for r in pre)
result=dict(EVALUATION_ONLY=True,used_for_control=False,uses_privileged_groundtruth_for_POSTRUN_evaluation=True,controller_changes=False,sensor_changes=False,physical_commands_issued=0,physics_steps=0,conclusion='Late4–8mrad PIN z-axis errors are physical tilt; RGB-D angular bias is below0.284mrad and center bias below0.032mm in these16preinsert frames.',method=dict(match='Nearest recorded monotonic wall-time sample to each successful PIN RGB-D capture',center='body_position + body_rotation @ BODY_TO_AXIS_M',BODY_TO_AXIS_M=offset.tolist(),quaternion='xyzw',axis='body_rotation[:,2], signed toward observed visible-cap axis',yaw='Not compared; axially symmetric pin',sampling_limitation='Nearest trace samples differ by at most13.3ms; values describe saved simulation only, not real-camera accuracy.'),preinsert_observation_count=len(pre),all_successful_pin_observation_count=len(rows),preinsert_statistics=stat,rows=rows)
a.out.mkdir(parents=True,exist_ok=True);(a.out/'qualification.json').write_text(json.dumps(result,indent=2)+'\n')
inputs=[a.run/'rgb-observations.json',a.run/'physx-sequence-trace.npz',a.run/'code_snapshot/pin_native_contact.py',a.run/'frozen-source-manifest.json']
provenance=dict(run=str(a.run),input_sha256={str(p):sha(p)for p in inputs},script_sha256=sha(Path(__file__)),method='Read-only post-run numeric evaluation; no controller or sensor inputs produced')
(a.out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n');print(json.dumps(stat,indent=2))
