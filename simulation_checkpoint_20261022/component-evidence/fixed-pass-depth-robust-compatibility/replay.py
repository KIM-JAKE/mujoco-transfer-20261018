from pathlib import Path
import sys,json,time,hashlib,numpy as np
root=Path('/home/gpuadmin/workspace/jake/mujoco');runs=root/'isaac_port/runs';model=next(runs.glob('*v514-*'));sys.path[:0]=[str(model/'code_snapshot'),str(root)];from rgbd_fast_tracker import FastRGBDTracker
from isaac_port.fieldkit.perception import CadPoseEstimator
tracker=FastRGBDTracker(CadPoseEstimator());results=[]
for number in [510,512,513]:
 run=next(runs.glob(f'*v{number}-*'));report=json.load(open(run/'pipeline-results.json'));pin=report['scattered_pick']['pin'];events=pin['events'];full=next(e for e in events if e['event']=='pin_full_depth_RGBD');pre=next(e for e in reversed(events)if e['event']=='pin_preinsert_RGBD'and e['wall_s']<full['wall_s']);obs=json.load(open(run/'rgb-observations.json'));a=min(obs,key=lambda x:abs(x['delivered_wall_s']-pre['wall_s']));b=min(obs,key=lambda x:abs(x['delivered_wall_s']-full['wall_s']));assert abs(a['delivered_wall_s']-pre['wall_s'])<1e-7 and abs(b['delivered_wall_s']-full['wall_s'])<1e-7;A=np.asarray(a['hand_TCP_world']);B=np.asarray(b['hand_TCP_world']);P=np.asarray(a['result']['pose']['center_m']);axis=np.asarray(a['result']['pose']['axis']);seed=B[:3,:3]@A[:3,:3].T@(P-A[:3,3])+B[:3,3];ax=B[:3,:3]@A[:3,:3].T@axis;file=run/'rgb-perception'/Path(b['work']).name/'rgbd.npz';z=np.load(file);packet={k:z[k]for k in z.files};row=dict(run=number,source_fullpass=report['full_task_pass'],observation=Path(b['work']).name,recorded_pose=b['result']['pose'],public_seed=seed.tolist(),public_axis=ax.tolist(),packet_SHA256=hashlib.sha256(file.read_bytes()).hexdigest(),replay={})
 for robust in [False,True]:
  start=time.perf_counter()
  try:p,m=tracker.estimate(packet,'pin',seed,ax,robust_pin_cap_circle=robust);row['replay'][str(robust)]=dict(accepted=True,pose=p,recorded_center_difference_m=float(np.linalg.norm(np.asarray(p['center_m'])-b['result']['pose']['center_m'])))
  except Exception as e:row['replay'][str(robust)]=dict(accepted=False,error=type(e).__name__+': '+str(e))
  row['replay'][str(robust)]['CPU_component_s']=time.perf_counter()-start
 results.append(row);print(json.dumps(dict(run=number,observation=row['observation'],old=row['replay']['False'],robust=row['replay']['True'])),flush=True)
Path('/tmp/fixed-pass-depth-robust-compatibility/results.json').write_text(json.dumps(dict(public_pixels_and_SDK_only=True,no_GT_used=True,no_GPU_jobs=True,no_runtime_changes=True,rows=results),indent=2)+'\n')
