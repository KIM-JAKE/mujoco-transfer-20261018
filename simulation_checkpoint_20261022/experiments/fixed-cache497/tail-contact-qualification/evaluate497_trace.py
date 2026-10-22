"""POST-RUN EVALUATOR ONLY. NEVER imported or read by controller."""
import numpy as np,json,hashlib
from pathlib import Path
from scipy.spatial.transform import Rotation
r=Path('/home/gpuadmin/workspace/jake/mujoco/.local/warm-worker-gpu2/snapshot/isaac_port/runs/native-roas-table-fixed-front-v497-tail-flat-contact-minus');obs=json.loads((r/'rgb-observations.json').read_text());tr=np.load(r/'physx-sequence-trace.npz');j=tr['part_names'].tolist().index('pin');rows=[]
unit=lambda a:np.asarray(a)/np.linalg.norm(a)
angle=lambda a,b:float(np.arccos(np.clip(unit(a)@unit(b),-1,1)))
for o in obs:
 if o.get('part')!='pin'or not o.get('result',{}).get('pose'):continue
 p=o['result']['pose'];t=o['captured_wall_s'];k=int(np.argmin(abs(tr['time_s']-t)));body=tr['parts'][k,j];R=Rotation.from_quat(body[3:]).as_matrix();P=body[:3]+R@np.array([0,-.00025,0]);n=R[:,2]
 if n@p['axis']<0:n=-n
 H=np.asarray(o['hand_TCP_world']);Gpos=(np.linalg.inv(H)@np.r_[P,1.])[:3];Gaxis=H[:3,:3].T@n;publicGpos=(np.linalg.inv(H)@np.r_[p['center_m'],1.])[:3];publicGaxis=H[:3,:3].T@p['axis'];row=dict(work=Path(o['work']).name,mode=o['mode'],captured_wall_s=t,trace_phase=str(tr['pin_phase'][k]),trace_time_delta_ms=float((tr['time_s'][k]-t)*1000),center_error_um=float(np.linalg.norm(P-p['center_m'])*1e6),axis_error_mrad=angle(n,p['axis'])*1000,physical_axis=n.tolist(),public_axis=p['axis'],evaluated_G_center_m=Gpos.tolist(),evaluated_G_axis=Gaxis.tolist(),public_G_center_m=publicGpos.tolist(),public_G_axis=publicGaxis.tolist());rows.append(row)
held=[x for x in rows if x['mode']=='held'];a,b=held[-2:];conclusion=dict(preload_to_postcarry_physical_G_center_mm=float(np.linalg.norm(np.asarray(a['evaluated_G_center_m'])-b['evaluated_G_center_m'])*1000),preload_to_postcarry_physical_G_axis_mrad=angle(a['evaluated_G_axis'],b['evaluated_G_axis'])*1000,preload_to_postcarry_public_G_center_mm=float(np.linalg.norm(np.asarray(a['public_G_center_m'])-b['public_G_center_m'])*1000),preload_to_postcarry_public_G_axis_mrad=angle(a['public_G_axis'],b['public_G_axis'])*1000,maximum_held_axis_estimation_error_mrad=max(x['axis_error_mrad']for x in held),maximum_held_center_estimation_error_um=max(x['center_error_um']for x in held))
result=dict(EVALUATION_ONLY=True,used_for_control=False,controller_imports=False,physical_commands_issued=0,physics_steps=0,method='Nearest timestamp trace bodyposition+R@[0,-.25mm,0]; quaternionxyzw; axis signed to observed cap; samecapture publicHand SDK',input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in[r/'rgb-observations.json',r/'physx-sequence-trace.npz']},rows=rows,conclusion=conclusion);Path('/tmp/fixed497-pin-tail-flat/actual497-EVALUATION-ONLY-trace.json').write_text(json.dumps(result,indent=2));print(json.dumps(conclusion,indent=2))
for x in held:print(x['work'],x['trace_phase'],round(x['trace_time_delta_ms'],3),round(x['center_error_um'],3),round(x['axis_error_mrad'],3))
