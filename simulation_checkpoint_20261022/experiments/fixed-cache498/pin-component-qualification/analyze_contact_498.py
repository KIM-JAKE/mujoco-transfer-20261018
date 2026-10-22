"""POST-RUN evaluator ONLY; no imports/outputs are consumed by runtime."""
import json,numpy as np,collections,hashlib
from pathlib import Path
from scipy.spatial.transform import Rotation
R=Path('/home/gpuadmin/workspace/jake/mujoco');r=R/'.local/warm-worker-gpu2/snapshot/isaac_port/runs/native-roas-table-fixed-front-v498-pin-contact-component';p=json.load(open(r/'pipeline-results.json'));records=p['pin_contact_evaluator_only']['records'];z=np.load(r/'physx-sequence-trace.npz');pi=z['part_names'].tolist().index('pin');names=p['dof_names'];ids=[names.index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)];model=json.load(open(r/'robot-kinematics.json'));cache={}
def context(k):
 if k not in cache:
  H=np.array(model['base'])
  for val,ax,(pre,post)in zip(z['q'][k,ids],model['axes'],model['frames']):
   X=np.eye(4);X[:3,:3]=Rotation.from_rotvec(np.array(ax)*val).as_matrix();H=H@pre@X@post
  H=H@model['tip'];part=z['parts'][k,pi];P=part[:3]+Rotation.from_quat(part[3:]).as_matrix()@np.array([0.,-.00025,0.]);cache[k]=(H,P)
 return cache[k]
phase_rows=collections.defaultdict(list)
for rec in records:
 k=int(np.argmin(abs(z['simulation_time_s']-rec['physics_s'])));H,P=context(k);phase=str(z['pin_phase'][k]);force=0.;torque=np.zeros(3);active=[];ny=[]
 for pt in rec['contacts']:
  impulse=np.asarray(pt.get('impulse',[0,0,0]));n=np.asarray(pt.get('normal',[0,0,0]));f=abs(float(impulse@n))/rec['substep_dt_s'];force+=f
  if f>1e-3:
   active.append(f);ny.append(abs(float((H[:3,:3].T@n)[1])));pos=np.asarray(pt['position']);torque+=H[:3,:3].T@np.cross(pos-P,impulse/rec['substep_dt_s'])
 phase_rows[phase].append(dict(normal_force_n=force,nonzero_normal_points=len(active),effective_normal_points=float(sum(active)**2/sum(v*v for v in active))if active else 0.,closingY_normal_alignment=float(np.average(ny,weights=active))if active else 0.,absolute_normal_impulse_torque_hand_Nm=abs(torque)))
out=[]
for phase,rr in phase_rows.items():
 row=dict(phase=phase,records=len(rr))
 for key in ['normal_force_n','nonzero_normal_points','effective_normal_points','closingY_normal_alignment']:
  values=np.asarray([a[key]for a in rr]);row[key]=dict(q10=float(np.quantile(values,.1)),median=float(np.median(values)),q90=float(np.quantile(values,.9)),max=float(values.max()))
 row['normal_only_torque_median_abs_handXYZ_Nm']=np.median([a['absolute_normal_impulse_torque_hand_Nm']for a in rr],axis=0).tolist();out.append(row)
result=dict(EVALUATION_ONLY=True,used_for_control=False,commands_or_steps=0,run=r.name,full_task_pass=p['full_task_pass'],pin_only_component=p['pin_only_component'],state=p['state'],wall_s=p['wall_verified_elapsed_s'],telemetry_stats={k:v for k,v in p['pin_contact_evaluator_only'].items()if k!='records'},input_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest()for f in[r/'pipeline-results.json',r/'physx-sequence-trace.npz']},method='Match callbackphysics_s to nearest trace simulation_time; publicq FK for Hand axes; evaluator-only bodyposition for torque reference. Per-manifold force is normalimpulse/substep, not motor torque/force command.',limitations=['Contact impulse stream may expose normal impulse only: no inference of friction/torsional torque absence.','Peak contact reaction may exceed30N evenwhen actuator effortcapremains30N; this is not a motorcapaudit.','Multiple substeps share controlstep timestamp; per-record statistics, not paired simultaneousnetwrench.','Component starts originalrings stock/emptyarms; not fulltask or afterringscontext.'],phases=out)
Path('/tmp/fixed498-pin-component/actual498-contact-EVALUATION-ONLY.json').write_text(json.dumps(result,indent=2))
for x in out:print(x['phase'],x['records'],'Nmedian',round(x['normal_force_n']['median'],2),'points',x['nonzero_normal_points']['median'],'normalY',round(x['closingY_normal_alignment']['median'],6),'tauXYZ',np.round(x['normal_only_torque_median_abs_handXYZ_Nm'],6).tolist())
