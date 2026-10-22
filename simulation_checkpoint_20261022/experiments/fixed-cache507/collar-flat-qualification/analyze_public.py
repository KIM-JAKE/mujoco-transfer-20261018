from pathlib import Path
import numpy as np,json,sys
from scipy.spatial.transform import Rotation
v=sys.argv[1] if len(sys.argv)>1 else'461';p=next(Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs').glob('*v'+v+'*'));report=json.load(open(p/'pipeline-results.json'));pin=report.get('scattered_pick',{}).get('pin',report.get('pin_skill_progress',{}));r=json.load(open(p/'robot-kinematics.json'));d=np.load(p/'executed-actuator-commands.npz');names=report['dof_names'];ids=[names.index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)]
def fk(q):
 T=np.array(r['base'])
 for x,a,(pre,post)in zip(q,r['axes'],r['frames']):
  X=np.eye(4);X[:3,:3]=Rotation.from_rotvec(np.asarray(a)*x).as_matrix();T=T@np.array(pre)@X@np.array(post)
 return T@np.array(r['tip'])
obs=json.load(open(p/'rgb-observations.json'));start=min(float(d['time_s'][i])for i,ph in enumerate(d['pin_phase'])if str(ph)=='pin_precision_encoder_settle');end=min(float(d['time_s'][i])for i,ph in enumerate(d['pin_phase'])if str(ph)in ['pin_single_axial_insert'])if any(str(ph)=='pin_single_axial_insert'for ph in d['pin_phase'])else float('inf')
rows=[];goal=np.array(pin.get('physical_axis_midpoint_goal_world_m',[-.3019,.2272,1.5285]))
for o in obs:
 if o.get('part')!='pin' or not start<=o['captured_wall_s']<end or not o['result'].get('pose'):continue
 t=o['captured_wall_s'];k=np.searchsorted(d['time_s'],t,side='right')-1;A=np.array(o['hand_TCP_world']);C=fk(d['command'][k,ids]);Ac=fk(d['actual_dof_position'][k,ids]);P=np.array(o['result']['pose']['center_m']);axis=o['result']['pose']['axis'];G=np.linalg.inv(A)@np.r_[P,1.];err=P-goal;row=dict(t=t,C=C.tolist(),A=A.tolist(),P=P.tolist(),axis=axis,G_translation=G[:3].tolist(),error=err.tolist(),command_actual_dxy=(C[:2,3]-A[:2,3]).tolist(),FK_public_error_m=float(np.linalg.norm(A-Ac)),q=d['command'][k,ids].tolist());rows.append(row)
print('start/end',start,end,'goal',goal)
for i,a in enumerate(rows):
 dC=np.array(a['C'])[:2,3]-np.array(rows[i-1]['C'])[:2,3]if i else np.zeros(2);dA=np.array(a['A'])[:2,3]-np.array(rows[i-1]['A'])[:2,3]if i else np.zeros(2);dP=np.array(a['P'])[:2]-np.array(rows[i-1]['P'])[:2]if i else np.zeros(2)
 print(i,round(a['t'],2),'errµm',np.round(np.array(a['error'])[:2]*1e6,1),'Cµm',np.round(np.array(a['C'])[:2,3]*1e6,1),'Aµm',np.round(np.array(a['A'])[:2,3]*1e6,1),'dC',np.round(dC*1e6,1),'dA',np.round(dA*1e6,1),'dP',np.round(dP*1e6,1),'FKerr',a['FK_public_error_m'])
Path('/tmp/pin465-analysis/sequence'+v+'.json').write_text(json.dumps(dict(source=p.name,scope='Only recorded actuator commands, public FK and RGB-D estimates; no free actor poses',goal=goal.tolist(),rows=rows),indent=2)+'\n')
