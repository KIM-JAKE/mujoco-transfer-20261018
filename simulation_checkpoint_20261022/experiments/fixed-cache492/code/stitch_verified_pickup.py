"""Reuse the tested first pickup motor targets, then execute the faster tail."""
from pathlib import Path
import numpy as np,json,sys
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1])]
from isaac_motion import Audit
src=np.load(HERE/'bin-picking/under30-motion07.npz');dst=np.load(HERE/'bin-picking/under30-ready04.npz');dt=1/480
names=json.load(open(HERE.parents[1]/'runs/native-roas-table-bin-under30-capture01/pipeline-results.json'))['dof_names'];ai={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']};left=[i for i,n in enumerate(names)if n.startswith('left_')]
a=np.searchsorted(src['source_time_s'],0.);b=np.searchsorted(src['source_time_s'],20.95);j=np.searchsorted(dst['source_time_s'],20.95)
prefix=src['command'][a:b+1].copy();lf=np.linspace(21.4145833333,25.0020833333,len(prefix))
for i in left:prefix[:,i]=np.interp(lf,src['source_time_s'],src['command'][:,i])
tail=dst['command'][j:].copy();before=prefix[-1].copy();v0=(prefix[-1].astype(float)-prefix[-2])/dt;a0=(prefix[-1].astype(float)-2*prefix[-2]+prefix[-3])/dt**2
v1=(tail[1].astype(float)-tail[0])/dt;a1=(tail[2].astype(float)-2*tail[1]+tail[0])/dt**2
T=.4;matrix=np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,2,0,0,0],[1,T,T*T,T**3,T**4,T**5],[0,1,2*T,3*T*T,4*T**3,5*T**4],[0,0,2,6*T,12*T*T,20*T**3]])
coef=np.linalg.solve(matrix,np.array([before-tail[0],v0-v1,a0-a1,np.zeros(18),np.zeros(18),np.zeros(18)]));N=min(len(tail),int(T/dt)+1);t=np.arange(N)*dt;tail[:N]+=np.array([t**p for p in range(6)]).T@coef
pad=0;command=np.concatenate([np.repeat(prefix[:1],pad,axis=0),prefix,tail[1:]]).astype(np.float32);source=np.r_[np.repeat(src['source_time_s'][a],pad),src['source_time_s'][a:b+1],dst['source_time_s'][j+1:]];clamp=np.concatenate([np.repeat(src['clamp'][a:a+1],pad,axis=0),src['clamp'][a:b+1],dst['clamp'][j+1:]]);right=np.r_[source[:pad+len(prefix)],dst['right_source_time_s'][j+1:]]
audit=Audit(command[:1],ai,dt)
for row in command[1:]:audit.update(row[None,:])
output=HERE/'bin-picking/under30-ready08.npz';np.savez_compressed(output,time_s=np.arange(len(command))*dt,command=command,clamp=clamp,source_time_s=source,right_source_time_s=right,normalization_parts=dst['normalization_parts'],normalization_source_time_s=dst['normalization_source_time_s']);report=dict(physical_cycle_pass=False,duration_s=(len(command)-1)*dt,verified_first_pickup_commands_only=True,command_limit_audit=audit.report());output.with_suffix('.json').write_text(json.dumps(report,indent=2));print(report)
