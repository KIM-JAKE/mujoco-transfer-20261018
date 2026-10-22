"""Slow the initial, contact-sensitive small-ring lift, preserving other paths."""
from pathlib import Path
import numpy as np,json,sys
from scipy.interpolate import PchipInterpolator
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1])]
from isaac_motion import Audit
p=HERE/'bin-picking';z=np.load(p/'under30-concurrent-coupled-prefix.npz');v={k:z[k].copy()for k in z.files};t=v['time_s'];r=v['right_source_time_s'];dt=1/480
# The left clock is slowed only in this brief common interval. Source curves,
# scene geometry and part dynamics otherwise remain unchanged.
a=np.searchsorted(r,37.55);b=np.searchsorted(r,38.5);x=t[a:b+1];u=(x-x[0])/(x[-1]-x[0]);stretch=1+3*np.sin(np.pi*u)**2
new=t.copy();new[a:b+1]=t[a]+np.r_[0,np.cumsum(np.diff(x)*(stretch[:-1]+stretch[1:])/2)];new[b+1:]+=new[b]-t[b]
nt=np.arange(int(np.ceil(new[-1]/dt))+1)*dt;clock=PchipInterpolator(new,t)(np.minimum(nt,new[-1]));q=np.column_stack([np.interp(clock,t,z['command'][:,j])for j in range(18)]).astype(np.float32)
for key in ['clamp','source_time_s','right_source_time_s']:
 arr=z[key];v[key]=np.interp(clock,t,arr)if arr.ndim==1 else np.column_stack([np.interp(clock,t,arr[:,j])for j in range(arr.shape[1])])
v['command']=q;v['time_s']=nt
# Apply the extra preload only once the ring is above the basket floor.
r=v['right_source_time_s'];l=v['source_time_s'];start=np.searchsorted(r,38.35);end=np.searchsorted(r,39.8);endtime=nt[end];fade=np.clip((nt-nt[start])/.15,0,1);old=np.clip((nt-endtime)/.15,0,1);depth=.0009*(fade*fade*(3-2*fade)-old*old*(3-2*old))*(r<51.63)
for j in [16,17]:q[:,j]-=depth
for k in range(1,len(q)):q[k,14:]=q[k-1,14:]+np.clip(q[k,14:]-q[k-1,14:],-.047*dt,.047*dt)
names=json.load(open(HERE.parents[1]/'runs/native-roas-table-bin-under30-capture01/pipeline-results.json'))['dof_names'];ai={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']};audit=Audit(q[:1],ai,dt)
for row in q[1:]:audit.update(row[None,:])
out=p/'under30-concurrent-soft-lift.npz';np.savez_compressed(out,**v);print('SOFT_LIFT',float(nt[-1]),audit.report())
o=json.load(open(p/'pipeline-options.json'));o.update(motion_file='bin-picking/'+out.name,donor_blend_s=1.8);(p/'pipeline-options.json').write_text(json.dumps(o,indent=2))
