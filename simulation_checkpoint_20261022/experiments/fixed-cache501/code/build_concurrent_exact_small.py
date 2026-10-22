"""Retime two lanes independently; synchronize actual handover dependencies."""
from pathlib import Path
import json,numpy as np,sys
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1])]
from retime_recorded_motion import prepare
from isaac_motion import Audit
OUT=HERE/'bin-picking/concurrent-exact-small';OUT.mkdir(exist_ok=True)
ROOT=HERE.parents[1]/'runs/native-roas-table-bin-under30-capture01';z=np.load(ROOT/'full-command-trace.npz');names=json.load(open(ROOT/'pipeline-results.json'))['dof_names'];t=z['time_s'];dt=1/480
ai={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']};fi={s:[i for i,n in enumerate(names)if n.startswith(s+'_native')]for s in ai}
barriers=dict(left=[25.,27.30,30.14,49.20,51.62,54.70,72.5],right=[20.95,27.30,30.14,46.4896,51.62,54.85,75.76])
lanes={};arrivals={};effective={}
for side in ai:
 out=OUT/(side+'-retimed01.npz')
 if not out.exists():
  mask=(t>=6.3)&(t<=(72.5021 if side=='left'else t[-1]));ts=t[mask];q=z['command'][mask].copy();clamp=z['clamp'][mask].copy();other='right'if side=='left'else'left';q[:,ai[other]+fi[other]]=q[0,ai[other]+fi[other]]
  clamp[:,2 if side=='left'else slice(0,2)]=0
  phases=np.array([str(x).split(' || ')[0 if side=='left'else -1]for x in z['phase'][mask]]);cuts=np.r_[0,np.flatnonzero(phases[1:]!=phases[:-1])+1,len(phases)]
  for a,b in zip(cuts[:-1],cuts[1:]):
   if not any(x in str(phases[a])for x in ['grasp_hold','close_hold','recipient_retention','giver_release_hold'])and np.ptp(q[a:b,ai[side]+fi[side]],axis=0).max()<1e-5 and np.ptp(clamp[a:b],axis=0).max()<1e-5:phases[a:b]='initial'
  phases=np.array([p+' || initial'if side=='left'else'initial || '+p for p in phases])
  if side=='left':
   ts=np.r_[ts,ts[-1]+np.arange(1,481)*dt];q=np.concatenate([q,np.repeat(q[-1:],480,axis=0)]);clamp=np.concatenate([clamp,np.repeat(clamp[-1:],480,axis=0)]);phases=np.r_[phases,np.repeat('installed_hold || initial',480)]
  src=OUT/(side+'-source01.npz');np.savez_compressed(src,time_s=ts,command=q,clamp=clamp,phase=phases)
  points=[('barrier'+str(i),point)for i,point in enumerate(barriers[side])if point<ts[-1]-.01]
  prepare(src,out,names,method='filtered-lp',stops_override=points)
 lane0=np.load(out);lane={key:lane0[key].copy()for key in lane0.files}
 verified=np.load(HERE/'bin-picking/under30-hybrid-fix02.npz');first=float(verified['normalization_source_time_s'][0]);k=int(np.searchsorted(verified['source_time_s'],first))+48;prefix=verified['command'][:k+1].copy();point=float(lane['normalization_source_time_s'][0]);hits=np.flatnonzero(abs(lane['source_time_s']-point)<1e-8);j=int(hits[len(hits)//2]);anchor=lane['command'][j].copy();inds=ai[side];n=96;a=len(prefix)-1-n;T=n*dt;orig=prefix.copy()
 M=np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,2,0,0,0],[1,T,T*T,T**3,T**4,T**5],[0,1,2*T,3*T*T,4*T**3,5*T**4],[0,0,2,6*T,12*T*T,20*T**3]])
 coef=np.linalg.solve(M,np.array([orig[a,inds],(orig[a,inds]-orig[a-1,inds])/dt,(orig[a,inds]-2*orig[a-1,inds]+orig[a-2,inds])/dt**2,anchor[inds],np.zeros(7),np.zeros(7)]));x=np.arange(n+1)*dt;prefix[a:,inds]=np.column_stack([x**m for m in range(6)])@coef
 pad=48
 lane['command']=np.concatenate([prefix,np.repeat(anchor[None],pad,axis=0),lane['command'][j+1:]])
 preclock=np.minimum(verified['source_time_s'][:k+1],point)if side=='right'else np.linspace(6.3,point,k+1)
 lane['source_time_s']=np.r_[preclock,np.repeat(point,pad),lane['source_time_s'][j+1:]]
 lane['clamp']=np.concatenate([verified['clamp'][:k+1],np.repeat(lane['clamp'][j:j+1],pad,axis=0),lane['clamp'][j+1:]])
 lane['time_s']=np.arange(len(lane['command']))*dt
 if side=='right':
  ref=np.load(HERE/'bin-picking/under30-hybrid-transport01.npz');rs=ref['right_source_time_s'];a=int(np.searchsorted(lane['source_time_s'],36.9));b=int(np.searchsorted(lane['source_time_s'],41.4));ra=int(np.searchsorted(rs,36.9));rb=int(np.searchsorted(rs,41.4));clip=ref['command'][ra:rb+1].copy();st=rs[ra:rb+1].copy();inds=ai['right'];x=np.arange(len(clip))*dt;T=x[-1];ease=lambda u:np.clip(u,0,1)**3*(10-15*np.clip(u,0,1)+6*np.clip(u,0,1)**2)
  d0=lane['command'][a,inds]-clip[0,inds];d1=lane['command'][b,inds]-clip[-1,inds];clip[:,inds]+=(1-ease(x/.15))[:,None]*d0+ease((x-T+.15)/.15)[:,None]*d1
  for j in fi['right']:clip[:,j]=np.interp(st,t,z['command'][:,j])
  lane['command']=np.concatenate([lane['command'][:a],clip,lane['command'][b+1:]])
  lane['clamp']=np.concatenate([lane['clamp'][:a],ref['clamp'][ra:rb+1],lane['clamp'][b+1:]])
  lane['source_time_s']=np.r_[lane['source_time_s'][:a],st,lane['source_time_s'][b+1:]];lane['time_s']=np.arange(len(lane['command']))*dt
 lanes[side]=lane;arr=[];pts=[]
 for i,point in enumerate(barriers[side]):
  if 'barrier'+str(i)in lane['normalization_parts']:
   ix=lane['normalization_parts'].tolist().index('barrier'+str(i));point=float(lane['normalization_source_time_s'][ix]);matches=np.flatnonzero(abs(lane['source_time_s']-point)<1e-8);k=matches[len(matches)//2]
  else:k=int(np.argmin(abs(lane['source_time_s']-point)));point=float(lane['source_time_s'][k])
  arr.append(float(lane['time_s'][k]));pts.append(point)
 arrivals[side]=arr+[float(lane['time_s'][-1])];effective[side]=pts
# Each faster lane waits at the barrier while the other completes its work.
clock={s:[]for s in ai};timeline=[];start={s:0. for s in ai};wall=0.
for b in range(len(barriers['left'])+1):
 length=max(arrivals[s][b]-start[s]for s in ai);N=int(np.ceil(length/dt));local=np.arange(N)*dt
 for s in ai:clock[s].extend(np.minimum(start[s]+local,arrivals[s][b]))
 timeline.extend(wall+local);wall+=N*dt
 for s in ai:start[s]=arrivals[s][b]
for s in ai:clock[s].append(float(lanes[s]['time_s'][-1]))
timeline=np.r_[timeline,wall];cmd=np.zeros((len(timeline),18),np.float32);controls=np.zeros((len(timeline),3));source={}
for s in ai:
 lane=lanes[s];ct=np.array(clock[s]);inds=ai[s]+fi[s]
 for j in inds:cmd[:,j]=np.interp(ct,lane['time_s'],lane['command'][:,j])
 for j in ([0,1]if s=='left'else[2]):controls[:,j]=np.interp(ct,lane['time_s'],lane['clamp'][:,j])
 source[s]=np.interp(ct,lane['time_s'],lane['source_time_s'])
# Additional preload only after the small ring has cleared the basket.
r=source['right'];a=np.searchsorted(r,39.8);u=np.clip((timeline-timeline[a])/.15,0,1);depth=.0009*u*u*(3-2*u)*(r<51.63)
for j in fi['right']:cmd[:,j]-=depth
l=source['left']
for j in fi['left']:cmd[:,j]-=.001*np.clip((.04-cmd[:,j])/.003,0,1)*((l>25)&(l<70))
for k in range(1,len(cmd)):
 for j in fi['left']+fi['right']:cmd[k,j]=cmd[k-1,j]+np.clip(cmd[k,j]-cmd[k-1,j],-.047*dt,.047*dt)
audit=Audit(cmd[:1],ai,dt)
for row in cmd[1:]:audit.update(row[None,:])
out=HERE/'bin-picking/under30-concurrent-exact-small.npz';np.savez_compressed(out,time_s=timeline,command=cmd,clamp=controls,source_time_s=source['left'],right_source_time_s=source['right'],normalization_parts=['ring1','ring2'],normalization_source_time_s=[effective['right'][0],effective['right'][3]])
report=dict(planned_s=float(timeline[-1]),physical_pass=False,barriers_source_s=effective,lane_arrival_times_s=arrivals,command_limit_audit=audit.report());out.with_suffix('.json').write_text(json.dumps(report,indent=2));print('CONCURRENT_CANDIDATE',report,flush=True)
p=HERE/'bin-picking/pipeline-options.json';o=json.load(open(p));o.update(motion_file='bin-picking/'+out.name,smooth_donor_blend=True,donor_blend_s=1.2,normalization_refine=True);p.write_text(json.dumps(o,indent=2))
