"""Retime two lanes independently; synchronize actual handover dependencies."""
from pathlib import Path
import json,numpy as np,sys
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1])]
from retime_recorded_motion import prepare
from isaac_motion import Audit
OUT=HERE/'bin-picking/concurrent-OD-safe';OUT.mkdir(exist_ok=True)
ROOT=HERE.parents[1]/'runs/native-roas-table-bin-under30-capture01';z=np.load(HERE/'bin-picking/od-pipeline-source.npz');names=json.load(open(ROOT/'pipeline-results.json'))['dof_names'];t=z['time_s'];dt=1/480
ai={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']};fi={s:[i for i,n in enumerate(names)if n.startswith(s+'_native')]for s in ai}
barriers=dict(left=[25.,27.30,30.14,49.20,51.62,54.70,73.0],right=[20.95,27.30,30.14,46.4896,51.62,54.85,75.76])
lanes={};arrivals={};effective={}
for side in ai:
 out=OUT/(side+'-retimed01.npz')
 if not out.exists():
  mask=(t>=6.3)&(t<=(73.0021 if side=='left'else t[-1]));ts=t[mask];q=z['command'][mask].copy();clamp=z['clamp'][mask].copy();other='right'if side=='left'else'left';q[:,ai[other]+fi[other]]=q[0,ai[other]+fi[other]]
  clamp[:,2 if side=='left'else slice(0,2)]=0
  phases=np.array([str(x).split(' || ')[0 if side=='left'else -1]for x in z['phase'][mask]]);cuts=np.r_[0,np.flatnonzero(phases[1:]!=phases[:-1])+1,len(phases)]
  for a,b in zip(cuts[:-1],cuts[1:]):
   if not any(x in str(phases[a])for x in ['grasp_hold','close_hold','recipient_retention','partial_retention','giver_release_hold'])and np.ptp(q[a:b,ai[side]+fi[side]],axis=0).max()<1e-5 and np.ptp(clamp[a:b],axis=0).max()<1e-5:phases[a:b]='initial'
  phases=np.array([p+' || initial'if side=='left'else'initial || '+p for p in phases])
  if side=='left':
   ts=np.r_[ts,ts[-1]+np.arange(1,481)*dt];q=np.concatenate([q,np.repeat(q[-1:],480,axis=0)]);clamp=np.concatenate([clamp,np.repeat(clamp[-1:],480,axis=0)]);phases=np.r_[phases,np.repeat('installed_hold || initial',480)]
  src=OUT/(side+'-source01.npz');np.savez_compressed(src,time_s=ts,command=q,clamp=clamp,phase=phases)
  points=[('barrier'+str(i),point)for i,point in enumerate(barriers[side])if point<ts[-1]-.01]
  prepare(src,out,names,method='filtered-lp',stops_override=points,loaded_right_accel=8.6,stop_hold_s=.14)
 lane_file=np.load(out);lane={key:lane_file[key].copy()for key in lane_file.files}
 if side=='right':
  from scipy.ndimage import minimum_filter1d,gaussian_filter1d
  from scipy.integrate import cumulative_trapezoid
  old_t=lane['time_s'].copy();finger_v=abs(np.gradient(lane['command'][:,fi['right']],1/480,axis=0)).max(axis=1)
  speed_cap=np.minimum(1.075,.0467/np.maximum(finger_v,1e-10));speed_cap=gaussian_filter1d(minimum_filter1d(speed_cap,size=61,mode='nearest'),sigma=10)
  pin_region=(lane['source_time_s']>=75.55)&(lane['source_time_s']<=84.30);pin_factor=np.where(pin_region,.94,1.);speed_cap*=gaussian_filter1d(minimum_filter1d(pin_factor,size=31,mode='nearest'),sigma=6);warped=np.r_[0,cumulative_trapezoid(1/speed_cap,old_t)];new_t=np.arange(int(np.ceil(warped[-1]*480))+1)/480;clock=np.interp(new_t,warped,old_t)
  for key in ['command','clamp','source_time_s']:
   values=lane[key];lane[key]=np.interp(clock,old_t,values)if values.ndim==1 else np.column_stack([np.interp(clock,old_t,values[:,j])for j in range(values.shape[1])])
  lane['command']=lane['command'].astype(np.float32);lane['time_s']=new_t
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
 delay={s:(.30 if s=='left'and b in [3,6]else 0.)for s in ai};length=max(arrivals[s][b]-start[s]+delay[s]for s in ai);N=int(np.ceil(length/dt));local=np.arange(N)*dt
 for s in ai:clock[s].extend(np.minimum(start[s]+np.maximum(local-delay[s],0),arrivals[s][b]))
 timeline.extend(wall+local);wall+=N*dt
 for s in ai:start[s]=arrivals[s][b]
for s in ai:clock[s].append(float(lanes[s]['time_s'][-1]))
timeline=np.r_[timeline,wall];cmd=np.zeros((len(timeline),18),np.float32);controls=np.zeros((len(timeline),3));source={}
for s in ai:
 lane=lanes[s];ct=np.array(clock[s]);inds=ai[s]+fi[s]
 for j in inds:cmd[:,j]=np.interp(ct,lane['time_s'],lane['command'][:,j])
 for j in ([0,1]if s=='left'else[2]):controls[:,j]=np.interp(ct,lane['time_s'],lane['clamp'][:,j])
 source[s]=np.interp(ct,lane['time_s'],lane['source_time_s'])
l=source['left']
strength=np.ones(len(l))
for begin,end in [(35.542,37.042),(63.265,64.765)]:
 u=np.clip((l-begin)/(end-begin),0,1);fade=u**3*(10-15*u+6*u*u);strength[(l>=begin)&(l<(49.2 if begin<40 else 70))]=1-fade[(l>=begin)&(l<(49.2 if begin<40 else 70))]
for j in fi['left']:
 original=cmd[:,j].copy();preload=(.001+.002*strength)*np.clip((.0385-original)/.002,0,1)*((l>25)&(l<70));cmd[:,j]=np.minimum(.04,original-preload+.001*np.clip((original-.0375)/.0015,0,1))
for k in range(1,len(cmd)):
 for j in fi['left']+fi['right']:cmd[k,j]=cmd[k-1,j]+np.clip(cmd[k,j]-cmd[k-1,j],-.047*dt,.047*dt)
audit=Audit(cmd[:1],ai,dt)
for row in cmd[1:]:audit.update(row[None,:])
out=HERE/'bin-picking/under30-OD-pin-motion-stable.npz';np.savez_compressed(out,time_s=timeline,command=cmd,clamp=controls,source_time_s=source['left'],right_source_time_s=source['right'],normalization_parts=['ring1','ring2','pin'],normalization_source_time_s=[effective['right'][0],effective['right'][3],effective['right'][6]])
report=dict(planned_s=float(timeline[-1]),physical_pass=False,barriers_source_s=effective,lane_arrival_times_s=arrivals,command_limit_audit=audit.report());out.with_suffix('.json').write_text(json.dumps(report,indent=2));print('CONCURRENT_CANDIDATE',report,flush=True)
p=HERE/'bin-picking/pipeline-options.json';o=json.load(open(p));o.update(motion_file='bin-picking/'+out.name,smooth_donor_blend=True,donor_blend_s=1.2,normalization_refine=True,od_grasp_pipeline=True,effort_grasp_after_lift=False,small_ring_force_before_lift=False,mimic_natural_frequency=1000.);plan=json.load(open(HERE.parents[1]/'runs/native-roas-table-od-plan07/od-plan.json'));o['handover_offsets_m']={part:row['handover_offset_m']for part,row in plan['poses'].items()};o['donor_blend_s']=1.5;o['normalization_parts']=['ring1','ring2','pin'];o['pin_in_motion_alignment']=True;p.write_text(json.dumps(o,indent=2))
