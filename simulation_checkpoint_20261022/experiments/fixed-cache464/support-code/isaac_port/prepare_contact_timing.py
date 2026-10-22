"""Create a separate joint/feedback reference with collision-interlock waits."""
import json,hashlib,argparse
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation,Slerp
from prepare import HERE,SOURCES
parser=argparse.ArgumentParser();parser.add_argument('--fast',action='store_true');parser.add_argument('--output',type=Path);args=parser.parse_args()
if args.fast:
 from task_timing_fast import source_time,event_time,TOTAL,LEFT_WAIT,RIGHT_WAIT,LEFT_RELEASED,RATE,BREAKPOINTS,TIMING_PARAMETERS
else:
 from task_timing import source_time,event_time,TOTAL,LEFT_WAIT,RIGHT_WAIT,LEFT_RELEASED
 RATE=1.;BREAKPOINTS=[LEFT_WAIT,RIGHT_WAIT,LEFT_RELEASED];TIMING_PARAMETERS={}
original=SOURCES['cap_button'].parent.parent/'cap_button_parallel_alt_20261016'
out=args.output or HERE/('generated/contact_task/fast'if args.fast else'generated/contact_task/retimed');out.mkdir(exist_ok=True,parents=True)
src=np.load(original/'trajectory.npz');model=mujoco.MjModel.from_xml_path(str(SOURCES['cap_button']))
ts=np.unique(np.r_[np.arange(0,TOTAL,1/60),BREAKPOINTS,TOTAL]);ts=ts[np.r_[True,np.diff(ts)>1e-8]];q=np.zeros((len(ts),model.nq));mp=np.zeros((len(ts),3,3));mq=np.zeros((len(ts),3,4))
clocks={side:np.array([source_time(side,t)for t in ts])for side in ['left','right']}
for i in range(model.njnt):
 joint=model.joint(i);side=joint.name.split('_')[0];adr=int(joint.qposadr[0]);q[:,adr]=np.interp(clocks[side],src['time_s'],src['qpos'][:,adr])
for i,side in enumerate(['right','left','left']):
 for k in range(3):mp[:,i,k]=np.interp(clocks[side],src['time_s'],src['mocap_pos'][:,i,k])
 r=Slerp(src['time_s'],Rotation.from_quat(src['mocap_quat'][:,i][:,[1,2,3,0]]))(clocks[side]);mq[:,i]=r.as_quat()[:,[3,0,1,2]]
# Physical pickup: keep the jaw open during approach and close only after
# the arm reaches its final grasp pose. Object targets remain unchanged.
if args.fast:
 begin=4.586333333333333;end=4.736333333333333;approach=3.394333333333333;driver=int(model.joint('left_grip_left_driver_joint').qposadr[0])
 arm=[int(model.joint('left_arm_fr3v2_joint'+str(j)).qposadr[0])for j in range(1,8)]
 arm_start=np.array([np.interp(begin,src['time_s'],src['qpos'][:,j])for j in arm]);arm_end=np.array([np.interp(end,src['time_s'],src['qpos'][:,j])for j in arm]);closed=float(np.interp(end,src['time_s'],src['qpos'][:,driver]))
 keys,indices=np.unique(src['qpos'][:,driver],return_index=True)
 grip=[int(model.joint(j).qposadr[0])for j in range(model.njnt)if model.joint(j).name.startswith('left_grip_')]
 for i,t in enumerate(clocks['left']):
  if not approach<=t<=end:continue
  if t<begin:
   u=float(np.clip((t-(begin-.3))/.3,0,1));q[i,arm]+=(arm_end-arm_start)*u*u*(3-2*u);angle=min(q[i,driver],.55)
  else:
   q[i,arm]=arm_end;u=float(np.clip((t-begin)/(end-begin),0,1));angle=.55+(closed-.55)*u*u*(3-2*u)
  for j in grip:q[i,j]=np.interp(angle,keys,src['qpos'][indices,j])
 TIMING_PARAMETERS=TIMING_PARAMETERS|dict(pin_approach_driver_angle_cap_rad=.55,arm_stationary_during_pin_closure=True)

# Open the real jaws fully before withdrawing from the installed rings.
# Keep the empty pin hand open and stationary during the button operation.
if args.fast:
 for side in ['left','right']:
  driver=int(model.joint(side+'_grip_left_driver_joint').qposadr[0]);keys,idx=np.unique(src['qpos'][:,driver],return_index=True);keys=np.r_[0.,keys]
  grip=[int(model.joint(j).qposadr[0])for j in range(model.njnt)if model.joint(j).name.startswith(side+'_grip_')]
  values={j:np.r_[0.,src['qpos'][idx,j]]for j in grip}
  begin,end=(3.127666666666667,3.244333333333333)if side=='left'else(5.633666666666667,5.750333333333333)
  closed=float(np.interp(begin,src['time_s'],src['qpos'][:,driver]))
  for i,t in enumerate(clocks[side]):
   angle=None
   if begin<=t<=end:angle=closed+(.01-closed)*(t-begin)/(end-begin)
   elif end<t<(3.394333333333333 if side=='left'else 6.3):angle=.01
   elif side=='left'and 3.394333333333333<=t<3.594333333333333:
    u=(t-3.394333333333333)/.2;angle=.01+(q[i,driver]-.01)*u*u*(3-2*u)
   elif side=='right'and 6.3<=t<6.6:
    u=(t-6.3)/.3;angle=.01+(q[i,driver]-.01)*u*u*(3-2*u)
   if side=='left'and 7.153<=t:
    u=float(np.clip((t-7.153)/.15,0,1));angle=float(np.interp(7.153,src['time_s'],src['qpos'][:,driver]))*(1-u)+.59*u
   if angle is not None:
    for j in grip:q[i,j]=np.interp(angle,keys,values[j])
 TIMING_PARAMETERS=TIMING_PARAMETERS|dict(ring_release_driver_angle_rad=.01,pin_release_driver_angle_rad=.59)

np.savez_compressed(out/'trajectory.npz',time_s=ts,qpos=q,mocap_pos=mp,mocap_quat=mq)
schedule=json.loads((original/'independent_schedule.json').read_text())
for side,lane in schedule['lanes'].items():
 delay=schedule['delays'][side]
 for row in lane:
  row['start_s']=event_time(side,row['start_s']+delay,start=True);row['end_s']=event_time(side,row['end_s']+delay)
timing_fields=dict(clock_module='task_timing_fast'if args.fast else'task_timing',timing_parameters=TIMING_PARAMETERS,reference_total_s=TOTAL,left_pickup_wait_s=[event_time('left',LEFT_WAIT),event_time('left',LEFT_WAIT,start=True)],right_button_wait_s=[event_time('right',RIGHT_WAIT),event_time('right',RIGHT_WAIT,start=True)])
schedule['delays']={'left':0,'right':0};schedule['retiming']=timing_fields
(out/'independent_schedule.json').write_text(json.dumps(schedule,indent=2)+'\n');(out/'config.json').write_bytes((original/'config.json').read_bytes())
(out/'retiming.json').write_text(json.dumps(dict(timing_parameters=TIMING_PARAMETERS,clock_module='task_timing_fast'if args.fast else'task_timing',original=str(original),original_sha256=hashlib.sha256((original/'trajectory.npz').read_bytes()).hexdigest(),total_s=TOTAL,samples=len(ts),wait_intervals_s={'left':timing_fields['left_pickup_wait_s'],'right':timing_fields['right_button_wait_s']},semantics='Joint references and object feedback targets only; no measured object states changed'),indent=2)+'\n')
print('RETIMED_REFERENCE',len(ts),TOTAL)
