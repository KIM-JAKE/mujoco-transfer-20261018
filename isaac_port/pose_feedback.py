"""Measured-object pose feedback. It commands FR3 motors, never workpiece poses."""
import json
import numpy as np
import mujoco
from task_path_offsets import pin_clearance_z
from scipy.spatial.transform import Rotation,Slerp

class PoseFeedback:
 def __init__(self,root,recording,names,speed):
  from source_model import load_model
  self.model=load_model(root);self.data=mujoco.MjData(self.model)
  self.names=names;self.qadr=np.array([self.model.joint(n).qposadr[0]for n in names]);self.speed=speed;self.motion_limited=False;self.tracking_data=mujoco.MjData(self.model)
  archive=np.load(recording/'trajectory.npz');self.source={key:archive[key].copy()for key in archive.files};archive.close();self.ts=self.source['time_s']
  if(recording/'retiming.json').exists():
   from importlib import import_module
   module=json.loads((recording/'retiming.json').read_text()).get('clock_module','task_timing');assert module in ['task_timing','task_timing_fast']
   self.clock=import_module(module).source_time
  else:self.clock=lambda side,t:t
  self.parts=['ring1','ring2','pin'];self.slerp=[Slerp(self.ts,Rotation.from_quat(self.source['mocap_quat'][:,i][:,[1,2,3,0]]))for i in range(3)]
  self.force_limits={'ring1':20.,'ring2':20.,'pin':50.};self.pin_insert_gain=.25
  self.button=None;self.button_hold=False;self.offset=np.zeros(len(names));self.previous=np.zeros(len(names));self.held=None
  pub=json.loads((recording/'independent_schedule.json').read_text());self.windows={};self.insertion={};self.arrivals={};self.grip_counts={};self.clear_counts={}
  self.pin_alignment_start=next(r['end_s']for r in pub['lanes']['left']if r['stage']['label']=='transfer_last_pin')+pub['delays']['left']
  self.button_interlock_reference=next(r['end_s']for r in pub['lanes']['left']if r['stage']['label']=='release_last_pin')+pub['delays']['left']
  for name,side,grasp,release in [('ring2','left','lift_two_parts_30mm','release_two_parts'),('ring1','right','lift_two_parts_30mm','release_two_parts'),('pin','left','lift_last_pin_30mm','release_last_pin')]:
   lane=pub['lanes'][side];delay=pub['delays'][side]
   begin=next(r['start_s'] for r in lane if r['stage']['label']==grasp)+delay
   end=next(r['start_s'] for r in lane if r['stage']['label']==release)+delay
   release_end=next(r['end_s'] for r in lane if r['stage']['label']==release)+delay
   self.windows[name]=(side,begin,end,release_end)
   grasp_label='grasp_last_pin'if name=='pin'else'grasp_two_parts'
   self.grip_counts[name]=0;self.clear_counts[name]=0
   self.arrivals[name]=(side,next(r['start_s']for r in lane if r['stage']['label']==grasp_label)+delay)
   approach='insert_last_pin_approach'if name=='pin'else'insert_two_parts_approach'
   self.insertion[name]=next(r['start_s'] for r in lane if r['stage']['label']==approach)+delay
 def target(self,name,t):
  i=self.parts.index(name);pos=np.array([np.interp(t,self.ts,self.source['mocap_pos'][:,i,k])for k in range(3)])
  if name=='pin':pos[2]+=pin_clearance_z(self.clock('left',t))
  return pos,self.slerp[i](t)
 def errors(self,name,t,positions,quaternions):
  i=self.parts.index(name);target,R=self.target(name,t);actual=Rotation.from_quat(quaternions[i,[1,2,3,0]])
  err=(R*actual.inv()).as_rotvec()
  if name!='pin':axis=R.apply([0,0,1]);err-=axis*np.dot(err,axis)
  return target-positions[i],err
 def gate(self,previous,proposed,positions,quaternions,forces,clamps=None):
  if isinstance(previous,dict):
   references={};holds={}
   for side in ['left','right']:
    self._gate_side=side
    references[side],holds[side]=self.gate(previous[side],proposed[side],positions,quaternions,forces,clamps)
   del self._gate_side
   released=all(previous[side]>=end-1e-7 and self.clear_counts[name]>=3 for name,(side,begin,seat,end)in self.windows.items())
   self.button_interlocked=not released
   gate=self.button_interlock_reference-1e-8
   if not released and previous['right']<=gate and proposed['right']>gate:
    references['right']=gate;holds['right']='button_wait_for_all_releases'
   return references,holds
  for name,(side,start,end,release_end)in self.windows.items():
   if getattr(self,'_gate_side',side)!=side:continue
   f=getattr(self,'pad_forces',{}).get(name,{})
   self.grip_counts[name]=self.grip_counts[name]+1 if all(f.get(jaw,0)>5 for jaw in ['left','right'])else 0
   gate=start-(1e-8 if self.motion_limited else .0001)
   if previous<=gate and proposed>gate and self.grip_counts[name]<3:return gate,name+'_grip_contact'
  for name,(side,start,end,release_end)in self.windows.items():
   if getattr(self,'_gate_side',side)!=side:continue
   f=getattr(self,'pad_forces',{}).get(name,{})
   self.clear_counts[name]=self.clear_counts[name]+1 if previous>=end and all(f.get(jaw,0)<.1 for jaw in ['left','right'])else 0
   gate=release_end-(1e-8 if self.motion_limited else .0001)
   if previous<=gate and proposed>gate:
    j=self.names.index(side+'_grip_left_driver_joint');desired=np.interp(gate,self.ts,self.planned_q[:,j])
    if self.clear_counts[name]<3 or abs(self.measured_q[j]-desired)>.01:return gate,name+'_release_clear'
  for name,(side,start)in self.arrivals.items():
   if getattr(self,'_gate_side',side)!=side:continue
   gate=start-(1e-8 if self.motion_limited else .0001)
   if previous<=gate and proposed>gate:
    indices=np.array([self.names.index(side+'_arm_fr3v2_joint'+str(i))for i in range(1,8)])
    target=np.array([np.interp(gate,self.ts,self.planned_q[:,i])for i in indices])+self.offset[indices]
    if np.max(abs(self.measured_q[indices]-target))>.0015 or np.max(abs(self.measured_qv[indices]))>.03:return gate,name+'_arrival'
  if self.motion_limited and 'pin'in self.insertion and getattr(self,'_gate_side','left')=='left':
   prealignment=getattr(self,'pin_prealignment_reference',None)
   if prealignment is not None:
    gate=prealignment-1e-8
    if previous<=gate and proposed>gate:
     pe,re=self.errors('pin',prealignment,positions,quaternions)
     if np.linalg.norm(pe[:2])>.00005 or abs(pe[2])>.0003 or np.linalg.norm(re)>.0015:return gate,'pin_clearance_alignment'
   gate=self.insertion['pin']-1e-8
   if previous<=gate and proposed>gate:
    pe,re=self.errors('pin',self.insertion['pin'],positions,quaternions)
    if np.linalg.norm(pe[:2])>.000025 or abs(pe[2])>.0003 or np.linalg.norm(re)>.0015:return gate,'pin_precision_alignment'
  for name,start in self.insertion.items():
   side=self.windows[name][0]
   if getattr(self,'_gate_side',side)!=side:continue
   if start<=previous<self.windows[name][2] and forces.get(name,0)>self.force_limits[name] and not(clamps and clamps[name]):
    seat_pe,seat_re=self.errors(name,self.windows[name][2],positions,quaternions)
    if np.linalg.norm(seat_pe)>=.00045 or np.linalg.norm(seat_re)>=.025:return previous,name+'_force'
  for name,(side,start,end,release_end) in self.windows.items():
   if getattr(self,'_gate_side',side)!=side:continue
   gate=end-(1e-8 if self.motion_limited else .0001)
   if previous<=gate and proposed>gate:
    pe,re=self.errors(name,end,positions,quaternions)
    tolerance=.00075 if clamps and clamps[name] else .00045
    if np.linalg.norm(pe)>tolerance or np.linalg.norm(re)>.025:return gate,name
    if clamps is not None and not clamps[name]:return gate,name+'_clamping'
  return proposed,None
 def compensated_errors(self,name,t,positions,quaternions,side):
  # Compare the grasp in the planned hand frame. Actuator following lag must
  # not be integrated again by the slower object-pose correction loop.
  i=self.parts.index(name);hand=self.model.body(side+'_grip_base').id
  actual_R=Rotation.from_matrix(self.data.xmat[hand].reshape(3,3));planned_R=Rotation.from_matrix(self.tracking_data.xmat[hand].reshape(3,3))
  delta=planned_R*actual_R.inv();predicted_position=self.tracking_data.xpos[hand]+delta.apply(positions[i]-self.data.xpos[hand])
  predicted_R=delta*Rotation.from_quat(quaternions[i,[1,2,3,0]])
  target,R=self.target(name,t);re=(R*predicted_R.inv()).as_rotvec()
  if name!='pin':axis=R.apply([0,0,1]);re-=axis*np.dot(re,axis)
  return target-predicted_position,re
 def update(self,t,actual,positions,quaternions,forces,dt=1/120):
  reference=t
  self.data.qpos[self.qadr]=actual;mujoco.mj_kinematics(self.model,self.data);mujoco.mj_comPos(self.model,self.data)
  if self.motion_limited:
   self.tracking_data.qpos[self.qadr]=np.array([np.interp(reference[self.names[j].split('_')[0]]if isinstance(reference,dict)else reference,self.ts,self.planned_q[:,j])for j in range(len(self.names))])+self.offset
   mujoco.mj_kinematics(self.model,self.tracking_data)
  before=self.offset.copy()
  for side in ['left','right']:
   t=reference[side]if isinstance(reference,dict)else reference
   indices=np.array([self.names.index(side+'_arm_fr3v2_joint'+str(i))for i in range(1,8)])
   active=next((name for name,(arm,start,end,release_end) in self.windows.items()if arm==side and start<=t<release_end),None)
   if active is None:
    if side=='right'and self.clock('right',t)>=7.4 and not getattr(self,'button_interlocked',False):
     if self.button is None:
      from button_contact_control import ButtonControl
      self.button=ButtonControl(self)
     # Latch only after the real plunger has stayed pressed for 100ms.
     # Transient contact peaks must not interrupt the press before completion.
     if getattr(self,'button_ready',False):self.button_hold=True
     if not self.button_hold:self.button.update(t,self.data,indices,self.offset)
    else:
     own_clock=self.clock(side,t)
     keep=(side=='right'and 5.7503333333<=own_clock<6.1)or(side=='left'and(3.2443333333<=own_clock<3.3943333333 or own_clock>=7.153))
     if not keep:self.offset[indices]*=np.exp(-dt/.35)
    continue
   if getattr(self,'clamped',{}).get(active,False):continue
   pe,re=self.errors(active,t,positions,quaternions);raw_pe=pe.copy();i=self.parts.index(active);seat_pe,seat_re=self.errors(active,self.windows[active][2],positions,quaternions)
   if self.motion_limited and not(active=='pin'and t>=self.pin_alignment_start):pe,re=self.compensated_errors(active,t,positions,quaternions,side)
   at_seat=t>=self.insertion[active]and np.linalg.norm(seat_pe)<.00045 and np.linalg.norm(seat_re)<.025
   terminal_hold=t>=self.windows[active][2]-2e-7
   prealignment_hold=active=='pin' and abs(t-getattr(self,'pin_prealignment_reference',-100.))<2e-7
   if getattr(self,'fast_cycle',False)and active=='pin'and t>=self.pin_alignment_start and not at_seat:
    # Track real transverse alignment without integrating vertical motor lag.
    compensated,_=self.compensated_errors(active,t,positions,quaternions,side);pe[2]=compensated[2]
   # Once the planned arm is stationary at the release boundary, following-lag
   # compensation must not hide the real remaining seating error. Clamp/release
   # still require their unchanged measured pose and contact conditions.
   if at_seat or terminal_hold:pe,re=seat_pe,seat_re
   if prealignment_hold:pe,re=self.errors(active,t,positions,quaternions)
   if np.linalg.norm(raw_pe)>.025:raise RuntimeError('Carried-part tracking error exceeds 25 mm: '+active)
   if t>=self.insertion[active]:
    pe=np.clip(pe,-.0002,.0002)
    if forces.get(active,0)>self.force_limits[active]and not at_seat:pe[2]=-min(.0001,(forces[active]-self.force_limits[active])*1e-6)
   jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp)
   bid=self.model.body(side+'_grip_base').id
   mujoco.mj_jac(self.model,self.data,jp,jr,np.asarray(positions[i],dtype=float),bid)
   dofs=np.array([self.model.joint(self.names[j]).dofadr[0]for j in indices])
   J=np.r_[jp[:,dofs],.15*jr[:,dofs]];e=np.r_[pe,.15*re]
   dq=J.T@np.linalg.solve(J@J.T+.005**2*np.eye(6),e)
   gain=self.pin_insert_gain if active=='pin'and self.clock('left',t)>=5.8 else .25
   if self.motion_limited:gain=(.12 if active=='pin'and t>=self.pin_alignment_start else .24 if t>=self.insertion[active]else .04)if getattr(self,'fast_cycle',False)else .04
   if prealignment_hold:gain=.08
   cap=(.00024 if getattr(self,'fast_cycle',False)else .00012)if self.motion_limited else .0015
   self.offset[indices]+=np.clip(gain*dq,-cap,cap)
   self.offset[indices]=np.clip(self.offset[indices],-.25,.25)
  self.velocity=(self.offset-before)/dt
  return self.offset,self.velocity
