"""Compare conservative fast clearance with original live PhysX queries.

Geometry-only regression; never executes or counts as a task success.
"""
from pathlib import Path
import json,time,numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from fcl_transport import FastClearSession

def execute(c):
 h=Path(__file__).resolve().parent;r=h.parents[1]/'runs'/c['config']['fast_clear_probe_source_run'];report=json.loads((r/'pipeline-results.json').read_text());a=np.load(r/'executed-actuator-commands.npz');observations=json.loads((r/'rgb-observations.json').read_text());names=report['dof_names'];ids={s:[names.index(s+'_arm_fr3v2_joint'+str(j))for j in range(1,8)]for s in ['left','right']};fi={s:[names.index(s+'_native_'+label)for label in ['left','right']]for s in ['left','right']};arms={'left':c['left_arm'],'right':c['arm']};rng=np.random.default_rng(3124);samples=[]
 for s in ['left','right']:
  for k in np.linspace(0,len(a['command'])-1,24).astype(int):samples.append((s,k,None,False))
 for phase,s,part in [('ring2_carry_to_handover','right','ring2'),('pin_tail_direct_carry_to_preinsert','right','pin')]:
  selected=np.flatnonzero(a['pin_phase']==phase)
  for k in selected[np.linspace(0,len(selected)-1,12).astype(int)]:samples.extend([(s,k,part,False),(s,k,part,True)])
 started=time.perf_counter();accepted=0;rejected=0;fallback=0;disagreements=[];counts={}
 for side,k,part,perturb in samples:
  other='left'if side=='right'else'right';checker=c['checkers'][side];oc=c['checkers'][other];q=a['command'][k,ids[side]].astype(float);oq=a['command'][k,ids[other]].astype(float);width=float(a['actual_dof_position'][k,fi[side]].sum());ow=float(a['actual_dof_position'][k,fi[other]].sum());checker.use_fast_clear_queries=False
  if perturb:q=np.clip(q+rng.normal(0,.2,7),arms[side].bounds[:,0],arms[side].bounds[:,1])
  if part:
   G=np.asarray(report['scattered_pick']['pin']['measured_grasp'])if part=='pin'else np.asarray(next(p for p in report['scattered_pick']['parts']if p['part']==part)['pickup']['measured_grasp']);checker.attach_payload(c['pin_context']['paths']['workpiece_pin']if part=='pin'else c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
  try:
   with checker.other_arm_at(oc,oq,ow):
    session=FastClearSession(checker.fast_planner,width);fast=session.clear(q);slow,why=checker.clear(q,width)
   accepted+=int(fast);rejected+=int(not slow);fallback+=int(not fast and slow)
   if fast and not slow:disagreements.append(dict(side=side,index=int(k),part=part,perturbed=perturb,collision=why))
  finally:checker.detach_payload()
 result=dict(state='static_fast_clear_regression_only',full_task_pass=False,source=r.name,samples=len(samples),fast_clear=accepted,PhysX_blocked=rejected,conservative_fallbacks=fallback,false_clear_vs_original=len(disagreements),disagreements=disagreements,passed=not disagreements,elapsed_s=time.perf_counter()-started,uses_free_body_poses=False,clamp_geometry='Full legal prismatic stroke envelope plus 2 mm; native CAD enlarged by 0.5 mm; original predictive pair gate retained')
 print('FAST_CLEAR_REGRESSION',json.dumps(result),flush=True);return result
