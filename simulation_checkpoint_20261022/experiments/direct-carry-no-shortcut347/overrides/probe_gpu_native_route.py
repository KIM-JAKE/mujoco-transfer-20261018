"""No-motion direct carry diagnosis from actual341 RGBD and encoders only."""
from pathlib import Path
import json,time,numpy as np
from contextlib import contextmanager
from isaac_kinematics import frame
from probe_native_right_ring import fixture_clear
from grasp_transfer_feasibility import hinted_route

def execute(c):
 out=c['vision'].out;source=out.parent/'native-roas-table-scatter-stable-v341-final-empty-retreat-all3-446';r=json.loads((source/'pipeline-results.json').read_text());obs=json.loads((source/'rgb-observations.json').read_text());data=np.load(source/'executed-actuator-commands.npz');names=r['dof_names'];ids={s:[names.index(s+'_arm_fr3v2_joint'+str(i))for i in range(1,8)]for s in ['left','right']};fi={s:[i for i,n in enumerate(names)if n.startswith(s+'_native_')]for s in ids};actual=data['actual_dof_position'][-1].astype(float);command=data['command'][-1].astype(float)
 diag=next(v for v in r['planning_diagnostics']if v['event']=='right_ring_fixture_search');G=np.asarray(diag['current_measured_grasp']);width=float(diag['checked_width_m']);rc,lc=c['checkers']['right'],c['checkers']['left'];offset=frame([0,0,.1034],np.eye(3));c['cmd'][0]=command
 # Same RGB-D stock inputs and known-Hand surface exclusion, actual encoders.
 stocks=[]
 for i,o in enumerate(obs[:3]):
  pkt=dict(np.load(source/'rgb-perception'/('observation-%03d-initial'%(i+1))/'rgbd.npz'))
  k=max(0,int(np.searchsorted(data['time_s'],o['captured_wall_s'])-1));w=float(data['actual_dof_position'][k,fi['right']].sum());points=c['observed_grasp_clearance'].points(pkt,c['calibration']);stocks.append(c['observed_grasp_clearance'].remove_known_hand_surface(points,np.asarray(o['hand_TCP_world']),w))
 rc.gpu_rgbd_stock_points=np.concatenate(stocks)
 @contextmanager
 def loaded():
  saved=rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative
  rc.attach_payload(c['part_collision_templates']['ring1'],offset@G)
  try:
   with rc.other_arm_at(lc,actual[ids['left']],float(actual[fi['left']].sum())):yield
  finally:rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved
 callbacks=[]
 def carry(q,g,w):
  start=time.perf_counter()
  with loaded():
   proposal=hinted_route(dict(c,config=dict(c['config'],shortcut_checked_routes=False)),'ring1_direct_right_carry',rc,command[ids['right']],q,w,seconds=3.)
   why=None;ok=False
   if proposal is not None:ok,why=rc.path(np.vstack([actual[ids['right']],np.asarray(proposal)]),w)
  callbacks.append(dict(elapsed_s=time.perf_counter()-start,proposal_knots=0 if proposal is None else len(proposal),original_actual_to_proposed_polyline_clear=bool(ok),first_rejection=why,goal_q=q.tolist(),verified_cubic=bool(getattr(proposal,'verified_cubic',False))))
  print('DIRECT347_CARRY',callbacks[-1],flush=True)
  return proposal if ok else None
 search=dict(c,_right_direct_current_heading_first=False,_right_direct_minimum_joint_margin_rad=0.,_right_direct_measured_q=actual[ids['right']],_right_direct_fixture_context_key=('recorded341',),_right_direct_fixture_cache={},_right_direct_fixture_counts={},_right_direct_fixture_examples={},_right_direct_carry_route_callback=carry)
 began=time.perf_counter()
 with rc.other_arm_at(lc,actual[ids['left']],float(actual[fi['left']].sum())):approved=fixture_clear(search,'ring1',G,width,deadline=began+180.)
 result=dict(state='recorded341_direct_carry_diagnosis',robot_motion_executed=False,hardware_tested=False,full_task_pass=False,uses_free_body_poses=False,source=source.name,source_policy='Original fullfixed/self/payload/pair checks; SAME current RGBD grasp/stock and encoder endpoints. Optional greedy shortcut search disabled, not any guard.',approved=approved,callbacks=callbacks,counts=search['_right_direct_fixture_counts'],examples=search['_right_direct_fixture_examples'],elapsed_s=time.perf_counter()-began)
 (out/'direct347-carry-diagnosis.json').write_text(json.dumps(result,indent=2));print('DIRECT347_RESULT',{k:v for k,v in result.items()if k not in ['approved','callbacks','examples']},flush=True);return result
