"""Static candidate search using saved image-relative grasp and encoders.

No task execution or simulator-body pose writes. Not a physical success.
"""
from pathlib import Path
import types,json,time,numpy as np
from fr3_path_timing import retime,retime_safe_polyline

def execute(c):
 h=Path(__file__).resolve().parent;r=h.parents[1]/'runs'/c['config']['handover_probe_source_run'];report=json.loads((r/'pipeline-results.json').read_text());part=next(p for p in report['scattered_pick']['parts']if p['part']=='ring2');G=np.array(part['pickup']['measured_grasp']);a=np.load(r/'executed-actuator-commands.npz');k=int(np.abs(a['time_s']-part['pickup']['elapsed_s']).argmin());cmd=a['command'][k:k+1].copy();cmd[0,c['left_ids']]=c['cmd'][0,c['left_ids']];actual=a['actual_dof_position'][k:k+1].copy();actual[0,c['left_ids']]=cmd[0,c['left_ids']]
 class RobotSnapshot:
  def get_dof_positions(self):return actual
 c=dict(c,cmd=cmd,art=RobotSnapshot(),config=dict(c['config'],prefer_near_handover_branch=True,sort_donor_orientations=True,handover_plan_budget_s=3.,warm_spline_routes=False));p=h/'paired_handover.py';planner=types.ModuleType('paired_handover');planner.__file__=str(p);exec(compile(p.read_text(),str(p),'exec'),planner.__dict__);results=[]
 def duration(qs,width):
  q=np.asarray(qs);start=np.array([width/2,width/2])
  if len(q)>2 and not getattr(qs,'verified_cubic',False):return retime_safe_polyline(q,1/480,start,start)[2]['duration_s']
  return retime(q,.05,1/480,start,start,optimize=True)[2]['duration_s']
 for offset in [[.1,-.1,0.],[.2,-.1,0.],[.3,-.15,0.],[.2,-.2,0.],[.2,-.15,.05],[.3,-.15,.05],[.1,-.05,.05],[.15,-.15,.1]]:
  c['config']['handover_offsets_override']=[offset];started=time.perf_counter()
  try:
   plan=planner.plan(c,'ring2',G);rd=duration(plan['donor_route'],float(actual[0,c['fi']].sum()));ld=duration(plan['receiver_route'],.068);row=dict(offset=offset,found=True,donor_s=rd,receiver_s=ld,serial_motion_s=rd+ld,right_q=plan['donor_route'][-1].tolist(),left_q=plan['receiver_q'].tolist(),receiver_parameters=[float(plan[x])for x in ['receiver_tilt_rad','receiver_yaw_rad']],incoming=plan['incoming'].tolist())
  except RuntimeError as error:row=dict(offset=offset,found=False,error=str(error))
  row['search_s']=time.perf_counter()-started;results.append(row);print('HANDOVER_CANDIDATE',json.dumps(row),flush=True)
 return dict(state='handover_timing_geometry_probe_only',full_task_pass=False,source=r.name,uses_free_body_poses=False,inputs='Saved RGB-D relative grasp, robot encoders and fixed CAD',candidates=results,best=min((v for v in results if v['found']),key=lambda v:v['serial_motion_s'],default=None))
