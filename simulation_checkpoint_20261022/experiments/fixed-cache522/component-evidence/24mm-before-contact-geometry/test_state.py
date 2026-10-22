from pathlib import Path
import ast,textwrap,types,sys,json,numpy as np
from scipy.spatial.transform import Rotation
O=Path(__file__).parent;s=(O/'overrides/direct_pin_skill.py').read_text();a=s.index('    adaptive_pin_gain=');b=s.index('    wait(.12,\'pin_full_depth_hold\')',a);block=textwrap.dedent(s[a:b]);tests=[]
class Blocked(RuntimeError):pass
def frame(P,R):T=np.eye(4);T[:3,:3]=R;T[:3,3]=P;return T
def exp(v):return Rotation.from_rotvec(v).as_matrix()
def axis_frame(a):return np.eye(3)
def run(poses,enabled=True,fail=None,badterminal=False,badhold=False):
 poses=list(poses);cmd=np.zeros((1,18),np.float32);cmd[0,0]=1.455;cmd[0,7:9]=.006;goal=np.array([0.,0.,1.5]);log=[];report={'events':[]};observed=[];assessed=[];corrections=[]
 class Arm:
  def fk(self,q):return frame([q[1],q[2],q[0]],np.eye(3))
 arm=Arm();art=types.SimpleNamespace(get_dof_positions=lambda:cmd.copy());ctx={'config':{},'cmd':cmd,'art':art}
 def observe(label,G,require_circle):
  i=len(observed);depth,rad=poses[min(i,len(poses)-1)];observed.append(label);P=goal-[0.,0.,depth];P[0]+=rad;return {'H':arm.fk(cmd[0,:7]),'result':{'pose':{'center_m':P,'axis':[0,0,1]}}}
 def assess(ctx,G,d,**kw):assessed.append((d,kw));return (not(badterminal and kw.get('entry_stage')),{'test':True})
 def rebranch(*args,**kwargs):raise RuntimeError('original_rebranch_unavailable')
 module=types.ModuleType('pin_current_terminal');module.assess=assess;module.plan_rebranch=rebranch;sys.modules['pin_current_terminal']=module
 def line(H0,H1,T,label):
  log.append((label,float(T),H0.copy(),H1.copy()))
  if fail==label:raise RuntimeError('partial_row_fault')
  cmd[0,0]=H1[2,3];cmd[0,1]=H1[0,3];cmd[0,2]=H1[1,3]
 def visual(C,A,G,goal,axis,z,**kwargs):
  corrections.append(kwargs);T=C.copy();T[2,3]=min(z,goal[2]-kwargs.get('preinsert_depth_m',.045));return T,{}
 class Checker:
  payload_path=None;payload_rows=None;payload_boxes=None;payload_relative=None
  def attach_payload(self,*args):pass
 env=dict(np=np,arm=arm,art=art,cmd=cmd,ids=np.arange(7),fi=[7,8],DT=1/480,pin_config={'pin_visual_adaptive_gain':True,'pin_visual_independent_component_feedback':True,'pin_visual_bounded_component_recovery':True,'pin_visual_servo_gain':.65,'pin_terminal_stroke_preflight':True,'pin_precision_component_deadband':True},visual_preinsert=True,entry_feedback_enabled=enabled,pin_sensor_hold=True,report=report,G=np.eye(4),R=np.eye(3),goal=goal,pin_hold_reference_G=np.eye(4),observe=observe,measured_grasp=lambda v,G:G.copy(),_pin_rgbd_sdk_hold_evidence=lambda *a:{'confirmed':not(badhold and len(observed)>1)},print=lambda *a,**kw:None,frame=frame,exp=exp,axis_frame=axis_frame,ctx=ctx,PinCartesianPreflightBlocked=Blocked,line=line,wait=lambda *a:None,step=lambda:None,started=0.,time=types.SimpleNamespace(perf_counter=lambda:0.),_pin_visual_preinsert_target=visual,_pin_bounded_xy_gain=lambda gain,*a:(gain,0),checker=Checker(),paths={'workpiece_pin':'/pin'},payload_grasp=lambda:np.eye(4),qualified_full_depth=True)
 error=None
 try:exec(block,env)
 except RuntimeError as e:error=str(e)
 return dict(log=log,observed=observed,assessed=assessed,corrections=corrections,error=error,env=env)
r=run([(.045,0),(.024,0)]);assert not r['error']and [x[0]for x in r['log']]==['pin_axial_move_to_entry','pin_single_axial_insert'];assert r['observed']==['pin_preinsert_RGBD','pin_entry_RGBD'];assert r['assessed'][0][1]=={'partial_entry':True};opts=r['assessed'][1][1];assert opts['entry_stage']and abs(opts['requested_s']-(.8*.024/.045))<1e-12 and abs(opts['withdraw_H'][2,3]-1.455)<1e-6;assert abs(sum(x[1]for x in r['log'])-.8)<1e-12;assert abs(r['env']['pre'][2,3]-1.455)<1e-6;tests.append('far_fullproof_exactpartial_then_freshentry_fullproof_finalstroke_farwithdraw')
r=run([(.045,0),(.024,.0003),(.024,0)]);assert not r['error']and len(r['observed'])==3 and r['corrections'][0]['preinsert_depth_m']==.024;tests.append('entry_servo_uses_explicit24plane_same_precision_controller')
r=run([(.045,0)],enabled=False);assert not r['error']and len(r['log'])==1 and r['log'][0][0]=='pin_single_axial_insert'and r['assessed'][0][1]=={};tests.append('flagoff_original_single45stroke')
for label in('pin_axial_move_to_entry','pin_single_axial_insert'):
 r=run([(.045,0),(.024,0)],fail=label);assert r['error']=='partial_row_fault'and r['log'][-1][0]==label;tests.append('no_retry_after_'+label+'_partialfault')
r=run([(.045,0),(.024,0)],badterminal=True);assert r['error']=='original_rebranch_unavailable'and len(r['log'])==1;tests.append('entry_original_terminal_block_prevents_finalstroke')
r=run([(.045,0),(.024,0)],badhold=True);assert r['error']and len(r['log'])==1;tests.append('entry_original_holdgate_failure_prevents_finalstroke')
r=run([(.045,0),(.030,0)]);assert '22–28' in r['error']and len(r['log'])==1;tests.append('explicit_entry_depth_gate_preserved')
r=run([(.045,.0004)]*15+[(.045,0)]);assert 'budget exhausted' in r['error']and len(r['observed'])==16 and not any(x[0]=='pin_axial_move_to_entry'for x in r['log']);tests.append('no_budget_left_far_pass_blocks_partial_beforemotion')
r=run([(.045,0)]+[(.024,.0004)]*15);assert 'did not converge' in r['error']and len(r['observed'])==16 and not any(x[0]=='pin_single_axial_insert'for x in r['log']);tests.append('entry_consumes_original_total16_budget_noreset')
(O/'state-tests.json').write_text(json.dumps(dict(passed=True,tests=tests,no_physical_trials=True),indent=2)+'\n');print(tests)
