import ast,copy,json,sys,types,contextlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
s=Path('/tmp/fixed493-small-balanced/overrides/loose_bin_task.py').read_text();tree=ast.parse(s)
feedback=next(n for n in ast.walk(tree)if isinstance(n,ast.If)and ast.unparse(n.test).startswith('small_balanced_feedback or'))
future=next(n for n in ast.walk(tree)if isinstance(n,ast.If)and ast.unparse(n.test)=='small_balanced_feedback'and any(isinstance(x,ast.Constant)and x.value=='balanced_SMALL_corrected_preclose_future_opening_admitted'for x in ast.walk(n)))
code=compile(ast.fix_missing_locations(ast.Module(body=[feedback,future],type_ignores=[])),'actual493_feedback','exec')
def frame(t,R):
 H=np.eye(4);H[:3,3]=t;H[:3,:3]=R;return H
class Arm:
 def fk(self,q):return np.eye(4)
 def solve(self,p,R,q):return np.asarray(q).copy()
class Checker:
 def __init__(self,ev):self.payload_path='old';self.payload_rows=['old'];self.payload_boxes=['old'];self.payload_relative=np.eye(4);self.resolution_rad=.035;self.ev=ev;self.same_arm_clearance=self
 def attach_payload(self,p,G):self.payload_path='new';self.ev.append('attach')
 @contextlib.contextmanager
 def other_arm_at(self,*a):yield
 def path(self,*a):self.ev.append('preliminary_proof');return self.path_ok,'blocked'
 def clear(self,*a):self.ev.append('native_proof');return self.native_ok,'blocked'
def case(kind):
 ev=[];positions=np.zeros((1,18));cmd=positions.copy();cmd[0,14:]=.04;positions[0,14:]=.04
 rc=Checker(ev);lc=Checker(ev);lc.path_ok=kind!='path_block';lc.native_ok=kind!='native_block';intended=np.eye(4);pre=frame([.0005 if kind!='bound'else .003,0,0],np.eye(3))
 def move(*args,**kw):
  assert ev[-1]=='preliminary_proof';assert kw['other_carried_part']=='ring2'and np.array_equal(kw['other_carried_grasp'],np.eye(4));ev.append('move')
  if kind=='move_fault':raise RuntimeError('partial step fault')
 def observe(*a):
  ev.append('fresh_image');return np.eye(4),dict(center_m=[.0045 if kind=='disturbance'else .0005,0,0],axis=[0,0,1])
 def sequential(*a,**kw):ev.append('future_opening');return kind!='opening_block'
 ns=dict(np=np,c=dict(config={},left_hand_known_empty=True,part_collision_templates={'ring2':'class'},planning_diagnostics=[]),small_balanced_feedback=True,part='ring2',arms={'left':Arm(),'right':Arm()},indices={'left':np.arange(7),'right':np.arange(7,14)},fingers={'right':np.array([14,15]),'left':np.array([16,17])},art=types.SimpleNamespace(get_dof_positions=lambda:positions),cmd=cmd,preclose_part=pre,intended_receiver_grasp=intended,G=np.eye(4),checkers={'right':rc,'left':lc},receiver_opening=.08,release_width=.08,frame=frame,axis_frame=lambda a:np.eye(3),exp=lambda r:Rotation.from_rotvec(r).as_matrix(),log=lambda R:Rotation.from_matrix(R).as_rotvec(),move=move,observe=observe,transfer=types.SimpleNamespace(receiver_aperture=lambda *a:.062,sequential_transfer_clear=sequential_transfer_clear if False else sequential))
 error=None
 try:exec(code,ns)
 except RuntimeError as e:error=str(e)
 assert lc.resolution_rad==.035 and rc.payload_path=='old'and lc.payload_path=='old'
 assert np.array_equal(intended,np.eye(4))
 if kind=='pass':assert error is None and ev.count('move')==1 and ev.count('fresh_image')==1 and ev.count('native_proof')==2 and ev.count('future_opening')==24
 elif kind in ['bound','path_block']:assert error and 'move'not in ev and 'fresh_image'not in ev
 elif kind=='move_fault':assert error=='partial step fault'and ev.count('move')==1 and 'fresh_image'not in ev
 elif kind in ['disturbance','native_block','opening_block']:assert error and ev.count('move')==1 and ev.count('fresh_image')==1
 return dict(case=kind,pass_=True,error=error,events=ev)
results=[case(k)for k in ['pass','bound','path_block','move_fault','disturbance','native_block','opening_block']]
# The original full-issued guard implementation remains literally AST-identical.
base=ast.parse(Path('/tmp/fixed490-width-and-placed-full/overrides/loose_bin_task.py').read_text())
getmove=lambda t:next(n for n in ast.walk(t)if isinstance(n,ast.FunctionDef)and n.name=='move')
assert ast.dump(getmove(base),include_attributes=False)==ast.dump(getmove(tree),include_attributes=False)
Path('/tmp/fixed493-small-balanced/lifecycle-tests.json').write_text(json.dumps(dict(all_pass=True,original_full_move_issued_guard_AST_unchanged=True,tests=results),indent=2));print('PASS 7 lifecycle cases; original full move AST unchanged')
