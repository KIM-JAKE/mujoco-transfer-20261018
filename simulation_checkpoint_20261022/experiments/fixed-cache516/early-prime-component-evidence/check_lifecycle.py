"""Controlled lifecycle check. Native collision engines are mocked, not qualified.

Runs literal production generator/priming/proof-key/controller logic. Tests that
only the timing of the non-actuating exposure changes; collision math is NOT
replaced or claimed verified by this fixture.
"""
from pathlib import Path
from contextlib import contextmanager
import ast,types,sys,json,hashlib,time
import numpy as np
BASE=Path('/tmp/fixed512-camera-priority-full/overrides');STAGE=Path(__file__).parent
text=(STAGE/'overrides/loose_bin_task.py').read_text();tree=ast.parse(text)
def find(tree,name):
 values=[n for n in ast.walk(tree)if isinstance(n,(ast.FunctionDef,ast.ClassDef))and n.name==name];assert len(values)==1,(name,len(values));return values[0]
def execute(nodes,env,filename='<literal-production>'):
 exec(compile(ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[])),filename,'exec'),env)
def module(name):
 value=types.ModuleType(name);sys.modules[name]=value;return value
# Exact original key implementation, unchanged.
keymod=module('exact_loaded_left_proof');exec(compile((BASE/'exact_loaded_left_proof.py').read_text(),str(BASE/'exact_loaded_left_proof.py'),'exec'),keymod.__dict__)
fixed=module('fixed_small_left_executor');fixed.__dict__['np']=np;fixed.__dict__['META']={'dt_s':1/480}
execute([find(ast.parse((BASE/'fixed_small_left_executor.py').read_text()),'SavedCommandRoute')],fixed.__dict__)
@contextmanager
def static_scope(*args):yield {}
fixed.static_scope=static_scope
pairmod=module('gpu_pair_path_scope')
@contextmanager
def certified_path_scope(*args):yield {}
pairmod.certified_path_scope=certified_path_scope
# Pure production frame function; importing the USD module is unnecessary here.
kin=ast.parse(Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/native_hand_layout/roas-table/isaac_kinematics.py').read_text())
env={'np':np,'contextmanager':contextmanager,'time':time};execute([find(kin,'frame')],env)
contactpath=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs/native-roas-table-fixed-front-v512-camera-priority-nominal/code_snapshot/loaded_contact_aperture.py')
contactmod=module('loaded_contact_aperture');exec(compile(contactpath.read_text(),str(contactpath),'exec'),contactmod.__dict__)
# Real production pairing state machine; only geometry _verify is observed by a
# fixture at its call boundary. No GPU, USD, physics or real arm is instantiated.
paired_tree=ast.parse((BASE/'paired_issued_overlap.py').read_text());env['PairedPickupReplanRequired']=type('PairedPickupReplanRequired',(RuntimeError,),{})
execute([find(paired_tree,'PairedIssuedOverlap')],env)
class Art:
 def __init__(self,q):self.q=q.copy()
 def get_dof_positions(self):return self.q.copy()
class Checker:
 def __init__(self,side):
  self.arm=types.SimpleNamespace(base=np.eye(4),tip=np.eye(4),frames=np.empty((0,2,4,4)),axes=np.empty((0,3)),bounds=np.array([[-3.,3.]]*7),hand_path=side+'_hand')
  self.pair_boxes=[];self.payload_boxes=[];self.payload_rows=[];self.payload_path=None;self.payload_relative=None;self.resolution_rad=.035;self.ignored_future_actors=set();self.loaded_contact_aperture=None
  self.same_arm_clearance=types.SimpleNamespace(margin_m=.0005,pairs=[],rows=[],geometries={},broadphase_enabled=True)
  self.placed_part_clearance=types.SimpleNamespace(generation=0,parts={})
 def detach_payload(self):self.payload_path=None;self.payload_rows=[];self.payload_boxes=[];self.payload_relative=None
 def attach_payload(self,path,G):self.payload_path=path;self.payload_relative=np.asarray(G).copy();self.payload_rows=[];self.payload_boxes=[]
 @contextmanager
 def other_arm_at(self,*args):yield
cmd=np.zeros((1,18),np.float32);cmd[:,14:]=.03;art=Art(cmd);lc,rc=Checker('left'),Checker('right')
c={'config':{'stability_serial':True,'fixed_supply_resume_after_right_grasp':True,'optimized_motion_timing':True,'defer_next_initial_until_receiver_bound':True},'_collision_physics_epoch':7,'_last_loaded_left_issued_full_proof':None,'dt':1/480,'cmd':cmd,'art':art,'left_ids':list(range(7)),'ids':list(range(7,14)),'left_fi':[14,15],'fi':[16,17],'checkers':{'left':lc,'right':rc},'goals':{'ring2':{'P':[0.,0.,1.]}},'part_collision_templates':{'ring2':'/ring2'},'planning_diagnostics':[],'fixed_prevalidated_small_left_routes':{'world_sha256':'sealed-world','model_sha256':'sealed-model','certificate':types.SimpleNamespace(metadata={'test':'immutable-geometry'})}}
binding=contactmod.LoadedContactAperture(side='left',part='ring2',payload_path='/ring2',encoder_samples=np.repeat(cmd[0,[14,15]][None,:],6,axis=0),sample_times_s=np.linspace(0,.1,6),contact={'opposed_contact_compatible':True,'both_hulls_near_CAD':True},measured_grasp=np.eye(4),control_dt_s=c['dt'],capture_age_physics_s=.1)
lc.loaded_contact_aperture=binding
counts={'full_guard':0,'full_guard_clear':True,'physics_steps':0,'fresh_pair_calls':0}
def full_path(*args):counts['full_guard']+=1;return counts['full_guard_clear'],{'fixture':'deliberate full guard failure'}
env.update(path_openings=full_path,_samples=lambda checker,qs:iter([]),pair_clear=lambda *args:(True,None))
left_tree=ast.parse((BASE/'left_loaded_path_guard.py').read_text());execute([find(left_tree,'current_query'),find(left_tree,'verify_issued')],env)
verify=env['verify_issued'];G=np.eye(4);issued=np.c_[np.array([[.001]*7,[.002]*7,[.003]*7]),np.repeat(cmd[0,[14,15]][None,:],3,axis=0)]
route=fixed.SavedCommandRoute(issued,np.r_[cmd[0,c['left_ids']],cmd[0,c['left_fi']]],{})
env.update(c=c,cmd=cmd,art=art,arms={'left':lc.arm,'right':rc.arm},indices={'left':c['left_ids'],'right':c['ids']},fingers={'left':c['left_fi'],'right':c['fi']},part='ring2',left_G=G,route=route,end=np.eye(4),D=.062,row={},free_yaw=True,verify_loaded_left_issued=verify)
def trajectory(side,qs,t,width=None,checked_edges=False):return qs.commands(c,side,width)
env['trajectory']=trajectory
pair=env['PairedIssuedOverlap'](c);env['overlap']=pair
execute([find(tree,'insert_program')],env)
# Original proof first: same as saved candidate admission; no actuation.
verify(c,'ring2',G,issued);assert counts['full_guard']==1
# Literal original nested priming function with a real nonlocal owner scope.
prime_node=find(tree,'prime_left_chunk')
factory=ast.parse('def make_prime(paired, epoch):\n left_program=None\n overlap=paired\n control_steps=epoch\n left_finished_step=0\n def state():return left_program,control_steps,left_finished_step\n').body[0]
factory.body.append(prime_node)
# Execute the actual newly-added macro block, including its type guard and
# command/epoch invariant, inside the same nonlocal owner as production.
task_loop=next(n for n in ast.walk(tree)if isinstance(n,ast.For)and isinstance(n.target,ast.Tuple)and [getattr(x,'id',None)for x in n.target.elts]==['task_index','part'])
assignment_index=next(i for i,n in enumerate(task_loop.body)if isinstance(n,ast.Assign)and isinstance(n.value,ast.Call)and isinstance(n.value.func,ast.Name)and n.value.func.id=='insert_program')
factory.body.extend(task_loop.body[assignment_index:assignment_index+2])
factory.body.append(ast.Return(value=ast.Tuple(elts=[ast.Name(id='prime_left_chunk',ctx=ast.Load()),ast.Name(id='state',ctx=ast.Load())],ctx=ast.Load())))
execute([factory],env)
before_cmd=cmd.copy();before_q=art.q.copy();before_epoch=c['_collision_physics_epoch']
prime,state=env['make_prime'](pair,c['_collision_physics_epoch']);program=state()[0];pair.prime=prime
assert counts['full_guard']==1 and c['planning_diagnostics'][-1]['hit'] is True
assert np.array_equal(cmd,before_cmd)and np.array_equal(art.q,before_q)and c['_collision_physics_epoch']==before_epoch and counts['physics_steps']==0
assert pair.chunk is not None and pair.chunk['cursor']==0 and not pair.awaiting_prime and pair.token is None
# Re-priming must not consume a row or repeat a proof.
prime();assert pair.chunk['cursor']==0 and counts['full_guard']==1 and np.array_equal(cmd,before_cmd)
# A camera-wait step/encoder change cannot inherit the same-epoch proof.
c['_collision_physics_epoch']+=1;counts['physics_steps']+=1
verify(c,'ring2',G,issued);assert counts['full_guard']==2 and c['planning_diagnostics'][-2]['hit'] is False
# Same epoch changed encoder/command/width/G/binding/world also forces fallback.
miss_checks=[]
for name,mutate,restore in [
 ('actual_encoder',lambda:art.q.__setitem__((0,0),.0001),lambda:art.q.__setitem__((0,0),0.)),
 ('command_width',lambda:cmd.__setitem__((0,14),np.float32(.03001)),lambda:cmd.__setitem__((0,14),np.float32(.03))),
 ('world_revision',lambda:c.__setitem__('_collision_world_revision',1),lambda:c.__setitem__('_collision_world_revision',0)),
]:
 verify(c,'ring2',G,issued);oldcount=counts['full_guard'];mutate();verify(c,'ring2',G,issued);assert counts['full_guard']==oldcount+1;miss_checks.append(name);restore()
# The original paired scheduler still needs a NEW proof using current encoders
# after the later camera wait, before it creates an execution token.
art.q[0,0]=.0002;pair.begin_pick();assert not pair.advance_left_this_step()
seen=[]
def fresh_pair(left,right,chunk,**kwargs):
 counts['fresh_pair_calls']+=1;seen.append(art.get_dof_positions().copy());return None
pair._verify=fresh_pair
right=np.tile(np.r_[cmd[0,c['ids']],cmd[0,c['fi']]],(3,1));pair.before_right(right,'loose_ring_pregrasp')
assert counts['fresh_pair_calls']==1 and seen[0][0,0]==art.q[0,0] and pair.chunk['cursor']==0
# Consume exactly the same immutable three issued rows, one at a time; exposure
# itself has not consumed or altered any row. Closing follows original finally.
consumed=[]
for row in issued:
 next(program);consumed.append(np.r_[cmd[0,c['left_ids']],cmd[0,c['left_fi']]].copy());pair.step_completed()
assert np.array_equal(np.asarray(consumed),issued.astype(np.float32).astype(float))
pair.after_right();program.close();assert pair.chunk is None
# The complete original full guard still rejects a changed-context collision.
counts['full_guard_clear']=False;c['_collision_physics_epoch']+=1
try:verify(c,'ring2',G,issued)
except RuntimeError as error:assert str(error).startswith('Loaded LEFT issued curve blocked')
else:raise AssertionError('Original changed-context full guard failure was ignored')
# All earlier source code, including camera priority and PIN, is byte-identical.
old=(BASE/'loose_bin_task.py').read_text();anchor="        left_program=insert_program()\n        if overlap is not None:overlap.awaiting_prime=True\n";start=text.index('        left_program=insert_program()\n');end=text.index("        if c['config'].get('pin_between_rings')",start)
assert text[:start]+anchor+text[end:]==old
value={'pass':True,'literal_generator_rows_and_prime_unchanged':True,'actual_new_macro_branch_executed':True,'existing_exact_epoch_proof_hit_before_physics':True,'no_command_encoder_or_physics_mutation_on_exposure':True,'second_prime_does_not_consume_or_reverify':True,'epoch_change_forces_original_full_guard':True,'additional_invalidation_checks':miss_checks,'original_failure_propagates':True,'fresh_current_encoder_pair_guard_required_before_motion':True,'issued_row_bytes_and_consumption_unchanged':True,'chunk_cleanup_unchanged':True,'earlier_camera_priority_PIN_and_macro_bytes_unchanged':True,'native_collision_math_mocked_for_lifecycle_only':True,'no_GPU_PhysX_or_live_jobs':True,'source_sha256':{name:hashlib.sha256((BASE/name).read_bytes()).hexdigest()for name in ['left_loaded_path_guard.py','exact_loaded_left_proof.py','paired_issued_overlap.py']},'new_macro_sha256':hashlib.sha256(text.encode()).hexdigest()}
(STAGE/'lifecycle-results.json').write_text(json.dumps(value,indent=2)+'\n');print(json.dumps(value,indent=2))
