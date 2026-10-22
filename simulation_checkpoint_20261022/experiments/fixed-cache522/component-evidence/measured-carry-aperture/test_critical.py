from pathlib import Path
from contextlib import contextmanager
import ast,copy,types,sys,json,time
import numpy as np
O=Path(__file__).parent;source=O/'overrides/direct_pin_skill.py';tree=ast.parse(source.read_text());helpers=[n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name in ['_pin_carry_aperture_domain','_pin_check_carry_aperture','_pin_admit_measured_carry']]
def frame(P,R):
 x=np.eye(4);x[:3,3]=P;x[:3,:3]=R;return x
state={};calls=[]
class Art:
 def get_dof_positions(self):return actual.copy()
class Checker:
 payload_path='/old';payload_rows=['oldrows'];payload_boxes=['oldboxes'];payload_relative=np.eye(4)
 def attach_payload(self,p,G):self.payload_path=p;self.payload_rows=['PINrows'];self.payload_boxes=['PINboxes'];self.payload_relative=G
checker=Checker();other=object();contexts=[(np.zeros(7),.01),(np.ones(7)*.01,.02)]
@contextmanager
def scope(c,ck,o,ct,qs,ws):
 assert c['checkers']=={'right':checker,'left':other};assert ct is contexts;assert ck.payload_path=='/pin';calls.append('GPU_scope_enter');yield;calls.append('GPU_scope_exit')
def path(ck,o,ct,rows,widths,selected,diagnostics):
 calls.append('native');assert selected=='/pin';assert ck.payload_boxes==['PINboxes'];assert np.array_equal(ck.payload_relative,frame([0,0,.1034],np.eye(3))@G)
 assert np.array_equal(rows[:2],np.vstack([actual[0,:7],cmd[0,:7]]));assert np.array_equal(rows[2:],issued);assert widths.min()<=.0095 and widths.max()>.0135
 diagnostics['all_original_guards']=True
 if state.get('raise'):raise RuntimeError('native fault')
 return not state.get('blocked',False),{'blocked':'native'}
sys.modules['pin_current_terminal']=types.SimpleNamespace(_other_contexts=lambda c:(other,contexts));sys.modules['dual_aperture_path_check']=types.SimpleNamespace(path_other_contexts=path);sys.modules['serial_gpu_pair_scope']=types.SimpleNamespace(serial_path_scope=scope)
env=dict(np=np,time=time,frame=frame,print=lambda *a,**k:None);exec(compile(ast.Module(body=helpers,type_ignores=[]),str(source),'exec'),env)
actual=np.zeros((1,9));actual[0,7:]=.01255/2;cmd=actual.copy();cmd[0,7:]=.0095/2;ctx=dict(art=Art(),fi=np.array([7,8]),ids=np.arange(7),cmd=cmd,dt=1/480,checker=checker,paths={'workpiece_pin':'/pin'},report={},config={});G=np.eye(4);G[:3,3]=[.001,.002,.003];issued=np.array([np.zeros(7),np.ones(7)*.1]);tests=[]
d=env['_pin_carry_aperture_domain'](ctx);assert d['lower_m']<=.0095-2*.0475/480 and d['upper_m']>=.0135+2*.0475/480;assert .0095 in d['widths_m'] and float(actual[0,7:].sum())in d['widths_m'];assert len(d['widths_m'])>=5;env['_pin_check_carry_aperture'](ctx,d);tests.append('full original hold band+fresh command+actual+tick domain;9.5mm force target allowed')
saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
for case in ['clear','blocked','fault']:
 state.clear();state.update(blocked=case=='blocked',raise_=case=='fault');state['raise']=case=='fault';calls.clear();before=cmd.copy()
 try:got=env['_pin_admit_measured_carry'](ctx,issued,G,'pin_tail_direct_carry_to_preinsert')
 except RuntimeError as e:assert case!='clear'and ('native fault'==str(e)if case=='fault'else'blocked before motion'in str(e))
 else:assert case=='clear'and got['lower_m']==d['lower_m']
 assert np.array_equal(cmd,before);assert checker.payload_path==saved[0]and checker.payload_rows is saved[1]and checker.payload_boxes is saved[2]and checker.payload_relative is saved[3]
 assert calls[:2]==['GPU_scope_enter','native'];tests.append(case+' full original verifier and payload restored, no commands/physics issued')
# Compile literal production motion and demonstrate proof-before-row and runtime bound stop.
motion=next(n for n in ast.walk(tree)if isinstance(n,ast.FunctionDef)and n.name=='motion');motion=copy.deepcopy(motion);motion.body=[n for n in motion.body if not isinstance(n,ast.Nonlocal)];exec(compile(ast.Module(body=[motion],type_ignores=[]),str(source),'exec'),env)
def fake_retime(*args,**kw):return issued.copy(),np.zeros(2),{'duration_s':1/480}
checks=[];state.clear()
def step():checks.append('step')
def admit(*args):checks.append('admit');return d
def check(*args):checks.append('runtime');env['_pin_check_carry_aperture_original'](*args)
env.update(ctx=ctx,arm=None,cmd=cmd,ids=np.arange(7),fi=np.array([7,8]),DT=1/480,pin_config={'pin_measured_carry_aperture_guard':True},pickup_opening_m=.024,report={'events':[]},started=time.perf_counter(),payload_grasp=lambda:G,step=step,retime=fake_retime,retime_safe_polyline=fake_retime,_pin_admit_measured_carry=admit,_pin_check_carry_aperture_original=env['_pin_check_carry_aperture'],_pin_check_carry_aperture=check)
env['motion'](issued,.1,'pin_tail_direct_carry_to_preinsert');assert checks==['admit','runtime','step','runtime'];tests.append('literal motion admits complete retimed field before first row; runtime checks before/after each step')
for cause in ['actual_hold','command_domain','lookahead_domain']:
 actual[0,7:]=.01255/2;cmd[0,7:]=.0095/2;bad=dict(d)
 if cause=='actual_hold':actual[0,7:]=.0136/2
 if cause=='command_domain':cmd[0,7:]=.02/2
 if cause=='lookahead_domain':bad['upper_m']=.0126
 try:env['_pin_check_carry_aperture_original'](ctx,bad)
 except RuntimeError:pass
 else:raise AssertionError(cause)
 tests.append(cause+' rejected')
# Archival flag-off: no new admission/perstep check, planner scalar still10.5mm.
actual[0,7:]=.01255/2;cmd[0,7:]=.0095/2;checks.clear();env['pin_config']={};env['motion'](issued,.1,'pin_tail_direct_carry_to_preinsert');assert checks==['step'];tests.append('flagoff original motion lifecycle')
(O/'critical-tests.json').write_text(json.dumps({'PASS':True,'tests':tests,'native_geometry_qualification':False,'no_jobs_or_actuation':True},indent=2)+'\n');print('PASS',len(tests))
