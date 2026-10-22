from pathlib import Path
import ast,json,numpy as np
from scipy.spatial.transform import Rotation as Rot

def exp(v):return Rot.from_rotvec(v).as_matrix()
def log(R):return Rot.from_matrix(R).as_rotvec()
def frame(p,R):
 T=np.eye(4);T[:3,:3]=R;T[:3,3]=p;return T

def load(p):
 node=next(n for n in ast.parse(Path(p).read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='_pin_visual_preinsert_target');scope=dict(np=np,exp=exp,log=log,frame=frame);exec(compile(ast.Module(body=[node],type_ignores=[]),str(p),'exec'),scope);return scope[node.name]
old=load('/tmp/fixed475-full/overrides/direct_pin_skill.py');new=load('/tmp/fixed476-pin-component-feedback/direct_pin_skill.py')
data=json.load(open('/tmp/pin465-analysis/sequence474.json'));goal=np.array(json.load(open('/tmp/fixed475-full/trial-config.json'))['pin_axis_goal_world_m']);results=[]
for i,row in enumerate(data['rows']):
 A=np.array(row['A']);C=np.array(row['C']);P=np.array(row['P']);axis=np.array(row['axis']);u=np.array([1.,0.,0.]);u-=axis*(u@axis);u/=np.linalg.norm(u);R=np.column_stack([u,np.cross(axis,u),axis]);G=np.linalg.inv(A)@frame(P,R)
 kw=dict(command_H=C,actual_H=A,measured_G=G,goal=goal,goal_axis=np.array([0.,0.,1.]),observed_z=P[2],gain=.5,translation_deadband_m=.000075,axis_deadband_rad=.003)
 oldT,_=old(**kw);legacyT,_=new(**kw);assert np.array_equal(oldT,legacyT)
 T,e=new(**kw,axis_gain=1.);expected_actual=A@np.linalg.inv(C)@T;pred=expected_actual@G
 radial=np.linalg.norm(P[:2]-goal[:2]);ae=np.arccos(np.clip(axis[2],-1,1));targetxy=P[:2] if radial<=.000075 else (P[:2]+goal[:2])/2
 assert np.linalg.norm(pred[:2,3]-targetxy)<1e-12
 expected_axis=axis if ae<=.003 else np.array([0.,0.,1.]);assert np.linalg.norm(pred[:3,2]-expected_axis)<1e-12
 results.append(dict(iteration=i,initial_radial_um=radial*1e6,initial_axis_mrad=ae*1e3,kinematic_commanded_radial_um=float(np.linalg.norm(pred[:2,3]-goal[:2])*1e6),kinematic_commanded_axis_mrad=float(np.arccos(np.clip(pred[2,2],-1,1))*1e3),translation_held=e['qualified_XY_preserved'],axis_held=e['qualified_axis_preserved']))
# Explicitly check a qualified center remains fixed during a large axis change.
A=np.eye(4);A[:3,:3]=exp(np.array([.04,-.03,.2]));A[:3,3]=[.2,.1,1.4];C=A.copy();C[:3,3]+=[.0002,-.0001,.0001];G=frame([.02,-.015,.04],exp(np.array([-.035,.025,0.])));P=(A@G)[:3,3];testgoal=P.copy();testgoal[:2]+=[.00002,-.00003];testgoal[2]+=.045
T,e=new(C,A,G,testgoal,[0,0,1],P[2],gain=.5,translation_deadband_m=.000075,axis_deadband_rad=.003,axis_gain=1.)
pred=A@np.linalg.inv(C)@T@G
assert np.linalg.norm(pred[:2,3]-P[:2])<1e-12 and np.linalg.norm(pred[:3,2]-[0,0,1])<1e-12
report=dict(status='PASS',scope='Algebraic public RGB-D+encoder replay only; NOT a physical success prediction',source='474 actual16 RGB-D observations and public encoders',legacy_default_target_bitwise_equal=True,independent_component_target_invariants_pass=True,qualified_center_during_axis_change_pass=True,rows=results)
Path('/tmp/fixed476-pin-component-feedback/public474-replay-check.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
