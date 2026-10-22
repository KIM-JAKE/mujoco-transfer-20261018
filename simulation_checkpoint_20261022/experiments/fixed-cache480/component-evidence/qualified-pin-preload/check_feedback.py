from pathlib import Path
import ast,json,numpy as np
from scipy.spatial.transform import Rotation as Rot
exp=lambda v:Rot.from_rotvec(v).as_matrix()
log=lambda R:Rot.from_matrix(R).as_rotvec()
def frame(p,R):
 T=np.eye(4);T[:3,:3]=R;T[:3,3]=p;return T

def load(path,name):
 f=next(x for x in ast.parse(Path(path).read_text()).body if isinstance(x,ast.FunctionDef)and x.name==name);env=dict(np=np,exp=exp,log=log,frame=frame);exec(compile(ast.Module(body=[f],type_ignores=[]),str(path),'exec'),env);return env[name]
p='/tmp/fixed478-pin-qualified-preload/direct_pin_skill.py';servo=load(p,'_pin_visual_preinsert_target');prior=load('/tmp/fixed476-full/overrides/direct_pin_skill.py','_pin_visual_preinsert_target');gainfn=load(p,'_pin_bounded_xy_gain')
# Reject invalid response states. No original gain floor or ceiling relaxed.
for g,r,pr,c in [(float('nan'),1e-4,None,.65),(.29,1e-4,None,.65),(.66,1e-4,None,.65),(.3,-1e-4,None,.65),(.3,1e-4,float('inf'),.65)]:
 try:gainfn(g,r,pr,0,c)
 except ValueError:pass
 else:raise AssertionError('Invalid gain state admitted')
g=.65;streak=0;previous=1000e-6;trace=[]
for radial in [1100e-6,1200e-6,1300e-6,1000e-6,800e-6,600e-6,400e-6,300e-6,200e-6,150e-6,160e-6,170e-6,180e-6]:
 g,streak=gainfn(g,radial,previous,streak);trace.append([g,streak]);assert .3<=g<=.65;previous=radial
assert trace[2][0]==.3 and trace[4][0]>.3 and max(x[0]for x in trace)==.65
# Near-qualified/weak improvement cannot accumulate strong-improvement credit.
assert gainfn(.3,80e-6,85e-6,1)==(.3,0)
assert gainfn(.3,70e-6,100e-6,1)==(.3,0)
rows=[];goal=np.array(json.load(open('/tmp/fixed476-full/trial-config.json'))['pin_axis_goal_world_m'])
for v in ['474','476']:
 for i,row in enumerate(json.load(open('/tmp/pin465-analysis/sequence'+v+'.json'))['rows']):
  A=np.array(row['A']);C=np.array(row['C']);P=np.array(row['P']);axis=np.array(row['axis']);u=np.array([1.,0.,0.]);u-=axis*(u@axis);u/=np.linalg.norm(u);R=np.column_stack([u,np.cross(axis,u),axis]);G=np.linalg.inv(A)@frame(P,R)
  kw=dict(command_H=C,actual_H=A,measured_G=G,goal=goal,goal_axis=np.array([0.,0.,1.]),observed_z=P[2],gain=.65,translation_deadband_m=.000075,axis_deadband_rad=.003)
  # Old/default component mode stays exactly476; legacy target also unchanged.
  for ag in [None,1.]:
   a,_=servo(**kw,axis_gain=ag);b,_=prior(**kw,axis_gain=ag);assert np.array_equal(a,b)
  kw['axis_deadband_rad']=.001;T,e=servo(**kw,axis_gain=.65);prediction=A@np.linalg.inv(C)@T@G
  radial=np.linalg.norm(P[:2]-goal[:2]);angle=np.arccos(np.clip(axis[2],-1,1));targetxy=P[:2] if radial<=.000075 else P[:2]+.65*(goal[:2]-P[:2]);assert np.linalg.norm(prediction[:2,3]-targetxy)<1e-12
  angle_after=float(np.arccos(np.clip(prediction[2,2],-1,1)));expected=angle if angle<=.001 else .35*angle;assert abs(angle_after-expected)<1e-9
  rows.append(dict(run=v,iteration=i,center_target_invariant=True,axis_target_invariant=True))
report=dict(status='PASS',scope='Only gain-law invariants and public RGB-D/encoder kinematic target replay; no physical outcome prediction',actual_frames=len(rows),legacy_and476_targets_bitwise_equal=True,bounded_gain_floor=.3,bounded_gain_ceiling=.65,invalid_gain_state_rejected=True,recovery_requires_two_consecutive_strong_improvements=True,trace=trace,rows=rows)
Path('/tmp/fixed478-pin-qualified-preload/feedback-qualification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['rows','trace']},indent=2))
