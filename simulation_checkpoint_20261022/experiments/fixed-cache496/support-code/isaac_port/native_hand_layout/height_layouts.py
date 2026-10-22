import evaluate_layouts as ev
from evaluate_layouts import *
base_record=json.load(open(HERE/'comparison-physical-band.json'));bands=base_record['grasp_bands'];p=prepare_part(parts[2]);bands['pin']=dict(part='pin',R=p['R'].tolist(),tcp=p['tcp'].tolist(),q_insert=p['q_insert'].tolist(),q_preinsert=ik('left',p['tcp']+[0,0,-.055],p['R'],p['q_insert']).tolist(),localP=p['localP'].tolist(),localR=p['localR'].tolist(),gap=.0097,hand_machine_clearance_m=float(g.clearance(p['R'],p['tcp'],.0097)[0]))

def freeik(side,tcp,R,seed):
 goal=tcp-R@np.array([0,0,.1034]);rgoal=Rotation.from_matrix(R@Rotation.from_euler('z',45,degrees=True).as_matrix())
 def fun(q):
  data.qpos[AD[side]]=q;mujoco.mj_kinematics(model,data);return np.r_[10*(data.xpos[FL[side]]-goal),(Rotation.from_matrix(data.xmat[FL[side]].reshape(3,3))*rgoal.inv()).as_rotvec()]
 for bias in [0,.5,-.5]:
  start=np.array(seed);start[0]+=bias;fit=least_squares(fun,np.clip(start,LOWER+.002,UPPER-.002),bounds=(LOWER+.002,UPPER-.002),max_nfev=70)
  if np.linalg.norm(fun(fit.x)[:3])<1e-4 and np.linalg.norm(fun(fit.x)[3:])<1e-4:return fit.x
 return None
# Use free redundancy only as a fallback; preserve established good branches.
def robust(side,tcp,R,seed):
 result=ik(side,tcp,R,seed)
 return result if result is not None and np.min(np.r_[result-LOWER,UPPER-result])>.02 else freeik(side,tcp,R,seed)
ev.ik=robust
rows=[]
for label,y,xs,zs in [('height_left_low',-.12,[-.122945,-.237558,-.302095],[.03,-.18,-.18]),('height_right_low',-.12,[-.122945,-.237558,-.302095],[-.18,.03,-.18])]:
 recipes={};failure=[]
 for part,x,dz in zip(['ring1','ring2','pin'],xs,zs):
  r,why=make_recipe(part,bands[part],y,dz,x=x)
  if r is None:failure.append(dict(part=part,reject=why));break
  start=READY[r['side']]if part!='pin'else recipes['ring2']['q_exit'];segs=part_segments(r,start)
  if segs is None:failure.append(dict(part=part,reject='entry'));break
  r['segments']=segs;r['duration_s']=sum(s['duration_s']for s in segs)
  if r['duration_s']>25:failure.append(dict(part=part,reject='velocity_bounds'));break
  recipes[part]=r;print('LATERAL_RECIPE',label,part,r['duration_s'],flush=True)
 row=dict(label=label,recipes=recipes,valid=False,failures=failure);rows.append(row)
 (HERE/'height-comparison.json').write_text(json.dumps(dict(layouts=rows,grasp_bands=bands,hardware_ready=False),indent=2))
base_record['layouts'].extend(rows);(HERE/'comparison-height-extended.json').write_text(json.dumps(base_record,indent=2))
