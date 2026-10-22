"""Compare native-hand paths under common start, task waits and joint limits.
Static/kinematic path simulation, not contact-based insertion certification.
"""
from layout_search import *
from scipy.spatial.transform import Slerp


def rrt(side,start,goal,gap,seed=7,max_nodes=500):
 def edge(a,b):
  count=max(5,int(np.ceil(np.max(np.abs(np.asarray(b)-np.asarray(a)))/.015))+1)
  return path_clear(side,[np.asarray(a),np.asarray(b)],gap,count)[0]
 if not clear(side,np.asarray(start),gap)[0] or not clear(side,np.asarray(goal),gap)[0]:return None
 if edge(start,goal):return [start,goal]
 for joint in [6,0,2,4,1,3]:
  for weight in [.35,.65,1.]:
   middle=np.array(start).copy();middle[joint]=start[joint]+weight*(goal[joint]-start[joint])
   if edge(start,middle)and edge(middle,goal):return [np.asarray(start),middle,np.asarray(goal)]
 rng=np.random.default_rng(seed);trees=[[(np.array(start),-1)],[(np.array(goal),-1)]]
 low=np.maximum(LOWER+.06,np.minimum(start,goal)-.9);high=np.minimum(UPPER-.06,np.maximum(start,goal)+.9)
 def extend(tree,target):
  i=int(np.argmin([np.linalg.norm(x[0]-target)for x in tree]));q=tree[i][0];dist=np.linalg.norm(target-q);new=q+(target-q)*min(1.,.22/max(dist,1e-9))
  if not edge(q,new):return None
  tree.append((new,i));return len(tree)-1
 def trace(tree,i):
  out=[]
  while i>=0:q,i=tree[i];out.append(q)
  return out[::-1]
 for k in range(max_nodes):
  a=k%2;b=1-a;target=trees[b][0][0]if rng.random()<.18 else rng.uniform(low,high);ia=extend(trees[a],target)
  if ia is None:continue
  while True:
   ib=extend(trees[b],trees[a][ia][0])
   if ib is None:break
   if np.linalg.norm(trees[b][ib][0]-trees[a][ia][0])<1e-6:
    qa=trace(trees[a],ia);qb=trace(trees[b],ib);path=qa+qb[-2::-1]if a==0 else qb+qa[-2::-1]
    # Deterministic shortcut only across collision-screened spans.
    for _ in range(12):
     if len(path)<3:break
     i,j=sorted(rng.integers(0,len(path),2))
     if j>i+1 and edge(path[i],path[j]):path=path[:i+1]+path[j:]
    return path
   if len(trees[0])+len(trees[1])>max_nodes:return None
 return None


def make_recipe(part,band,y,dz,baseline=False,x=None):
 side='right'if part=='ring1'else'left';R=np.array(band['R']);tcp=np.array(band['tcp']);qi=np.array(band['q_insert']);pre=np.array(band['q_preinsert']);gap=band['gap'];old=next(p for p in parts if p['part']==part);Rp=R@np.array(band['localR']).T
 if baseline:
  ref=old['endpoints'][0];pick=np.array(ref['tcp_world_m']);Rp_pick=np.array(ref['hand_rotation'])@np.array(old['constant_hand_to_part_rotation']).T;Rpick=Rp_pick@np.array(band['localR']);seed=np.array(ref['arm_clear_ik'][0]['qpos'])
 else:
  pick=tcp+[0,y-tcp[1],dz];Rpick=R;seed=pre
  if x is not None:pick[0]=x
 qpick=ik(side,pick,Rpick,seed)
 if qpick is None:return None,'pickup_ik'
 ok,why=clear(side,qpick,.08 if part.startswith('ring')else .02)
 if not ok:return None,dict(pickup=why)
 axis=Rpick[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis)
 if axis[1]>0:axis=-axis
 extracted=ik(side,pick+axis*.10,Rpick,qpick)
 if extracted is None:return None,'extract_ik'
 lifted=ik(side,pick+axis*.10+[0,0,.04],Rpick,extracted)
 if lifted is None:return None,'lift_ik'
 route=[qpick,extracted,lifted];transfer=rrt(side,lifted,pre,gap,max_nodes=260)
 if transfer is None:return None,'transfer_rrt'
 route+=transfer[1:]+[qi]
 ok,why=path_clear(side,route,gap,17)
 if not ok:return None,dict(transfer=why)
 return dict(part=part,side=side,tcp=pick.tolist(),R=Rpick.tolist(),localP=band['localP'],localR=band['localR'],gap=gap,q_pick=qpick.tolist(),q_path=[x.tolist()for x in route],q_exit=pre.tolist(),extraction_axis=axis.tolist(),hand_machine_insert_clearance_m=band['hand_machine_clearance_m']),None


def part_segments(recipe,start):
 side=recipe['side'];gap=recipe['gap'];qs=[np.asarray(x)for x in recipe['q_path']];entry=rrt(side,np.asarray(start),qs[0],.08 if recipe['part'].startswith('ring')else .02,max_nodes=280)
 if entry is None:return None
 out=[]
 def motion(a,b,label):out.append(dict(a=np.asarray(a).tolist(),b=np.asarray(b).tolist(),duration_s=segment_time(a,b),label=label,gap_m=gap))
 for a,b in zip(entry[:-1],entry[1:]):motion(a,b,'empty_approach')
 out.append(dict(a=qs[0].tolist(),b=qs[0].tolist(),duration_s=.18,label='grasp',gap_m=gap))
 for i,(a,b)in enumerate(zip(qs[:-1],qs[1:])):motion(a,b,'extract'if i==0 else'lift'if i==1 else'insert'if i==len(qs)-2 else'transfer')
 out.append(dict(a=qs[-1].tolist(),b=qs[-1].tolist(),duration_s=.18,label='clamp',gap_m=gap))
 out.append(dict(a=qs[-1].tolist(),b=qs[-1].tolist(),duration_s=.18,label='release',gap_m=.08 if recipe['part'].startswith('ring')else .02))
 motion(qs[-1],recipe['q_exit'],'withdraw')
 for x in out:x['part']=recipe['part']
 return out


def pose_at(segments,t,ready):
 elapsed=0
 for seg in segments:
  T=seg['duration_s']
  if t<elapsed+T:
   u=float(np.clip((t-elapsed)/T,0,1));h=u*u*u*(10-15*u+6*u*u);return np.array(seg['a'])*(1-h)+np.array(seg['b'])*h,seg
  elapsed+=T
 return np.array(segments[-1]['b'])if segments else np.array(ready),segments[-1]if segments else None


def cross_clear(lanes,delays,dt=.06):
 end=max(sum(s['duration_s']for s in lanes[side])+delays[side]for side in AD)
 for t in np.arange(0,end+dt/2,dt):
  poses={};obs={}
  for side in AD:
   q,seg=pose_at(lanes[side],t-delays[side],READY[side]);gap=seg['gap_m'] if seg else .08
   if seg and seg['label']in ('empty_approach','withdraw'):gap=.08 if seg['part'].startswith('ring')else .02
   obs[side]=objects(side,q,gap)[0]
  for na,a in obs['left']:
   if na.endswith('link0'):continue
   for nb,b in obs['right']:
    if nb.endswith('link0'):continue
    if pairhit(a,b):return False,dict(time_s=float(t),pair=[na,nb])
 return True,None


def button_path(start):
 side='right';contact=np.array([.3559690421575,-.24715,1.527]);found=[]
 for sign in [1,-1]:
  R=np.column_stack([[0,0,sign],[sign,0,0],[0,1,0]])
  tcp=contact-R[:,2]*.0095;pre=ik(side,tcp-[0,.04,0],R,np.asarray(start))
  if pre is None:continue
  final=ik(side,tcp,R,pre)
  if final is None:continue
  route=rrt(side,np.asarray(start),pre,0.,max_nodes=300)
  if route is None:continue
  if not path_clear(side,[pre,final],0.,21)[0]:continue
  segs=[]
  for a,b in zip(route[:-1],route[1:]):segs.append(dict(a=a.tolist(),b=b.tolist(),duration_s=segment_time(a,b),gap_m=0.,label='button_approach',part='button'))
  press=[dict(a=pre.tolist(),b=final.tolist(),duration_s=max(.18,segment_time(pre,final)),gap_m=0.,label='button_press',part='button'),dict(a=final.tolist(),b=final.tolist(),duration_s=.10,gap_m=0.,label='button_dwell',part='button'),dict(a=final.tolist(),b=pre.tolist(),duration_s=max(.18,segment_time(final,pre)),gap_m=0.,label='button_retract',part='button')]
  found.append((sum(s['duration_s']for s in segs+press),segs,press))
 return min(found,key=lambda a:a[0])if found else None


def main():
 begin=time.monotonic();center=json.load(open(HERE/'centered-grasps.json'));refined=json.load(open(HERE/'refined-grasp-bands.json'))if (HERE/'refined-grasp-bands.json').exists()else[];bands={}
 for p in parts:
  rows=next(r['candidates']for r in center if r['part']==p['part'])
  if p['part']!='pin':rows+=next((r['candidates']for r in refined if r['part']==p['part']),[])
  if not rows:raise RuntimeError('No centered band '+p['part'])
  if p['part']!='pin':
   preferred=[r for r in rows if r.get('yaw_deg',0)==0 and r.get('pitch_delta_deg',0)==0 and abs(r.get('x_offset_m',0))<=.0046 and r['hand_machine_clearance_m']>.001]
   bands[p['part']]=min(preferred,key=lambda r:abs(r.get('x_offset_m',0))) if preferred else max(rows,key=lambda r:r['hand_machine_clearance_m'])
  else:bands[p['part']]=max(rows,key=lambda r:r['hand_machine_clearance_m'])
 results=[]
 for label,y in [('original',None),('far_front',-.22),('middle_front',-.12),('near_front',-.02),('closest_front',.08)]:
  recipes={};failure=[]
  for part in ['ring1','ring2','pin']:
   choices=[]
   for dz in ([-.08]if y is None else[-.18,-.08,.03]):
    r,why=make_recipe(part,bands[part],y,dz,y is None)
    if r is None:failure.append(dict(part=part,dz=dz,reject=why));continue
    start=READY[r['side']]if part!='pin'else recipes['ring2']['q_exit']if 'ring2'in recipes else READY['left'];segs=part_segments(r,start)
    if segs is None:failure.append(dict(part=part,dz=dz,reject='empty_approach_rrt'));continue
    duration=sum(s['duration_s']for s in segs)
    if duration>30:
     failure.append(dict(part=part,dz=dz,reject='velocity_envelope_or_duration'));continue
    r['segments']=segs;r['duration_s']=duration;choices.append(r)
   if not choices:break
   recipes[part]=min(choices,key=lambda r:r['duration_s']);print('RECIPE',label,part,recipes[part]['duration_s'],flush=True)
  if len(recipes)!=3:
   results.append(dict(label=label,valid=False,failures=failure));(HERE/'comparison-progress.json').write_text(json.dumps(results,indent=2));continue
  button=button_path(recipes['ring1']['q_exit'])
  if button is None:results.append(dict(label=label,valid=False,reject='native_button_path',recipes=recipes));continue
  _,approach,press=button;lanes=dict(left=recipes['ring2']['segments']+recipes['pin']['segments'],right=recipes['ring1']['segments']+approach)
  options=[]
  for side in ['right','left']:
   for delay in [0,.25,.5,.75,1.,1.5,2.,2.5,3.]:
    delays={s:delay if s==side else 0 for s in AD};ok,why=cross_clear(lanes,delays,.08)
    if ok:
     ready=max(sum(s['duration_s']for s in lanes[s])+delays[s]for s in AD);options.append((ready+sum(s['duration_s']for s in press),delays));break
  if not options:results.append(dict(label=label,valid=False,reject='cross_arm',recipes=recipes));continue
  total,delays=min(options);row=dict(label=label,valid=True,cycle_time_s=total,recipes=recipes,lanes=lanes,delays=delays,button_finish=press,scope='kinematic trajectory time; physical full assembly unvalidated',fixture_geometry_checked=False)
  results.append(row);(HERE/'comparison-progress.json').write_text(json.dumps(results,indent=2));print('LAYOUT',label,total,delays,flush=True)
 (HERE/'comparison.json').write_text(json.dumps(dict(layouts=results,seconds=time.monotonic()-begin,grasp_bands=bands,hardware_ready=False,full_physics_assembly_tested=False),indent=2))
if __name__=='__main__':main()
