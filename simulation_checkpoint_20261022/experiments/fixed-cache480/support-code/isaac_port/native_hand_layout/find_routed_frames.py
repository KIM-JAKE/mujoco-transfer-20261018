from evaluate_layouts import *
def manager_hit(obj,manager):
 query=fcl.CollisionData();manager.collide(obj,query,fcl.defaultCollisionCallback);return bool(query.result.is_collision)
import heapq

record=json.load(open(HERE/'comparison-physical-band.json'));output=[];rng=np.random.default_rng(19)

def tube(a,b,radius):
 a=np.array(a);b=np.array(b);v=b-a;length=np.linalg.norm(v)
 if length<1e-7:return fcl.CollisionObject(fcl.Sphere(radius),fcl.Transform(np.eye(3),a))
 z=v/length;x=np.cross(z,[0,0,1])
 if np.linalg.norm(x)<1e-5:x=np.array([1.,0,0])
 x/=np.linalg.norm(x);y=np.cross(z,x);R=np.column_stack([x,y,z]);return fcl.CollisionObject(fcl.Capsule(radius,length),fcl.Transform(R,(a+b)/2))

for layout in record['layouts']:
 if len(layout.get('recipes',{}))!=3:continue
 swept=[];shapes={s:[g.checker.shape(row,True)for row,obj in ROBOT[s]]+[shape for name,shape,r,p in g.shapes]for s in AD}
 for side in AD:
  for (name,o),shape in zip(objects(side,READY[side],.08)[0],shapes[side]):swept.append(fcl.CollisionObject(shape,fcl.Transform(o.getRotation(),o.getTranslation())))
 for part,recipe in layout['recipes'].items():
  side=recipe['side']
  for seg in recipe['segments']:
   if seg['label']in ('empty_approach','transfer'):continue  # free motions are replanned with the frame present
   for u in np.linspace(0,1,9):
    h=u**3*(10-15*u+6*u*u);q=np.array(seg['a'])*(1-h)+np.array(seg['b'])*h;obs,tcp,R=objects(side,q,.08 if part.startswith('ring')else .02)
    for (name,o),shape in zip(obs,shapes[side]):swept.append(fcl.CollisionObject(shape,fcl.Transform(o.getRotation(),o.getTranslation())))
    if seg['label']not in ('empty_approach','withdraw','release'):
     Rp=R@np.array(recipe['localR']).T;pos=tcp-Rp@np.array(recipe['localP']);swept.append(fcl.CollisionObject(fcl.Sphere(.038 if part=='ring1'else .032 if part=='ring2'else .020),fcl.Transform(np.eye(3),pos)))
 for row,o in g.checker.robot:
  if row['body'].startswith('official_FR3_Duo'):
   b=model.body(row['body']).id;R=data.xmat[b].reshape(3,3);pos=data.xpos[b]+R@row['position'];swept.append(fcl.CollisionObject(g.checker.shape(row,True),fcl.Transform(R@g.checker.rot(row['quaternion']),pos)))
 robot=fcl.DynamicAABBTreeCollisionManager();robot.registerObjects(swept);robot.setup();frames=[]
 def safe(a,b,r):
  o=tube(a,b,r);return not manager_hit(o,g.manager)and not manager_hit(o,robot)
 for part,recipe in layout['recipes'].items():
  R=np.array(recipe['R']);Rp=R@np.array(recipe['localR']).T;P=np.array(recipe['tcp'])-Rp@np.array(recipe['localP']);axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis);rad=.0375 if part=='ring1'else .031 if part=='ring2'else .008
  roots=[]
  if part=='pin':roots=[(P+np.array([0,0,.05]),None)]
  else:
   for angle in [0]:
    direction=Rotation.from_euler('z',angle,degrees=True).apply(axis)
    for radius in [rad+.065,rad+.085]:roots.append((P-direction*radius+[0,0,-.010],angle))
  available=[(r,a)for r,a in roots if safe(r,r,.004)]
  root,fork_angle=available[0] if available else roots[0];best=None
  if not available:
   print('ROUTE_ROOT_BLOCKED',layout['label'],part,root,flush=True);frames.append(dict(part=part,found=False));continue
  targets=[]
  for x in [-.9,-.6,-.3,0,.3,.6]:
   for z in [1.18,1.4,1.7,2.0,2.2]:
    goal=np.array([x,-1.15,z])
    if safe([x,-1.15,.04],goal,.02):targets.append(goal)
  targets.sort(key=lambda p:np.linalg.norm(root-p));points=[root];parents=[-1]
  for k in range(1500):
   goal=targets[k%len(targets)]if targets else None
   if goal is None:break
   target=goal if rng.random()<.35 else rng.uniform([-1.2,-1.15,.95],[.8,.35,2.3]);i=int(np.argmin([np.linalg.norm(p-target)for p in points]));a=points[i];dist=np.linalg.norm(target-a);b=a+(target-a)*min(1.,.1/max(dist,1e-9));r=.004 if min(np.linalg.norm(a-root),np.linalg.norm(b-root))<.25 else .0125
   if not safe(a,b,r):continue
   points.append(b);parents.append(i)
   if np.linalg.norm(b-goal)<.45 and safe(b,goal,.0125):
    chain=[goal,b];j=i
    while j>=0:chain.append(points[j]);j=parents[j]
    chain.reverse()
    # Shorten bends only if the replacement solid tube also clears geometry.
    for _ in range(200):
     if len(chain)<3:break
     i,j=sorted(rng.integers(0,len(chain),2));r=.004 if np.linalg.norm(chain[i]-root)<.25 else .0125
     if j>i+1 and safe(chain[i],chain[j],r):chain=chain[:i+1]+chain[j:]
    chain.append(np.array([goal[0],goal[1],.04]));best=dict(nodes=[p.tolist()for p in chain],head_root=root.tolist(),part_origin=P.tolist(),anchor=[float(goal[0]),-1.15,.04],small_tube_radius_m=.004,main_tube_radius_m=.0125,fork_angle_deg=fork_angle,empty_approach_must_be_replanned=True);break
  frames.append(dict(part=part,found=best is not None,best=best));print('ROUTED_FRAME',layout['label'],part,bool(best),'nodes',len(best['nodes'])if best else 0,flush=True)
 output.append(dict(layout=layout['label'],frames=frames,all_found=all(x['found']for x in frames),machine_attachment=False,structural_strength_verified=False,scope='rigid tube-to-CAD and sampled swept-robot/part keepout check'))
 (HERE/'routed-independent-frames.json').write_text(json.dumps(output,indent=2))
