"""Find independent floor-supported frame routes without attaching to the M80."""
from evaluate_layouts import *


def box(a,b,width,label):
 a=np.array(a,float);b=np.array(b,float);delta=b-a;length=np.linalg.norm(delta)
 if length<1e-6:return None
 z=delta/length;x=np.cross(z,[0,0,1])
 if np.linalg.norm(x)<1e-5:x=np.array([1,0,0.])
 x/=np.linalg.norm(x);y=np.cross(z,x);R=np.column_stack([x,y,z]);size=np.array([width,width,length]);pos=(a+b)/2
 return dict(label=label,pos=pos.tolist(),R=R.tolist(),size=size.tolist()),fcl.CollisionObject(fcl.Box(*size),fcl.Transform(R,pos))


def manager_hit(obj,manager):
 query=fcl.CollisionData();manager.collide(obj,query,fcl.defaultCollisionCallback);return bool(query.result.is_collision)


record=json.load(open(HERE/'comparison-physical-band.json'))if (HERE/'comparison-physical-band.json').exists()else json.load(open(HERE/'comparison.json'));output=[]
for layout in record['layouts']:
 if len(layout.get('recipes',{}))!=3:continue
 # Sample all moving robot geometry so the support frame does not occupy a
 # planned swept region; no certified continuous collision guarantee is implied.
 swept=[]
 for part,recipe in layout['recipes'].items():
  side=recipe['side']
  for seg in recipe['segments']:
   for u in np.linspace(0,1,7):
    h=u**3*(10-15*u+6*u*u);q=np.array(seg['a'])*(1-h)+np.array(seg['b'])*h
    shapes=[g.checker.shape(row,True)for row,obj in ROBOT[side]]+[shape for name,shape,r,p in g.shapes]
    for (name,o),shape in zip(objects(side,q,.08 if part.startswith('ring')else .02)[0],shapes):
     # Copy mutable arm collision objects before the next pose overwrites them.
     swept.append(fcl.CollisionObject(shape,fcl.Transform(o.getRotation(),o.getTranslation())))
 robot_manager=fcl.DynamicAABBTreeCollisionManager();robot_manager.registerObjects(swept);robot_manager.setup();frames=[]
 for part,recipe in layout['recipes'].items():
  R=np.array(recipe['R']);Rp=R@np.array(recipe['localR']).T;P=np.array(recipe['tcp'])-Rp@np.array(recipe['localP']);axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis);radius=.0375 if part=='ring1'else .031 if part=='ring2'else .008
  root=P-axis*(radius+.065)+[0,0,-(.010 if part!='pin'else -.006)];found=[]
  for x in [-.9,-.7,-.5,-.3,-.1,.1,.3,.5,.7]:
   for z in [1.18,1.28,1.38,1.65,1.85,2.05,2.25]:
    anchor=np.array([x,-1.15,.04]);top=np.array([x,-1.15,z])
    for mode in [0,1]:
     elbow=np.array([x,root[1],z])if mode==0 else np.array([root[0],-1.15,z]);last=np.array([root[0],root[1],z]);nodes=[anchor,top,elbow,last,root];pieces=[];okay=True
     for i,(a,b)in enumerate(zip(nodes[:-1],nodes[1:])):
      item=box(a,b,.04 if i==0 else .025 if i<3 else .010,part+'_frame'+str(i))
      if item is None:continue
      row,obj=item
      if manager_hit(obj,g.manager)or manager_hit(obj,robot_manager):okay=False;break
      pieces.append(row)
     if okay:
      plate=dict(label=part+'_floor_base',pos=[x,-1.15,.02],R=np.eye(3).tolist(),size=[.35,.35,.04]);po=fcl.CollisionObject(fcl.Box(*plate['size']),fcl.Transform(np.array(plate['R']),np.array(plate['pos'])))
      if manager_hit(po,g.manager):continue
      pieces.append(plate);cost=sum(np.linalg.norm(np.array(b)-np.array(a))for a,b in zip(nodes[:-1],nodes[1:]));found.append(dict(cost=cost,boxes=pieces,root=root.tolist(),part_origin=P.tolist(),anchor=anchor.tolist()))
   if found:break
  best=min(found,key=lambda a:a['cost'])if found else None;frames.append(dict(part=part,found=best is not None,best=best));print('FRAME',layout['label'],part,'found',bool(best),flush=True)
 output.append(dict(layout=layout['label'],frames=frames,all_found=all(f['found']for f in frames),machine_contact_or_attachment=False,scope='CAD box route / sampled robot swept volumes; strength and vibration not evaluated'))
 (HERE/'independent-support-frames.json').write_text(json.dumps(output,indent=2))
