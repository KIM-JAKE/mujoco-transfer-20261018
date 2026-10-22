"""Search drop-and-retract fork motions after measured grasp; static metal stays fixed."""
from finalize_fixture_paths import *
record=json.load(open(HERE/'comparison-physical-band.json'));layout=next(x for x in record['layouts']if x['label']=='near_front');frame=next(x for x in json.load(open(HERE/'routed-independent-frames.json'))if x['layout']=='near_front');heads=fixtures(frame,layout['recipes']);results=[]
future=[]
for part,recipe in layout['recipes'].items():
 for seg in recipe['segments']:
  if seg['label']in ('empty_approach','transfer','extract','lift'):continue
  side=recipe['side'];shape_list=[g.checker.shape(row,True)for row,o in ROBOT[side]]+[shape for name,shape,R,P in g.shapes]
  for u in np.linspace(0,1,6):
   h=u**3*(10-15*u+6*u*u);q=np.array(seg['a'])*(1-h)+np.array(seg['b'])*h
   for (name,o),shape in zip(objects(side,q,.08 if part.startswith('ring')else .02)[0],shape_list):future.append(fcl.CollisionObject(shape,fcl.Transform(o.getRotation(),o.getTranslation())))
manager=fcl.DynamicAABBTreeCollisionManager();manager.registerObjects(future);manager.setup()

def collides_manager(obj,mgr):
 d=fcl.CollisionData();mgr.collide(obj,d,fcl.defaultCollisionCallback);return bool(d.result.is_collision)
for part in ['ring1','ring2']:
 recipe=layout['recipes'][part];rows=[(name,o,o.getTranslation().copy(),o.getRotation().copy())for name,o in heads if name in (part+'_fork',part+'_spine')];own=objects(recipe['side'],np.array(recipe['q_pick']),recipe['gap'])[0];R=np.array(recipe['R']);axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis);cross=np.cross([0,0,1],axis);valid=[];tested=0
 for drop in [.008,.010,.012,.015,.020]:
  for direction in [-axis,axis,cross,-cross,(-axis-cross)/np.sqrt(2),(axis-cross)/np.sqrt(2)]:
   for stroke in [.06,.08,.10,.12,.16]:
    delta=direction*stroke+[0,0,-drop];okay=True
    # Retracted head must clear all subsequent robot poses in this conservative union.
    for name,o,p,r in rows:
     o.setTransform(fcl.Transform(r,p+delta))
     if collides_manager(o,g.manager)or collides_manager(o,manager):okay=False;break
    if not okay:continue
    offsets=[np.array([0,0,-drop*u])for u in np.linspace(0,1,9)]+[direction*stroke*u+[0,0,-drop]for u in np.linspace(0,1,17)]
    for shift in offsets:
     for name,o,p,r in rows:
      o.setTransform(fcl.Transform(r,p+shift))
      if collides_manager(o,g.manager)or any(pairhit(o,body)for _,body in own):okay=False;break
     if not okay:break
    tested+=1
    if okay:
     duration=max(.08,1.875*drop/.15)+max(.10,1.875*stroke/.5)+.04;row=dict(part=part,drop_m=drop,horizontal_delta_m=(direction*stroke).tolist(),stowed_delta_m=delta.tolist(),duration_s=duration,stroke_m=stroke);valid.append(row);print('RETIRE_HEAD',row,flush=True)
  if valid:break
 best=min(valid,key=lambda x:x['duration_s'])if valid else None;results.append(dict(part=part,best=best,tested_clear_endpoint_candidates=tested));(HERE/'retracting-heads.json').write_text(json.dumps(results,indent=2))
