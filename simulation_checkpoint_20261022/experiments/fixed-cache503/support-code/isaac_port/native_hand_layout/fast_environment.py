"""Cache unchanged robot/M80 checks while checking every candidate's new solids.
Keys include side, joint configuration, aperture; no moving environment is cached.
"""
import numpy as np,fcl
import layout_search as ls
import evaluate_layouts as ev
import compact_table_search as csp
import compact_layout_plan as cp
_original_clear=ls.clear
_baseline=fcl.DynamicAABBTreeCollisionManager();_baseline.registerObjects(csp.ORIGINAL);_baseline.setup()
_props=fcl.DynamicAABBTreeCollisionManager();_props.setup();_cache={};_prop_rows=[]
_scene_registry={};_prop_result_cache=None
class FixedScene(list):
 def __init__(self,rows,cache_token):super().__init__(rows);self.cache_token=cache_token
_base_shapes=[]
for row,obj in ls.g.checker.robot:
 if row['body'].startswith('official_FR3_Duo'):
  # Fixed Duo brackets are concave. Their complete CAD triangle meshes retain
  # the openings; a single convex hull incorrectly fills those openings.
  b=ls.model.body(row['body']).id;R=ls.data.xmat[b].reshape(3,3);P=ls.data.xpos[b]+R@row['position'];_base_shapes.append(fcl.CollisionObject(ls.g.checker.shape(row,False),fcl.Transform(R@ls.g.checker.rot(row['quaternion']),P)))
_base=fcl.DynamicAABBTreeCollisionManager();_base.registerObjects(_base_shapes);_base.setup()

def set_env(pieces):
 global _props,_prop_rows,_prop_result_cache
 token=getattr(pieces,'cache_token',None)
 if token is not None and token in _scene_registry:
  _props,_prop_rows,_prop_result_cache=_scene_registry[token];ls.g.manager=_baseline;return
 _prop_rows=pieces
 ls.g.manager=_baseline
 _props=fcl.DynamicAABBTreeCollisionManager();_props.registerObjects([o for _,o in pieces]);_props.setup()
 _prop_result_cache={}if token is not None else None
 if token is not None:_scene_registry[token]=(_props,_prop_rows,_prop_result_cache)

def remember(key,result):
 if _prop_result_cache is not None:_prop_result_cache[key]=result
 return result

def clear(side,q,gap,extra=None,margin=.0001):
 if extra:return _original_clear(side,q,gap,extra,margin)
 key=(side,tuple(np.round(q,10)),round(gap,10),margin,getattr(ls.g,'layout_payload_token',None),getattr(ls.g,'layout_collision_search_mode',False),ls.BBOX_SCREEN_ENABLED)
 if _prop_result_cache is not None and key in _prop_result_cache:return _prop_result_cache[key]
 result=_cache.get(key)
 if result is None:
  ls.g.manager=_baseline;result=_original_clear(side,q,gap,margin=margin)
  if result[0]:
   for name,obj in ls.objects(side,q,gap)[0]:
    if name.endswith('link0'):continue
    reserve=ls.clearance_margin(name,margin);distance=ls.threshold_distance(_base,obj,reserve)
    if distance<reserve:result=(False,[name,'fixed_duo_base',float(distance)]);break
  _cache[key]=result
 if not result[0]:return remember(key,result)
 obs,_,_=ls.objects(side,q,gap)
 for name,obj in obs:
  if name.endswith('link0'):continue
  threshold=ls.clearance_margin(name,margin);distance=ls.threshold_distance(_props,obj,threshold)
  if distance<threshold:
   culprit='new_fixture';reported_distance=distance
   for label,other in _prop_rows:
    hit=ls.pairhit(obj,other)if threshold<=0 else fcl.distance(other,obj,fcl.DistanceRequest(),fcl.DistanceResult())<threshold
    if hit:culprit=label;break
   return remember(key,(False,[name,culprit,float(reported_distance)]))
 return remember(key,(True,None))

def install():
 ls.clear=ev.clear=csp.clear=cp.clear=clear
 csp.set_env=cp.set_env=set_env
 return set_env
