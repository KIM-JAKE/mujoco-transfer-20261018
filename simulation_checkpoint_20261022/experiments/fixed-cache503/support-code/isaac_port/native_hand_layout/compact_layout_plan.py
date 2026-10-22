"""Plan small fixed holders on an independent table, then compare scheduled paths.
Sampled rigid geometry and kinematic timing, not a hardware deployment recipe.
"""
from compact_table_search import *
import os

def candidate_pieces(c,top_z=1.1):
 spec=copy.deepcopy(c['geometry']);rows=[];feet=[]
 for a in spec:
  if a['name'].endswith('_post'):
   a['b'][2]=top_z+.004
   feet.append(dict(name=a['name']+'_foot',type='box',pos=[a['b'][0],a['b'][1],top_z+.002],size=[.024,.024,.004],R=np.eye(3).tolist()))
  if a['type']=='box':rows.append(shape_box(a['name'],a['pos'],a['size'],np.array(a['R'])))
  elif a['type']=='capsule':rows.append(tube(a['name'],a['a'],a['b'],a['radius']))
  elif a['type']=='sphere':rows.append((a['name'],fcl.CollisionObject(fcl.Sphere(a['radius']+(.003 if a.get('passive_gate')else 0)),fcl.Transform(np.eye(3),np.array(a['pos'])))))
  else:rows.append((a['name'],fcl.CollisionObject(fcl.Cylinder(a['radius'],a['length']),fcl.Transform(np.array(a['R']),np.array(a['pos'])))))
 for a in feet:rows.append(shape_box(a['name'],a['pos'],a['size'],np.eye(3)))
 return rows,spec+feet

def seg(a,b,label,part,gap):return dict(a=np.array(a).tolist(),b=np.array(b).tolist(),duration_s=segment_time(a,b),label=label,part=part,gap_m=gap)

def route(side,a,b,gap,limit=180,allow_search=True):
 a=np.array(a);b=np.array(b)
 disk=Path(os.environ.get('M80_ROUTE_CACHE',str(HERE/'repaired-routes-cache.json')));key=json.dumps([side,np.round(a,8).tolist(),np.round(b,8).tolist(),round(gap,7)])
 memory=json.load(open(disk))if disk.exists()else{}
 for previous in memory.get(key,[]):
  path=[np.array(x)for x in previous]
  if path_clear(side,path,gap,31)[0]:return path
 if path_clear(side,[a,b],gap,41)[0]:return[a,b]
 # Neighbouring cells differ by only a few centimetres. Reuse a screened
 # corridor, checking both endpoint joins and the complete new fixture scene.
 nearby=[]
 for previous_key,paths in memory.items():
  other_side,pa,pb,other_gap=json.loads(previous_key)
  if other_side!=side or abs(other_gap-gap)>1e-7:continue
  for reverse in [False,True]:
   first=np.array(pb if reverse else pa);last=np.array(pa if reverse else pb);da=np.linalg.norm(a-first);db=np.linalg.norm(b-last)
   if da<2. and db<2.:nearby.append((da+db,paths,reverse))
 for _,paths,reverse in sorted(nearby,key=lambda row:row[0])[:4]:
  for previous in paths:
   old=[np.array(x)for x in (previous[::-1]if reverse else previous)];chain=[a]+old+[b];chain=[q for i,q in enumerate(chain)if i==0 or np.linalg.norm(q-chain[i-1])>1e-7]
   if path_clear(side,chain,gap,41)[0]:return chain
 cached_path=HERE/'compact-path-candidates.json'
 if cached_path.exists():
  cached=json.load(open(cached_path))
  for rows in cached.values():
   for row in rows[:1]:
    r=row['recipe']
    if r['side']!=side:continue
    label='empty_approach'if np.linalg.norm(a-READY[side])<1e-5 else'transfer'
    spans=[s for s in r['segments']if s['label']==label]
    if not spans:continue
    chain=[a]+[np.array(spans[0]['a'])]+[np.array(s['b'])for s in spans]+[b]
    chain=[q for i,q in enumerate(chain)if i==0 or np.linalg.norm(q-chain[i-1])>1e-7]
    if path_clear(side,chain,gap,41)[0]:return chain
 for middle in [READY[side]]:
  if path_clear(side,[a,middle,b],gap,41)[0]:return[a,middle,b]
 if not allow_search:return None
 for seed in [7,21,45]:
  p=rrt(side,a,b,gap,seed=seed,max_nodes=limit)
  if p and path_clear(side,p,gap,41)[0]:
   memory.setdefault(key,[]).append([q.tolist()for q in p]);temp=disk.with_suffix('.'+str(os.getpid())+'.tmp');temp.write_text(json.dumps(memory));temp.replace(disk);return p
 return None

def recipe_for(c,extra,start=None):
 r=copy.deepcopy(c['recipe']);part=r['part'];b=BANDS[part];side=r['side'];R=np.array(r['R']);tcp=np.array(r['tcp']);q=np.array(r['q_pick']);gap=r['gap'];op=.08 if part!='pin'else .02
 set_env(extra)
 pre=None
 for delta in [np.array([0,0,-.025]),-R[:,2]*.025,-R[:,2]*.05]:
  trial=ik(side,tcp+delta,R,q)
  if trial is not None and path_clear(side,[trial,q],op,21)[0]:pre=trial;break
 if pre is None:return None,'pregrasp'
 axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis)
 if axis[1]>0:axis=-axis
 qe=ik(side,tcp+axis*.1,R,q);ql=ik(side,tcp+axis*.1+[0,0,.04],R,qe)if qe is not None else None
 if ql is None:return None,'extract_ik'
 if not path_clear(side,[q,qe,ql],gap,31)[0]:return None,'extract_collision'
 target=np.array(b['q_preinsert']);qi=np.array(b['q_insert']);transfer=route(side,ql,target,gap)
 if transfer is None:return None,'transfer'
 if not path_clear(side,[target,qi],gap,41)[0]:return None,'insert'
 if not path_clear(side,[qi,target],op,41)[0]:return None,'withdraw'
 start=READY[side]if start is None else np.array(start);entry=route(side,start,pre,op)
 if entry is None:return None,'entry'
 segments=[]
 for a,z in zip(entry[:-1],entry[1:]):segments.append(seg(a,z,'empty_approach',part,op))
 segments.append(seg(pre,q,'pregrasp',part,op));segments.append(dict(a=q.tolist(),b=q.tolist(),duration_s=max(.18,(op-gap)/.1+.08),label='grasp',part=part,gap_m=gap))
 if part=='pin':segments.append(dict(a=q.tolist(),b=q.tolist(),duration_s=.15,label='clip_open',part=part,gap_m=gap))
 path=[q,qe,ql]+transfer[1:]+[qi]
 for i,(a,z)in enumerate(zip(path[:-1],path[1:])):segments.append(seg(a,z,'extract'if i==0 else'lift'if i==1 else'insert'if i==len(path)-2 else'transfer',part,gap))
 segments.append(dict(a=qi.tolist(),b=qi.tolist(),duration_s=.18,label='clamp',part=part,gap_m=gap));segments.append(dict(a=qi.tolist(),b=qi.tolist(),duration_s=max(.18,(op-gap)/.1+.05),label='release',part=part,gap_m=op));segments.append(seg(qi,target,'withdraw',part,op))
 duration=sum(s['duration_s']for s in segments)
 if duration>25:return None,'near_position_velocity_limit'
 r.update(segments=segments,q_exit=target.tolist(),q_path=[a.tolist()for a in path],duration_s=duration,pregrasp_delta_m=delta.tolist());return r,None

def main():
 candidates=json.load(open(HERE/'compact-head-candidates.json'));tables=json.load(open(HERE/'compact-tables.json'));out=[];chosen={}
 for cx in [-.25,-.35,-.45]:
  t=dict(center=[cx,-.25,1.1]);desk=table(*t['center']);t['hits']=machine_hits(desk);t['valid']=not t['hits']and endpoint_clear(desk);tables.append(t)
 (HERE/'compact-tables.json').write_text(json.dumps(tables,indent=2))
 # Shortest endpoint lower-bound candidates are attempted first. Table supports
 # are kept below the fingers, and never attached to the original machine.
 for part in ['ring1','ring2','pin']:
  opts=[x for x in candidates if x['part']==part];wins=[]
  for c in opts:
   r=c['recipe'];head,_=candidate_pieces(c);r0=np.array(r['tcp']);post=next(x for x in c['geometry']if x['name']==part+'_post');foot=np.array(post['b']);valid_tables=[t for t in tables if t['valid']and t['center'][2]==1.1 and abs(foot[0]-t['center'][0])<.24 and abs(foot[1]-t['center'][1])<.165]
   if not valid_tables:continue
   t=min(valid_tables,key=lambda x:abs(x['center'][0]+.3)+abs(x['center'][1]+.25));desk=table(*t['center']);r,why=recipe_for(c,desk+head,start=BANDS['ring2']['q_preinsert']if part=='pin'else None)
   print('COMPACT_RECIPE',part,c['recipe']['tcp'],bool(r),why,flush=True)
   out.append(dict(part=part,tcp=c['recipe']['tcp'],valid=r is not None,reject=why))
   if r is not None:
    wins.append(dict(candidate=c,recipe=r,table=t,geometry=candidate_pieces(c)[1]));chosen[part]=wins;(HERE/'compact-path-candidates.json').write_text(json.dumps(chosen,indent=2))
    if len(wins)>=3:break
  if not wins:break
 (HERE/'compact-path-attempts.json').write_text(json.dumps(out,indent=2));print('COMPACT_PATH_COUNTS',{k:len(v)for k,v in chosen.items()},flush=True)
if __name__=='__main__':main()
