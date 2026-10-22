"""Screen/repair nominal CAD payload transfers against the complete static cell.
Stored parts and other installed parts are conservatively present together.
Grasp contacts/insertion forces remain outside this rigid-path test.
"""
from payload_shapes import shape_for,bounds_for
from five_layout_search import *
from contextlib import contextmanager
from fast_environment import FixedScene
import sys,hashlib
SHAPES={p:shape_for(p)for p in ['ring1','ring2','pin']}
_SCENES={}

@contextmanager
def margin_mode(coarse):
 old=getattr(g,'layout_collision_search_mode',False);mode=getattr(g,'layout_payload_collision_only',False);token=getattr(g,'layout_payload_token',None);definition=getattr(g,'layout_payload_definition',None)
 g.layout_collision_search_mode=coarse;g.layout_payload_collision_only=coarse
 if definition is not None:g.layout_payload_token=(definition,coarse)
 try:yield
 finally:g.layout_collision_search_mode=old;g.layout_payload_collision_only=mode;g.layout_payload_token=token

@contextmanager
def carrying(recipe,coarse=False):
 old=g.shapes;token=getattr(g,'layout_payload_token',None);bounds=getattr(g,'layout_payload_bounds',None);mode=getattr(g,'layout_payload_collision_only',False);scene_mode=getattr(g,'layout_collision_search_mode',False);definition=getattr(g,'layout_payload_definition',None);L=np.array(recipe['localR']);lp=np.array(recipe['localP'])
 g.shapes=list(old)+[('payload',SHAPES[recipe['part']],L.T,np.array([0,0,.1034])-L.T@lp)]
 g.layout_payload_definition=json.dumps([recipe['part'],recipe['localP'],recipe['localR']]);g.layout_payload_token=(g.layout_payload_definition,coarse);g.layout_payload_collision_only=coarse;g.layout_collision_search_mode=coarse
 g.layout_payload_bounds=bounds_for(recipe['part'])
 try:yield
 finally:g.shapes=old;g.layout_payload_token=token;g.layout_payload_bounds=bounds;g.layout_payload_collision_only=mode;g.layout_collision_search_mode=scene_mode;g.layout_payload_definition=definition

def obstacles(row,recipe=None):
 held=recipe['part']if recipe else None
 signature=hashlib.sha256(json.dumps([row['geometry'],row['stock_parts'],row['table_center_m'],row['table_size_m']],sort_keys=True).encode()).hexdigest();key=(signature,held)
 if key in _SCENES:return _SCENES[key]
 pieces=table(*row['table_center_m'])+pieces_from_specs([s for s in row['geometry']if not s.get('render_only')])
 for stock in row['stock_parts']:
  # Retain the complete loaded supply conservatively throughout transfers.
  # This also makes shared trajectory sections reusable across all five cells.
  pieces.append(('stored_'+stock['part']+'_'+str(stock['index']),fcl.CollisionObject(SHAPES[stock['part']],fcl.Transform(np.array(stock['R']),np.array(stock['P'])))))
 for part,b in BANDS.items():
  if part==held:continue
  tcp,R=fk('right'if part=='ring1'else'left',np.array(b['q_insert']));Rp=R@np.array(b['localR']).T;P=tcp-Rp@np.array(b['localP']);pieces.append(('installed_'+part,fcl.CollisionObject(SHAPES[part],fcl.Transform(Rp,P))))
 pieces=FixedScene(pieces,key);_SCENES[key]=pieces;return pieces

def payload_bridge(recipe,a,b):
 side=recipe['side'];gap=recipe['gap'];pa,Ra=fk(side,a);pb,Rb=fk(side,b)
 if not path_clear(side,[a,a],gap,2)[0]or not path_clear(side,[b,b],gap,2)[0]:return None
 for height in [pa[2]+.02,pa[2]+.06,1.53,1.58]:
  high=robust(side,pa+[0,0,height-pa[2]],Ra,a)
  if high is None or not path_clear(side,[a,high],gap,41)[0]:continue
  for y in [0.,-.05,.04]:
   point=np.array([pb[0],y,height]);qa=robust(side,point,Ra,high);qb=robust(side,point,Rb,b)
   if qa is None or qb is None:continue
   if not path_clear(side,[high,qa],gap,61)[0]or not path_clear(side,[qb,b],gap,61)[0]:continue
   rv=Rotation.from_matrix(Ra.T@Rb).as_rotvec();senses=[1,-1]if abs(np.linalg.norm(rv)-np.pi)<.001 else[1]
   for sense in senses:
    chain=[a,high,qa];okay=True
    for u in np.linspace(0,1,13)[1:]:
     R=Rb if u==1 else Ra@Rotation.from_rotvec(sense*u*rv).as_matrix();seed=chain[-1].copy();seed[0]=(1-u)*qa[0]+u*qb[0];q=robust(side,point,R,seed)
     if q is None or not path_clear(side,[chain[-1],q],gap,17)[0]:okay=False;break
     chain.append(q)
    if okay and path_clear(side,[chain[-1],qb,b],gap,41)[0]:
     chain.extend([qb,b]);print('PAYLOAD_CARTESIAN_ESCAPE',recipe['part'],height,y,sense,flush=True);return chain
 return None

def audit_recipe(row,r):
 repaired=[];issues=[];original=r['segments'];prepared=[];cursor=0
 while cursor<len(original):
  s=original[cursor];end=cursor+1
  if s['label']!='transfer':prepared.append(s);cursor=end;continue
  while end<len(original)and original[end]['label']=='transfer':end+=1
  spans=original[cursor:end];chain=[np.array(spans[0]['a'])]+[np.array(x['b'])for x in spans];set_env(obstacles(row,r))
  # Search uses exact collision predicates. Final per-segment validation below
  # retains the 0.1 mm payload distance reserve, avoiding costly distance-based
  # random-tree expansion without weakening the final acceptance criterion.
  with carrying(r,coarse=True):
   okay,why=path_clear(r['side'],chain,r['gap'],61)
   if not okay:
    print('PAYLOAD_GROUP_REPAIR',r['part'],why,flush=True);replacement=payload_bridge(r,chain[0],chain[-1])
    if replacement is None:replacement=route(r['side'],chain[0],chain[-1],r['gap'],limit=350)
    if replacement is not None:spans=[seg(a,b,'transfer',r['part'],r['gap'])for a,b in zip(replacement[:-1],replacement[1:])]
    else:issues.append(dict(label='transfer_group',why=why))
  prepared+=spans;cursor=end
 r['segments']=prepared
 for index,s in enumerate(r['segments']):
  if s['label']not in('transfer','empty_approach','withdraw','insert'):
   repaired.append(s);continue
  set_env(obstacles(row,r if s['label']in('transfer','insert')else None));context=carrying(r)if s['label']=='transfer'else None
  if context:context.__enter__()
  try:
   a=np.array(s['a']);b=np.array(s['b']);count=max(41,int(np.ceil(np.max(abs(b-a))/.01))+1);okay,why=path_clear(r['side'],[a,b],s['gap_m'],count)
   if not okay and s['label']!='withdraw':
    print('PAYLOAD_REPAIR',row['label'],r['part'],index,s['label'],why,flush=True);path=None
    if s['label']=='transfer':
     P,R=fk(r['side'],a)
     for rise in [.02,.04,.06]:
      high=robust(r['side'],P+[0,0,rise],R,a)
      if high is not None and path_clear(r['side'],[a,high,b],s['gap_m'],61)[0]:path=[a,high,b];break
    if path is None:path=route(r['side'],a,b,s['gap_m'],limit=550)
    if path is not None:
     repaired.extend(seg(x,z,s['label'],r['part'],s['gap_m'])for x,z in zip(path[:-1],path[1:]));continue
   if not okay:issues.append(dict(segment=index,label=s['label'],why=why))
   repaired.append(s)
  finally:
   if context:context.__exit__(None,None,None)
 if not issues:
  smooth=[];index=0
  while index<len(repaired):
   s=repaired[index];end=index+1
   if s['label']not in('transfer','empty_approach'):smooth.append(s);index=end;continue
   while end<len(repaired)and repaired[end]['label']==s['label']:end+=1
   group=repaired[index:end];path=[np.array(group[0]['a'])]+[np.array(x['b'])for x in group];set_env(obstacles(row,r if s['label']=='transfer'else None));context=carrying(r,coarse=True)if s['label']=='transfer'else margin_mode(True)
   context.__enter__()
   try:
    k=0
    while k<len(path)-1:
     chosen=k+1
     for z in range(len(path)-1,k+1,-1):
      count=max(41,int(np.ceil(np.max(abs(path[z]-path[k]))/.01))+1)
      if segment_time(path[k],path[z])>sum(x['duration_s']for x in group[k:z])+1e-9:continue
      if not path_clear(r['side'],[path[k],path[z]],s['gap_m'],count)[0]:continue
      with margin_mode(False):fine=path_clear(r['side'],[path[k],path[z]],s['gap_m'],count)[0]
      if fine:chosen=z;break
     smooth.append(seg(path[k],path[chosen],s['label'],r['part'],s['gap_m']));k=chosen
   finally:
    context.__exit__(None,None,None)
   index=end
  print('PAYLOAD_SMOOTHED',r['part'],sum(s['duration_s']for s in repaired),sum(s['duration_s']for s in smooth),flush=True);repaired=smooth
 r['segments']=repaired;r['duration_s']=sum(s['duration_s']for s in repaired)
 holding=[s for s in repaired if s['label']in('clear_datum','extract','lift','transfer','insert')]
 if holding:
  chain=[holding[0]['a']]+[s['b']for s in holding];r['q_path']=[q for i,q in enumerate(chain)if i==0 or np.linalg.norm(np.array(q)-chain[i-1])>1e-9]
 cache=HERE/'audited-recipes';cache.mkdir(exist_ok=True);key=hashlib.sha256(json.dumps([row['geometry'],row['stock_parts'],row['table_center_m'],r['part'],r['tcp'],r['localP'],r['localR']],sort_keys=True).encode()).hexdigest()[:24];(cache/(key+'.json')).write_text(json.dumps(dict(label=row['label'],recipe=r,issues=issues,geometry_scope='exact CAD; 0.1 mm payload reserve after collision-only planning'),indent=2))
 return issues

if __name__=='__main__':
 path=Path(sys.argv[1]);row=json.load(open(path))
 if not row.get('individual_paths_complete'):raise SystemExit('Individual paths incomplete; no nominal task claim.')
 issues=[]
 for part in ['ring1','ring2']:issues+= [dict(part=part,**x)for x in audit_recipe(row,row['recipes'][part])]
 for slot in row['pin_slots']:issues+=[dict(part='pin',slot=slot['slot'],**x)for x in audit_recipe(row,slot['recipe'])]
 row.update(nominal_payload_paths_pass=not issues,payload_path_issues=issues,payload_audit_scope='Transfers with rigid nominal CAD payload; empty approach and withdrawal against stored and installed parts. Not a force-based assembly test.')
 row['recipes']['pin']=copy.deepcopy(row['pin_slots'][0]['recipe']);out=path.with_name(path.stem+'-payload.json');out.write_text(json.dumps(row,indent=2));print('PAYLOAD_AUDIT_DONE',row['label'],not issues,len(issues),out,flush=True)
