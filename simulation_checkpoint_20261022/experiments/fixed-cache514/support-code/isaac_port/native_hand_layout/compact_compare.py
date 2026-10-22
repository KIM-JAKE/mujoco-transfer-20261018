"""Combine compact heads, enforce common table and sample both arms together."""
from compact_layout_plan import *
import heapq
from model_torque_retime import screen as model_screen
from fast_environment import install
set_env=install()

def serialize(rows,geometry,table_center):
 return dict(label='compact_table',recipes=rows,geometry=geometry,table_center_m=table_center,table_size_m=[.5,.35],common_start_q={s:READY[s].tolist()for s in AD},hardware_ready=False,full_physics_assembly_tested=False)

_PAIR_CACHE={}
def arm_pair_clear(left,right,margin=.001,left_payload=None,right_payload=None):
 # Exact repeated states recur across the five slots and DAG edges. Store only
 # their collision result; no pose rounding or geometry simplification is used.
 def payload_key(value):
  return None if value is None else (value['part'],tuple(np.asarray(value['localP']).ravel()),tuple(np.asarray(value['localR']).ravel()))
 key=(tuple(left[0]),float(left[1]),tuple(right[0]),float(right[1]),margin,payload_key(left_payload),payload_key(right_payload),getattr(g,'layout_payload_token',None))
 if key in _PAIR_CACHE:return _PAIR_CACHE[key]
 result=_arm_pair_clear(left,right,margin,left_payload,right_payload)
 if len(_PAIR_CACHE)>200000:_PAIR_CACHE.clear()
 _PAIR_CACHE[key]=result
 return result

def _arm_pair_clear(left,right,margin=.001,left_payload=None,right_payload=None):
 from layout_search import OBJECT_BOUNDS,OBJECT_OBBS
 obs=[];aabbs=[];obbs=[]
 for side,state in [('left',left),('right',right)]:
  q,gap=state;rows=objects(side,q,gap)[0];payload=left_payload if side=='left'else right_payload
  if payload:
   from payload_shapes import shape_for,bounds_for
   from layout_search import register_bounds
   tcp,R=fk(side,q);Rp=R@np.array(payload['localR']).T;P=tcp-Rp@np.array(payload['localP']);obj=fcl.CollisionObject(shape_for(payload['part']),fcl.Transform(Rp,P));rows.append((side+'_payload',obj));register_bounds(obj,Rp,P,bounds_for(payload['part']))
  obs.append(rows)
  aabbs.append({id(obj):OBJECT_BOUNDS[id(obj)]for _,obj in rows})
  obbs.append({id(obj):OBJECT_OBBS[id(obj)]for _,obj in rows})
 if os.environ.get('M80_PAIR_OBB_SCREEN')=='1':
  A=[(name,obj)for name,obj in obs[0]if not name.endswith('link0')];B=[(name,obj)for name,obj in obs[1]if not name.endswith('link0')]
  alo=np.asarray([aabbs[0][id(o)][0]for _,o in A]);ahi=np.asarray([aabbs[0][id(o)][1]for _,o in A]);blo=np.asarray([aabbs[1][id(o)][0]for _,o in B]);bhi=np.asarray([aabbs[1][id(o)][1]for _,o in B])
  separation=np.maximum(np.maximum(alo[:,None]-bhi[None,:],blo[None,:]-ahi[:,None]),0.)
  for i,j in np.argwhere(np.sum(separation**2,axis=2)<=(margin+1e-9)**2):
   a,b=A[i][1],B[j][1];ca,Ra,ha=obbs[0][id(a)];cb,Rb,hb=obbs[1][id(b)]
   axes=np.concatenate([Ra.T,Rb.T,np.cross(Ra.T[:,None,:],Rb.T[None,:,:]).reshape(9,3)])
   length=np.linalg.norm(axes,axis=1);projected=np.abs(axes@(cb-ca))-np.abs(axes@Ra)@ha-np.abs(axes@Rb)@hb
   # A separating projection greater than the reserve is a lower bound on
   # distance between the enclosed CAD shapes. Otherwise use original FCL.
   if np.any(projected>margin*length+1e-9):continue
   if fcl.collide(a,b,fcl.CollisionRequest(),fcl.CollisionResult())or fcl.distance(a,b,fcl.DistanceRequest(),fcl.DistanceResult())<margin:return False
  return True
 manager=fcl.DynamicAABBTreeCollisionManager();manager.registerObjects([o for name,o in obs[0]if not name.endswith('link0')]);manager.setup()
 for name,obj in obs[1]:
  if name.endswith('link0'):continue
  from layout_search import threshold_distance
  if threshold_distance(manager,obj,margin)<margin:return False
 return True

def state_at(seg,t):
 u=np.clip(t/seg['duration_s'],0,1);h=u**3*(10-15*u+6*u*u)
 first=seg.get('gap_start_m',seg['gap_m']);last=seg.get('gap_finish_m',seg['gap_m']);width_time=abs(last-first)/.095
 aperture=first+(last-first)*np.clip(t/max(width_time,1e-9),0,1)if first!=last else first
 return np.array(seg['a'])*(1-h)+np.array(seg['b'])*h,float(aperture)

def annotate_apertures(lane,start=.08):
 """Represent closing and opening sweeps, including the final resting width."""
 previous=start
 for s in lane:
  end=s['gap_m']
  if s['label']=='close_empty_hand_for_button':end=0.
  elif s['label']=='pin_preshape':end=.02
  s['gap_start_m']=previous;s['gap_finish_m']=end
  if s['label']not in ('grasp','release','close_empty_hand_for_button','pin_preshape','open_hand_for_ring'):s['gap_start_m']=end
  previous=end
 return lane

def coordinate(lanes,dt=.05,initial_widths=None):
 for side,lane in lanes.items():annotate_apertures(lane,(initial_widths or{}).get(side,.08))
 L,R=lanes['left'],lanes['right'];n,m=len(L),len(R);dist={(0,0):0.};parent={};heap=[(0.,0,0)];cache={};blocked={}
 def stationary(lane,i):return(np.array(lane[i]['a']if i<len(lane)else lane[-1]['b']),lane[i-1].get('gap_finish_m',lane[i-1]['gap_m'])if i else lane[0].get('gap_start_m',.08))
 while heap:
  elapsed,i,j=heapq.heappop(heap)
  if elapsed>dist[(i,j)]+1e-8:continue
  if(i,j)==(n,m):break
  for di,dj in [(1,1),(1,0),(0,1)]:
   if i+di>n or j+dj>m:continue
   a=L[i]if di else None;b=R[j]if dj else None;T=max(a['duration_s']if a else 0,b['duration_s']if b else 0);okay=True
   for t in np.linspace(0,T,max(3,int(T/dt)+2)):
    left=state_at(a,t)if a else stationary(L,i);right=state_at(b,t)if b else stationary(R,j)
    lp=a.get('payload')if a else L[i-1].get('payload')if i else None;rp=b.get('payload')if b else R[j-1].get('payload')if j else None
    key=(lp['part']if lp else None,rp['part']if rp else None,*np.round(np.r_[left[0],left[1],right[0],right[1]],8))
    if key not in cache:cache[key]=arm_pair_clear(left,right,left_payload=lp,right_payload=rp)
    if not cache[key]:
     okay=False
     if os.environ.get('M80_COORD_DIAGNOSTIC')=='1':blocked.setdefault((i,j),[]).append(dict(move=[di,dj],local_time_s=float(t),left_label=a['label']if a else'idle',right_label=b['label']if b else'idle',left_q=left[0].tolist(),right_q=right[0].tolist(),left_gap=left[1],right_gap=right[1],left_payload=lp,right_payload=rp))
     break
   if not okay:continue
   node=(i+di,j+dj);value=elapsed+T
   if value<dist.get(node,float('inf')):dist[node]=value;parent[node]=((i,j),di,dj,T);heapq.heappush(heap,(value,*node))
 if(n,m)not in parent:
  if os.environ.get('M80_COORD_DIAGNOSTIC')=='1':
   frontier=sorted(dist,key=lambda k:(n-k[0]+m-k[1],dist[k]))[:5];report=dict(lane_lengths=[n,m],reachable_states=len(dist),frontier=[dict(node=list(k),elapsed_s=dist[k],blocked=blocked.get(k,[]))for k in frontier]);(HERE/'coordination-last-failure.json').write_text(json.dumps(report,indent=2));print('COORD_FRONTIER',report['lane_lengths'],len(dist),[list(k)for k in frontier],flush=True)
  return None
 steps=[];node=(n,m)
 while node!=(0,0):
  prev,di,dj,T=parent[node];i,j=prev;steps.append(dict(left_index=i if di else None,right_index=j if dj else None,duration_s=T));node=prev
 steps.reverse();elapsed=0
 for step in steps:step['start_s']=elapsed;elapsed+=step['duration_s']
 return dict(duration_s=elapsed,steps=steps,cross_arm_clearance_reserve_m=.001,collision_sample_dt_s=dt)

def main():
 choices=json.load(open(HERE/'compact-path-candidates.json'));results=[];attempts=[]
 combinations=list(itertools.product(*(choices[p]for p in ['ring1','ring2','pin'])));combinations.sort(key=lambda c:max(c[0]['recipe']['duration_s'],c[1]['recipe']['duration_s']+c[2]['recipe']['duration_s']))
 for ci,combo in enumerate(combinations):
  recipes={x['recipe']['part']:copy.deepcopy(x['recipe'])for x in combo};geometry=[a for c in combo for a in candidate_pieces(c['candidate'])[1]];posts=[a['b']for a in geometry if a['name'].endswith('_post')];posts=np.array(posts)
  # Reuse a compact table, with every post seated inside its tabletop outline.
  center=None
  for cx in [-.25,-.35,-.45]:
   for cy in [-.25,-.3]:
    if np.max(np.abs(posts[:,0]-cx))>.24 or np.max(np.abs(posts[:,1]-cy))>.165:continue
    desk=table(cx,cy,1.1)
    if not machine_hits(desk)and endpoint_clear(desk):center=[cx,cy,1.1];break
   if center:break
  if center is None:attempts.append(dict(index=ci,reject='no_common_500x350_table'));continue
  pieces=table(*center)
  for c in combo:pieces+=candidate_pieces(c['candidate'])[0]
  if not endpoint_clear(pieces):attempts.append(dict(index=ci,reject='critical_endpoint'));continue
  set_env(pieces);failure=[]
  for part,r in recipes.items():
   repaired=[]
   for si,s in enumerate(r['segments']):
    okay,why=path_clear(r['side'],[np.array(s['a']),np.array(s['b'])],s['gap_m'],31)
    if not okay and s['label']in('empty_approach','transfer'):
     print('COMPACT_REPAIR_BEGIN',ci,part,s['label'],why,flush=True)
     replacement=route(r['side'],s['a'],s['b'],s['gap_m'],limit=220)
     if replacement:
      repaired.extend(seg(a,b,s['label'],part,s['gap_m'])for a,b in zip(replacement[:-1],replacement[1:]));continue
    if not okay:failure.append(dict(part=part,segment=si,label=s['label'],why=why));break
    repaired.append(s)
   if not any(f['part']==part for f in failure):r['segments']=repaired
  if failure:
   print('COMPACT_COMBINATION_REJECT',ci,failure,flush=True);attempts.append(dict(index=ci,reject=failure));continue
  model_reports=[];model_bad=False
  for part,r in recipes.items():
   for s in r['segments']:
    opening=.08 if part.startswith('ring')else .02
    if s['label']=='grasp':s['duration_s']=max(.18,(opening-r['gap'])/.095+.08)
    if s['label']=='release':s['duration_s']=max(.18,(opening-r['gap'])/.095+.05)
    s['duration_s'],report=model_screen(s,r['side'],r)
    if report:
     model_reports.append(dict(part=part,label=s['label'],**report));model_bad|=report['static_gravity_exceeds_95percent_budget']
   r['duration_s']=sum(s['duration_s']for s in r['segments'])
  if model_bad:attempts.append(dict(index=ci,reject='imported_model_static_torque_budget'));continue
  # Pin starts after ring2's withdrawal; this matches how it was planned.
  previous=json.load(open(HERE/'active-fixture-plan.json'));button=None
  if previous.get('button_approach')and previous.get('button_finish'):
   app=copy.deepcopy(previous['button_approach']);press=copy.deepcopy(previous['button_finish'])
   if np.linalg.norm(np.array(app[0]['a'])-recipes['ring1']['q_exit'])<1e-5 and all(path_clear('right',[np.array(s['a']),np.array(s['b'])],0.,31)[0]for s in app+press):button=(sum(s['duration_s']for s in app+press),app,press)
  if button is None:button=button_path(recipes['ring1']['q_exit'])
  if button is None:attempts.append(dict(index=ci,reject='button'));continue
  _,approach,press=button;qr=recipes['ring1']['q_exit'];close=dict(a=qr,b=qr,duration_s=.08/.095+.05,label='close_empty_hand_for_button',part='button',gap_m=.08)
  for s in approach+press:
   s['duration_s'],report=model_screen(s,'right',dict(part='button'))
   if report:model_reports.append(dict(part='button',label=s['label'],**report))
  ql=recipes['ring2']['q_exit'];preshape=dict(a=ql,b=ql,duration_s=.06/.095+.05,label='pin_preshape',part='pin',gap_m=.08)
  lanes=dict(left=recipes['ring2']['segments']+[preshape]+recipes['pin']['segments'],right=recipes['ring1']['segments']+[close]+approach)
  print('COMPACT_COORDINATE_BEGIN',ci,flush=True);schedule=coordinate(lanes)
  if not schedule:attempts.append(dict(index=ci,reject='cross_arm_schedule'));continue
  # Button is pressed after all three clamp/release operations complete.
  # Audit the terminal left pose against every button press sample.
  left=(np.array(lanes['left'][-1]['b']),lanes['left'][-1]['gap_m']);button_clear=all(arm_pair_clear(left,state_at(s,t))for s in press for t in np.linspace(0,s['duration_s'],21))
  if not button_clear:attempts.append(dict(index=ci,reject='button_cross_arm'));continue
  row=serialize(recipes,geometry,center);row.update(label='compact_'+str(ci),kinematic_path_valid=True,cycle_time_s=schedule['duration_s']+sum(s['duration_s']for s in press),coordination=schedule,lanes=lanes,button_finish=press,model_screen=model_reports,task='ring1 + ring2 + pin insertion, pneumatic/mechanical clamp waits, button press and return',limitations=['sampled rigid collision screening','no full-cell dynamic insertion','0.18s clamp wait is assumed','0.095m/s gripper total aperture speed and 80ms grasp allowance','no measured PLC latency','calibrated tool inertia not available'])
  # Bound the actual solid head structure, separately from the table footprint.
  points=[]
  for spec in geometry:
   if spec['type']=='capsule':points.extend([np.array(spec['a'])-spec['radius'],np.array(spec['b'])+spec['radius']])
   elif spec['type']=='box':
    R=np.array(spec['R']);half=np.abs(R)@ (np.array(spec['size'])/2);points.extend([np.array(spec['pos'])-half,np.array(spec['pos'])+half])
   else:points.extend([np.array(spec['pos'])-.003,np.array(spec['pos'])+.003])
  pts=np.array(points);row['fixture_aabb_min_m']=pts.min(0).tolist();row['fixture_aabb_max_m']=pts.max(0).tolist();row['fixture_envelope_mm']=np.ceil((pts.max(0)-pts.min(0))*1000).astype(int).tolist();row['size_caption']='상판 500 × 350mm · 받침 전체 약 '+' × '.join(map(str,row['fixture_envelope_mm']))+'mm'
  # Base plate only connects the tabletop-mounted posts. Its final dimensions
  # are screened as another solid before it can be described as validated.
  row['base_plate']=None
  results.append(row);(HERE/'compact-comparison.json').write_text(json.dumps(results,indent=2));(HERE/'compact-selected.json').write_text(json.dumps(min(results,key=lambda r:r['cycle_time_s']),indent=2));print('COMPACT_COMPARISON',ci,row['cycle_time_s'],row['fixture_envelope_mm'],flush=True)
  if len(results)>=3:break
 (HERE/'compact-combination-attempts.json').write_text(json.dumps(attempts,indent=2));print('COMPACT_FINISHED',len(results),flush=True)
if __name__=='__main__':main()
