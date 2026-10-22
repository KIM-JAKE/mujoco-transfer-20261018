"""Compare compact five-cycle cassette locations with explicit storage solids.
Produces stopped-joint paths first; whole-cell coordination is a separate stage.
"""
from shallow_layout_plan import *
from five_cycle_storage import ring_specs,pin_specs,part_pose
from fast_environment import install
from fast_environment import clear as environment_clear
import os
set_env=install()


def pieces_from_specs(spec):
 return candidate_pieces(dict(geometry=spec),top_z=1.1)[0]

def pin_clearance_route(side,a,b,gap,entry,exact_goal=True):
 """Clear the supply rack vertically before crossing it with the wide palm."""
 a=np.asarray(a);b=np.asarray(b)
 if path_clear(side,[a,b],gap,41)[0]:return[a,b]
 pa,Ra=fk(side,a);pb,Rb=fk(side,b)
 low_height=max(pb[2]+.14,pa[2]+.02)if entry else max(pa[2]+.10,pb[2]+.02)
 for height in [low_height,1.58,1.63]:
  waypoints=[(np.array([pa[0],-.03,pa[2]]),Ra),(np.array([pa[0],-.03,height]),Ra),(np.array([pa[0],-.03,height]),Rb),(np.array([pb[0],pb[1],height]),Rb),(pb,Rb)]if entry else[(np.array([pa[0],pa[1],height]),Ra),(np.array([pb[0],-.03,height]),Rb),(np.array([pb[0],-.03,pb[2]]),Rb),(pb,Rb)]
  path=[a];failed=False
  for P,R in waypoints:
   origin,R0=fk(side,path[-1]);target=robust(side,P,R,path[-1])
   if target is not None and path_clear(side,[path[-1],target],gap,41)[0]:path.append(target);continue
   start=path[-1].copy();rotation=Slerp([0,1],Rotation.from_matrix(np.array([R0,R])));chain=[]
   for u in np.linspace(0,1,9)[1:]:
    q=robust(side,origin*(1-u)+P*u,rotation([u]).as_matrix()[0],chain[-1]if chain else start)
    if q is None or not path_clear(side,[chain[-1]if chain else start,q],gap,15)[0]:failed=True;break
    chain.append(q)
   if failed:break
   path+=chain
  if not failed:
   if exact_goal and np.linalg.norm(path[-1]-b)>.001:
    if not path_clear(side,[path[-1],b],gap,41)[0]:continue
    path.append(b)
   return path
 return None

def finish_pin_recipe(tcp,R,L,lp,q,pre,raised,ext,lift,entry,transfer,qi,distance):
 gap=.0097;target=np.array(transfer[-1]);side='left'
 if not path_clear(side,[target,qi],gap,41)[0]or not path_clear(side,[qi,target],.02,41)[0]:return None,'insert_withdraw'
 ss=[seg(a,z,'empty_approach','pin',.02)for a,z in zip(entry[:-1],entry[1:])]
 ss+=[seg(pre,q,'pregrasp','pin',.02),dict(a=q.tolist(),b=q.tolist(),duration_s=max(.18,(.02-gap)/.095+.08),label='grasp',part='pin',gap_m=gap),dict(a=q.tolist(),b=q.tolist(),duration_s=.15,label='clip_open',part='pin',gap_m=gap),seg(q,raised,'clear_datum','pin',gap),seg(raised,ext,'extract','pin',gap),seg(ext,lift,'lift','pin',gap)]
 ss+=[seg(a,z,'transfer','pin',gap)for a,z in zip(transfer[:-1],transfer[1:])]
 ss+=[seg(target,qi,'insert','pin',gap),dict(a=qi.tolist(),b=qi.tolist(),duration_s=.18,label='clamp',part='pin',gap_m=gap),dict(a=qi.tolist(),b=qi.tolist(),duration_s=.18,label='release',part='pin',gap_m=.02),seg(qi,target,'withdraw','pin',.02)]
 r=dict(part='pin',side=side,tcp=tcp.tolist(),R=R.tolist(),localP=lp.tolist(),localR=L.tolist(),gap=gap,q_pick=q.tolist(),q_exit=target.tolist(),pregrasp_delta_m=(fk(side,pre)[0]-tcp).tolist(),extraction_distance_m=distance,segments=ss,q_path=[x.tolist()for x in[q,raised,ext,lift]+transfer[1:]+[qi]],duration_s=sum(s['duration_s']for s in ss))
 return r,None

def pin_recipe_structured(template,C,extra,start,R):
 b=BANDS['pin'];L=np.array(b['localR']);lp=np.array(b['localP']);Rp=R@L.T;tcp=np.array(C)+Rp@lp;side='left';gap=.0097;set_env(extra);axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis)
 if axis[1]>0:axis=-axis
 dummy=robust(side,tcp+axis*.04,R,np.array(template['q_pick']))
 if dummy is None:return None
 entry=pin_clearance_route(side,start,dummy,.02,True,exact_goal=False)
 if entry is None:return None
 pre=np.array(entry[-1]);q=robust(side,tcp,R,pre)
 if q is None or not path_clear(side,[pre,q],.02,41)[0]:return None
 axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis)
 if axis[1]>0:axis=-axis
 raised=robust(side,tcp+[0,0,.003],R,q);ext=robust(side,tcp+axis*.04+[0,0,.003],R,raised)if raised is not None else None;lift=robust(side,tcp+axis*.04+[0,0,.04],R,ext)if ext is not None else None
 if lift is None or not path_clear(side,[q,raised,ext,lift],gap,41)[0]:return None
 transfer=pin_clearance_route(side,lift,np.array(b['q_preinsert']),gap,False,exact_goal=False)
 if transfer is None:return None
 final,Rfinal=fk(side,np.array(b['q_insert']));qi=robust(side,final,Rfinal,transfer[-1])
 if qi is None:return None
 result,_=finish_pin_recipe(tcp,R,L,lp,q,pre,raised,ext,lift,entry,transfer,qi,.04)
 return result

def pin_recipe(template,C,extra,start,stock_R=None):
 b=BANDS['pin'];R=np.array(b['R'])if stock_R is None else np.array(stock_R);L=np.array(b['localR']);lp=np.array(b['localP']);Rp=R@L.T;tcp=np.array(C)+Rp@lp;side='left';gap=.0097;set_env(extra);q=None;pre=None;last=None;poses=[];seen=[];target=np.array(b['q_preinsert']);qi=np.array(b['q_insert'])
 structured=pin_recipe_structured(template,C,extra,start,R)
 if structured is not None:
  print('PIN_STRUCTURED_SUCCESS',C,structured['duration_s'],flush=True);return structured,None
 for bias in [0,-.5,.5,-1.,1.,-1.5,1.5]:
  seed=np.array(template['q_pick']);seed[0]+=bias;trial=robust(side,tcp,R,np.clip(seed,LOWER+.01,UPPER-.01))
  if trial is None:continue
  if any(np.linalg.norm(trial-v)<.02 for v in seen):continue
  seen.append(trial)
  okay,last=environment_clear(side,trial,.02)
  if not okay:continue
  axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis)
  if axis[1]>0:axis=-axis
  candidate=None
  for delta in [axis*.04,np.array([0,0,-.025])]:
   attempt=robust(side,tcp+delta,R,trial)
   if attempt is None:continue
   okay,last=path_clear(side,[attempt,trial],.02,41)
   if okay:candidate=attempt;break
  if candidate is None:continue
  qr=robust(side,tcp+[0,0,.003],R,trial)
  for distance in [.04,.06,.10]:
   qe=robust(side,tcp+axis*distance+[0,0,.003],R,qr)if qr is not None else None
   ql=robust(side,tcp+axis*distance+[0,0,.04],R,qe)if qe is not None else None
   if ql is None:continue
   okay,last=path_clear(side,[trial,qr,qe,ql],gap,41)
   if okay:
    direct=int(not path_clear(side,[np.array(start),candidate],.02,41)[0])+int(not path_clear(side,[ql,target],gap,41)[0]);cost=segment_time(start,candidate)+segment_time(ql,target)
    poses.append((direct,cost,trial,candidate,qr,qe,ql,distance));break
 if not poses:return None,dict(stage='collision_aware_pick',last=last)
 if os.environ.get('M80_ROUTE_DEBUG'):print('PIN_ROUTE_POSES',np.array(C).tolist(),[(round(x[2][0],3),x[0],round(x[1],3),x[-1])for x in poses],flush=True)
 for _,_,qc,pc,rc,ec,lc,dc in sorted(poses,key=lambda row:row[:2]):
  cached_entry=route(side,start,pc,.02,limit=200,allow_search=False)
  if cached_entry is None:continue
  cached_transfer=route(side,lc,target,gap,limit=220,allow_search=False)
  if cached_transfer is None:continue
  result,why=finish_pin_recipe(tcp,R,L,lp,qc,pc,rc,ec,lc,cached_entry,cached_transfer,qi,dc)
  if result is not None:
   if os.environ.get('M80_ROUTE_DEBUG'):print('PIN_CACHED_CORRIDOR',result['duration_s'],flush=True)
   return result,None
 for _,_,q,pre,raised,ext,lift,extraction_distance in sorted(poses,key=lambda row:row[:2])[:4]:
  if os.environ.get('M80_ROUTE_DEBUG'):print('PIN_ROUTE_SEARCH',float(q[0]),flush=True)
  entry=pin_clearance_route(side,start,pre,.02,True)
  if entry is None:entry=route(side,start,pre,.02,limit=200)
  if entry is None:continue
  transfer=pin_clearance_route(side,lift,target,gap,False)
  if transfer is None:transfer=route(side,lift,target,gap,limit=220)
  if transfer is not None:break
 else:return None,dict(stage='entry_transfer_routes',branches=len(poses))
 axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis)
 if axis[1]>0:axis=-axis
 if pre is None or lift is None:return None,'pickup_ik'
 for path,opening,label in [([pre,q],.02,'pregrasp'),([q,raised,ext,lift],gap,'extract_lift')]:
  if not path_clear(side,path,opening,41)[0]:return None,label
 r=dict(part='pin',side='left',tcp=tcp.tolist(),R=R.tolist(),localP=lp.tolist(),localR=L.tolist(),gap=gap,q_pick=q.tolist(),q_exit=target.tolist(),pregrasp_delta_m=[0,0,-.025],extraction_distance_m=extraction_distance);ss=[seg(a,z,'empty_approach','pin',.02)for a,z in zip(entry[:-1],entry[1:])]+[seg(pre,q,'pregrasp','pin',.02),dict(a=q.tolist(),b=q.tolist(),duration_s=max(.18,(.02-gap)/.095+.08),label='grasp',part='pin',gap_m=gap),seg(q,ext,'extract','pin',gap),seg(ext,lift,'lift','pin',gap)]+[seg(a,z,'transfer','pin',gap)for a,z in zip(transfer[:-1],transfer[1:])]+[seg(target,qi,'insert','pin',gap),dict(a=qi.tolist(),b=qi.tolist(),duration_s=.18,label='clamp',part='pin',gap_m=gap),dict(a=qi.tolist(),b=qi.tolist(),duration_s=.18,label='release',part='pin',gap_m=.02),seg(qi,target,'withdraw','pin',.02)]
 if not path_clear(side,[target,qi],gap,41)[0]or not path_clear(side,[qi,target],.02,41)[0]:return None,'insert_withdraw'
 for i,s in enumerate(ss):
  if s['label']=='extract':
   ss[i:i+1]=[dict(a=q.tolist(),b=q.tolist(),duration_s=.15,label='clip_open',part='pin',gap_m=gap),seg(q,raised,'clear_datum','pin',gap),seg(raised,ext,'extract','pin',gap)];break
 r.update(segments=ss,q_path=[x.tolist()for x in[q,raised,ext,lift]+transfer[1:]+[qi]],duration_s=sum(s['duration_s']for s in ss),pregrasp_delta_m=(fk(side,pre)[0]-tcp).tolist());return r,None

def ring_recipe(c,pieces):
 """Choose arm redundancy with the complete supply fixture in place."""
 base=c['recipe'];part=base['part'];side=base['side'];R=np.array(base['R']);tcp=np.array(base['tcp']);gap=base['gap'];set_env(pieces);choices=[];seen=[]
 cached=[]
 for path in sorted(HERE.glob('five-layout-*.json')):
  try:previous=json.load(open(path))
  except (ValueError,OSError):continue
  if not isinstance(previous,dict):continue
  r=previous.get('recipes',{}).get(part)
  if not r or np.linalg.norm(np.array(r['tcp'])-tcp)>1e-7 or np.linalg.norm(np.array(r['R'])-R)>1e-7:continue
  if all(path_clear(side,[np.array(s['a']),np.array(s['b'])],s['gap_m'],41)[0]for s in r['segments']):cached.append(r)
 if cached:return copy.deepcopy(min(cached,key=lambda r:r['duration_s'])),None
 for bias in [0,.5,-.5,1.,-1.,1.5,-1.5,2.,-2.]:
  seed=np.array(base['q_pick']);seed[0]+=bias;q=robust(side,tcp,R,np.clip(seed,LOWER+.01,UPPER-.01))
  if q is None or any(np.linalg.norm(q-v)<.02 for v in seen):continue
  seen.append(q);pre=robust(side,tcp+[0,0,.04],R,q)
  if pre is None:continue
  if not path_clear(side,[pre,q],.08,41)[0]or not path_clear(side,[q,pre],gap,41)[0]:continue
  trial=copy.deepcopy(c);trial['recipe'].update(q_pick=q.tolist(),q_pregrasp=pre.tolist())
  direct=path_clear(side,[pre,np.array(BANDS[part]['q_preinsert'])],gap,41)[0]
  choices.append((not direct,segment_time(READY[side],pre)+segment_time(pre,BANDS[part]['q_preinsert']),trial,copy.deepcopy(BANDS[part])))
 # Keep the identical target TCP and part pose while changing the redundant
 # elbow posture. A fixed joint branch unnecessarily constrained transfers.
 b=BANDS[part];goal,Rgoal=fk(side,np.array(b['q_preinsert']));final,Rfinal=fk(side,np.array(b['q_insert']));targets=[]
 for bias in [0,.5,-.5,1.,-1.,1.5,-1.5]:
  seed=np.array(b['q_preinsert']);seed[0]+=bias;qt=robust(side,goal,Rgoal,np.clip(seed,LOWER+.01,UPPER-.01))
  if qt is None or any(np.linalg.norm(qt-v[0])<.02 for v in targets):continue
  qi=robust(side,final,Rfinal,qt)
  if qi is None or not path_clear(side,[qt,qi],gap,41)[0]or not path_clear(side,[qi,qt],.08,41)[0]:continue
  band=copy.deepcopy(b);band.update(q_preinsert=qt.tolist(),q_insert=qi.tolist());targets.append((qt,band))
 alternatives=[]
 for _,_,trial,_ in choices:
  pre=np.array(trial['recipe']['q_pregrasp'])
  for qt,band in targets:
   direct=path_clear(side,[pre,qt],gap,41)[0]
   alternatives.append((not direct,segment_time(READY[side],pre)+segment_time(pre,qt),trial,band))
 choices+=alternatives
 choices.sort(key=lambda row:row[:2]);failures=[]
 for _,_,trial,band in choices[:4]:
  r,why=recipe_shallow(trial,pieces,band)
  if r is not None:return r,None
  failures.append(why)
 return None,dict(stage='ring_redundancy_routes',branches=len(choices),failures=failures)


def main():
 source=json.load(open(HERE/'compact-path-candidates.json'));raw=json.load(open(HERE/'shallow-head-candidates.json'));cases=[]
 # Fast/low/front arrangements retain the same 500x350 table outline.
 cases.append(('high_frontbank',source['ring1'][0],source['ring2'][1],[-.25,-.37,1.335]))
 cases.append(('low_frontbank',source['ring1'][1],source['ring2'][0],[-.25,-.37,1.315]))
 def raw_candidate(part,tcp,yaw):
  c=next(c for c in raw if c['part']==part and np.linalg.norm(np.array(c['recipe']['tcp'])-tcp)<1e-7 and c['recipe']['stock_yaw_deg']==yaw);return dict(candidate=c,recipe=c['recipe'])
 def shifted(c,delta):
  out=copy.deepcopy(c);r=out['candidate']['recipe'];r['tcp']=(np.array(r['tcp'])+delta).tolist();q=robust(r['side'],np.array(r['tcp']),np.array(r['R']),np.array(r['q_pick']))
  if q is not None:
   pre=robust(r['side'],np.array(r['tcp'])+[0,0,.04],np.array(r['R']),q)
   if pre is not None:r.update(q_pick=q.tolist(),q_pregrasp=pre.tolist())
  out['recipe']=r;return out
 cases.append(('front_frontbank',raw_candidate('ring1',[-.05,-.32,1.38],45),raw_candidate('ring2',[-.4,-.32,1.38],-45),[-.25,-.37,1.335]))
 cases.sort(key=lambda case:case[0]!='front_frontbank')
 cases=[('right_high',shifted(source['ring1'][0],[.05,0,0]),source['ring2'][1],[-.27,-.27,1.335]),('right_low',shifted(source['ring1'][1],[.05,0,.006]),source['ring2'][0],[-.27,-.27,1.335]),('right_mixed',shifted(source['ring1'][1],[.05,0,.006]),source['ring2'][1],[-.27,-.27,1.335]),('right_front',shifted(raw_candidate('ring1',[-.05,-.32,1.38],45),[.05,0,0]),raw_candidate('ring2',[-.4,-.32,1.38],-45),[-.27,-.16,1.335])]+cases
 if os.environ.get('M80_LAYOUT_CASES'):cases=[c for c in cases if c[0]in os.environ['M80_LAYOUT_CASES'].split(',')]
 results=[];comparison_path=HERE/os.environ.get('M80_COMPARISON_FILE','five-layout-path-comparison.json')
 for label,c1,c2,pincenter in cases:
  pincenter=[float(os.environ.get('M80_PIN_CENTER_X',pincenter[0])),float(os.environ.get('M80_PIN_CENTER_Y',pincenter[1])),pincenter[2]]
  if os.environ.get('M80_PIN_HEIGHT'):pincenter=[*pincenter[:2],float(os.environ['M80_PIN_HEIGHT'])];label+='_h'+str(round(pincenter[2]*1000))
  if os.environ.get('M80_PIN_PITCH'):label+='_p'+str(round(float(os.environ['M80_PIN_PITCH'])*1000))
  table_center=[-.20 if label.startswith('right_')else-.25,-.3,1.1];desk=table(*table_center)
  if machine_hits(desk):print('FIVE_TABLE_REJECT',label,flush=True);continue
  geometry=[];metadata={};stocks=[]
  for part,c in [('ring1',c1),('ring2',c2)]:
   P,Rp=part_pose(c['recipe']);spec,envelope,meta=ring_specs(part,P,np.array(c['recipe']['R']));envelope['collision_only']=True;geometry+=spec+[envelope];metadata[part]=meta
   geometry.append(dict(name=part+'_elevator_plate',type='cylinder',pos=meta['plate_initial_center'],R=np.eye(3).tolist(),radius=meta['plate_radius_m'],length=meta['plate_thickness_m'],render_only=True))
   for i in range(5):stocks.append(dict(part=part,index=i,P=(P+[0,0,-i*meta['pitch_m']]).tolist(),R=Rp.tolist()))
  yaw=float(os.environ.get('M80_PIN_YAW','0'));A=Rotation.from_euler('z',yaw,degrees=True).as_matrix();baseR=np.array(BANDS['pin']['R']);pinR=A@baseR;pinL=np.array(BANDS['pin']['localR']);pinRp=pinR@pinL.T;ps,pcenters,meta=pin_specs(pincenter,baseR)
  if yaw:
   center=np.array(pincenter)
   for spec in ps:
    for key in ['pos','a','b']:
     if key in spec:spec[key]=(center+A@(np.array(spec[key])-center)).tolist()
    if 'R'in spec:spec['R']=(A@np.array(spec['R'])).tolist()
   pcenters=[(center+A@(np.array(C)-center)).tolist()for C in pcenters];meta['extraction_axis']=(A@np.array(meta['extraction_axis'])).tolist()
   label+='_yaw'+str(int(yaw))
  meta['stock_yaw_deg']=yaw;geometry+=ps;metadata['pin']=meta
  for i,C in enumerate(pcenters):stocks.append(dict(part='pin',index=i,P=C,R=pinRp.tolist()))
  # The elevator plate is covered by the cassette envelope throughout its stroke.
  pieces=desk+pieces_from_specs([a for a in geometry if not a.get('render_only')]);row=dict(label='five_'+label,storage_capacity=dict(ring1=5,ring2=5,pin=5),table_center_m=table_center,table_size_m=[.5,.35],geometry=geometry,stock_parts=stocks,label_centers=dict(pin=pincenter),common_start_q={s:READY[s].tolist()for s in AD},storage_metadata=metadata,recipes={},pin_slots=[],failures=[],hardware_ready=False,full_physics_assembly_tested=False)
  static_hits=machine_hits(pieces)
  if static_hits:
   row['failures'].append(dict(stage='fixture_machine_intersection',hits=static_hits));results.append(row);(HERE/('five-layout-'+label+'.json')).write_text(json.dumps(row,indent=2));print('FIVE_REJECT',label,row['failures'],flush=True);continue
  cross_fixture=[]
  for i,(name,a)in enumerate(pieces):
   if name.startswith('independent_table'):continue
   group=name.split('_')[0]
   for other,b in pieces[i+1:]:
    if other.startswith('independent_table')or other.split('_')[0]==group:continue
    if pairhit(a,b):cross_fixture.append([name,other])
  if cross_fixture:
   row['failures'].append(dict(stage='cross_fixture_intersection',hits=cross_fixture));results.append(row);(HERE/('five-layout-'+label+'.json')).write_text(json.dumps(row,indent=2));print('FIVE_REJECT',label,row['failures'],flush=True);continue
  set_env(pieces)
  ready_failures=[]
  for side in AD:
   okay,why=environment_clear(side,READY[side],.08)
   if not okay:ready_failures.append(dict(side=side,why=why))
  # Actual insertion branches are screened by each recipe. A blocked legacy
  # elbow pose must not reject another IK branch at the same CAD target.
  if ready_failures:
   row['failures'].append(dict(stage='ready',hits=ready_failures));results.append(row);(HERE/('five-layout-'+label+'.json')).write_text(json.dumps(row,indent=2));print('FIVE_REJECT',label,row['failures'],flush=True);continue
  for part,c in [('ring1',c1),('ring2',c2)]:
   print('FIVE_RING_PLAN_BEGIN',label,part,flush=True);r,why=ring_recipe(c['candidate'],pieces)
   if r is None:row['failures'].append(dict(part=part,why=why));break
   row['recipes'][part]=r;print('FIVE_RING_PLAN_DONE',label,part,r['duration_s'],flush=True)
  if len(row['recipes'])==2:
   (HERE/('five-layout-'+label+'.json')).write_text(json.dumps(row,indent=2))
   for slot,C in enumerate(pcenters):
    template=min(row['pin_slots'],key=lambda s:np.linalg.norm(np.array(s['recipe']['tcp'])-C))['recipe']if row['pin_slots']else source['pin'][0]['recipe']
    print('FIVE_PIN_PLAN_BEGIN',label,slot+1,flush=True);r,why=pin_recipe(template,C,pieces,np.array(row['recipes']['ring2']['q_exit']),pinR)
    if r is None:row['failures'].append(dict(part='pin',slot=slot+1,why=why));print('FIVE_PIN_REJECT',label,slot+1,why,flush=True);continue
    row['pin_slots'].append(dict(slot=slot+1,recipe=r));row['recipes']['pin']=r;print('FIVE_PIN_PLAN_DONE',label,slot+1,r['duration_s'],flush=True)
    (HERE/('five-layout-'+label+'.json')).write_text(json.dumps(row,indent=2))
  row['individual_paths_complete']=len(row['recipes'])==3 and len(row['pin_slots'])==5 and not row['failures'];results.append(row);(HERE/('five-layout-'+label+'.json')).write_text(json.dumps(row,indent=2));comparison_path.write_text(json.dumps(results,indent=2));print('FIVE_LAYOUT_DONE',label,row['individual_paths_complete'],row['failures'],flush=True)
 comparison_path.write_text(json.dumps(results,indent=2))
if __name__=='__main__':main()
