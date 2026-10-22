"""Native Hand paths with independent, physically screened retracting fork heads."""
from finalize_fixture_paths import *
record=json.load(open(HERE/'comparison-physical-band.json'));layout=next(x for x in record['layouts']if x['label']=='near_front');frame=next(x for x in json.load(open(HERE/'routed-independent-frames.json'))if x['layout']=='near_front');pieces=fixtures(frame,layout['recipes']);heads=json.load(open(HERE/'retracting-heads.json'));moves={x['part']:x['best']for x in heads};original=list(statics);initial=[(name,o,o.getTranslation().copy(),o.getRotation().copy())for name,o in pieces];out=dict(label='active_near_front',recipes=layout['recipes'],support_frames=frame['frames'],head_motions=moves,failures=[],common_start_q={s:READY[s].tolist()for s in AD})

cached=json.load(open(HERE/'active-fixture-plan.json'))if (HERE/'active-fixture-plan.json').exists()else None
if cached and not cached.get('failures'):
 out['recipes']['ring1']=cached['recipes']['ring1'];out['completed_parts']=['ring1']
else:out['completed_parts']=[]

def environment(r1=False,r2=False):
 for name,o,p,R in initial:
  shift=np.zeros(3)
  if (name.startswith('ring1_fork')or name.startswith('ring1_spine'))and r1:shift=np.array(moves['ring1']['stowed_delta_m'])
  if (name.startswith('ring2_fork')or name.startswith('ring2_spine'))and r2:shift=np.array(moves['ring2']['stowed_delta_m'])
  o.setTransform(fcl.Transform(R,p+shift))
 g.manager=fcl.DynamicAABBTreeCollisionManager();g.manager.registerObjects(original+[o for _,o,_,_ in initial]);g.manager.setup()

def motion(a,b,label,part,gap):return dict(a=np.array(a).tolist(),b=np.array(b).tolist(),duration_s=segment_time(a,b),label=label,part=part,gap_m=gap)

def safe_route(side,a,b,gap,label,part):
 okay,why=path_clear(side,[a,b],gap,25)
 route=[a,b]if okay else rrt(side,a,b,gap,max_nodes=350)
 if route is None:
  out['failures'].append(dict(part=part,label=label,why=why));return []
 # A coarse RRT edge is never accepted without a finer second check.
 okay,why=path_clear(side,route,gap,31)
 if not okay:out['failures'].append(dict(part=part,label=label,why=why));return []
 return [motion(a,b,label,part,gap)for a,b in zip(route[:-1],route[1:])]

for part in ['ring1','ring2','pin']:
 if part in out['completed_parts']:continue
 r=out['recipes'][part];side=r['side'];old=r['segments'];qpick=np.array(r['q_pick']);R=np.array(r['R']);tcp=np.array(r['tcp']);gap=r['gap'];open_gap=.08 if part.startswith('ring')else .02;segs=[];start=READY[side]if part!='pin'else np.array(out['recipes']['ring2']['q_exit'])
 # Ring1 is retired before ring2's last approach; both are retired for pin work.
 environment(part in('ring2','pin'),part=='pin')
 pre=None;pre_shift=None
 for delta in [np.array([0,0,-.025]),-R[:,2]*.025,-R[:,0]*.025,R[:,0]*.025,-R[:,2]*.06,-R[:,2]*.10,np.array([0,-.025,-.015])]:
  trial=ik(side,tcp+delta,R,qpick)
  if trial is not None and clear(side,trial,open_gap)[0] and path_clear(side,[trial,qpick],open_gap,25)[0]:pre=trial;pre_shift=delta;break
 if pre is None:out['failures'].append([part,'no_clear_pregrasp']);continue
 r['pregrasp_delta_m']=pre_shift.tolist()
 print('APPROACH_BEGIN',part,'pre',pre_shift.tolist(),flush=True)
 segs+=safe_route(side,start,pre,open_gap,'empty_approach',part)
 print('APPROACH_END',part,out['failures'],flush=True)
 segs+=safe_route(side,pre,qpick,open_gap,'pregrasp',part)
 close=max(.18,(open_gap-gap)/.1+.08);segs.append(dict(a=qpick.tolist(),b=qpick.tolist(),duration_s=close,label='grasp',part=part,gap_m=gap))
 if part.startswith('ring'):
  segs.append(dict(a=qpick.tolist(),b=qpick.tolist(),duration_s=moves[part]['duration_s'],label='head_retract',part=part,gap_m=gap));environment(True,part=='ring2')
 else:segs.append(dict(a=qpick.tolist(),b=qpick.tolist(),duration_s=.15,label='stock_clip_open',part=part,gap_m=gap));environment(True,True)
 # Preserve collision-screened extraction/lift geometry rather than shortening
 # a transfer without checking the other native Hand and the empty holders.
 oldpath=[np.array(q)for q in r['q_path']]
 for i,(a,b)in enumerate(zip(oldpath[:-1],oldpath[1:])):
  label='extract'if i==0 else'lift'if i==1 else'insert'if i==len(oldpath)-2 else'transfer'
  segs+=safe_route(side,a,b,gap,label,part)
 qend=oldpath[-1];segs.append(dict(a=qend.tolist(),b=qend.tolist(),duration_s=.18,label='clamp',part=part,gap_m=gap));segs.append(dict(a=qend.tolist(),b=qend.tolist(),duration_s=max(.18,(open_gap-gap)/.1+.05),label='release',part=part,gap_m=open_gap));segs+=safe_route(side,qend,np.array(r['q_exit']),open_gap,'withdraw',part)
 r['segments']=segs;r['duration_s']=sum(s['duration_s']for s in segs);out['completed_parts'].append(part);print('ACTIVE_PATH',part,r['duration_s'],out['failures'],flush=True);(HERE/'active-fixture-plan.json').write_text(json.dumps(out,indent=2))
environment(True,True);button=button_path(out['recipes']['ring1']['q_exit'])
if button:
 out['button_approach']=button[1];out['button_finish']=button[2]
else:out['failures'].append('button_path')
out['valid']=not out['failures'];(HERE/'active-fixture-plan.json').write_text(json.dumps(out,indent=2));print('ACTIVE_COMPLETE',out['valid'],flush=True)
