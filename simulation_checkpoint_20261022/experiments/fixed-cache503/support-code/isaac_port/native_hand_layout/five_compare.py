"""Compare feasible five-capacity layouts using stopped, screened arm motions.
No full dynamic insertion or hardware certification. Does not update fieldkit.
"""
from compact_compare import *
from five_layout_search import pieces_from_specs
import sys

def screened_button(start):
 # Reuse the original native-hand button solution only after checking every
 # segment against the current, fully occupied fixture scene.
 for filename in ['five-button-cache.json','active-fixture-plan.json']:
  path=HERE/filename
  if not path.exists():continue
  previous=json.load(open(path));approach=copy.deepcopy(previous.get('button_approach',[]));press=copy.deepcopy(previous.get('button_finish',[]))
  if not approach or not press or np.max(abs(np.asarray(approach[0]['a'])-start))>1e-7:continue
  if not all(path_clear('right',[np.asarray(s['a']),np.asarray(s['b'])],0.,41)[0]for s in approach+press):continue
  chain=[np.asarray(approach[0]['a'])]+[np.asarray(s['b'])for s in approach];short=[];k=0
  while k<len(chain)-1:
   chosen=k+1
   for end in range(len(chain)-1,k+1,-1):
    if path_clear('right',[chain[k],chain[end]],0.,61)[0]:chosen=end;break
   short.append(seg(chain[k],chain[chosen],'button_approach','button',0.));k=chosen
  record=dict(button_approach=short,button_finish=press);cache=HERE/'five-button-cache.json';temporary=cache.with_suffix('.'+str(os.getpid())+'.tmp');temporary.write_text(json.dumps(record,indent=2));temporary.replace(cache)
  print('FIVE_BUTTON_REUSED',filename,len(approach),len(short),flush=True)
  return sum(s['duration_s']for s in short+press),short,press
 print('FIVE_BUTTON_SEARCH',flush=True);result=button_path(start)
 if result:
  _,approach,press=result;cache=HERE/'five-button-cache.json';temporary=cache.with_suffix('.'+str(os.getpid())+'.tmp');temporary.write_text(json.dumps(dict(button_approach=approach,button_finish=press),indent=2));temporary.replace(cache)
 return result

def retime(recipe):
 reports=[]
 for s in recipe['segments']:
  if s['label']in('grasp','clip_open','clear_datum','extract','lift','transfer','insert','clamp','release'):s['payload']={k:copy.deepcopy(recipe[k])for k in ['part','localP','localR']}
  if s['label']in('grasp','release'):
   opening=.02 if recipe['part']=='pin'else .08
   s['duration_s']=max(.18,(opening-recipe['gap'])/.095+(.08 if s['label']=='grasp'else .05))
  s['duration_s'],report=model_screen(s,recipe['side'],recipe)
  if report:reports.append(dict(part=recipe['part'],label=s['label'],**report))
 recipe['duration_s']=sum(s['duration_s']for s in recipe['segments'])
 return reports

def prepare_lane(lane,recipes,split=False):
 if not split:return copy.deepcopy(lane)
 out=[]
 for s in lane:
  if s['label']in('transfer','empty_approach','button_approach')and s['duration_s']>.45:
   a=np.array(s['a']);b=np.array(s['b']);mid=(a+b)/2
   for x,y in [(a,mid),(mid,b)]:
    sub=copy.deepcopy(s);sub.update(a=x.tolist(),b=y.tolist(),duration_s=segment_time(x,y));sub['duration_s'],_=model_screen(sub,recipes[sub['part']]['side']if sub['part']!='button'else'right',recipes.get(sub['part'],dict(part='button')));out.append(sub)
  else:out.append(copy.deepcopy(s))
 return out

def compare(path):
 row=json.load(open(path))
 if not row.get('individual_paths_complete'):return dict(label=row['label'],eligible=False,reason='individual_paths_incomplete',failures=row['failures'])
 if not row.get('nominal_payload_paths_pass'):return dict(label=row['label'],eligible=False,reason='payload_geometry_unverified')
 from audit_five_geometry import obstacles
 pieces=obstacles(row);set_env(pieces)
 reports=[];base={p:copy.deepcopy(row['recipes'][p])for p in ['ring1','ring2']}
 for r in base.values():reports+=retime(r)
 qright=base['ring1']['q_exit'];button=screened_button(qright)
 if button is None:return dict(label=row['label'],eligible=False,reason='button_path')
 _,approach,press=button
 for s in approach+press:
  s['duration_s'],report=model_screen(s,'right',dict(part='button'))
  if report:reports.append(dict(part='button',label=s['label'],**report))
 cycles=[]
 for slot in row['pin_slots']:
  recipes=copy.deepcopy(base);recipes['pin']=copy.deepcopy(slot['recipe']);localreports=retime(recipes['pin']);reports+=localreports;qleft=recipes['ring2']['q_exit']
  left=recipes['ring2']['segments']+[dict(a=qleft,b=qleft,duration_s=.06/.095+.05,label='pin_preshape',part='pin',gap_m=.02)]+recipes['pin']['segments']
  right=recipes['ring1']['segments']+[dict(a=qright,b=qright,duration_s=.08/.095+.05,label='close_empty_hand_for_button',part='button',gap_m=0.)]+copy.deepcopy(approach)
  trial=None
  for parked in [False,True]:
   if parked:
    extra=route('left',left[-1]['b'],READY['left'],.02,limit=450)
    if extra is None:continue
    for a,b in zip(extra[:-1],extra[1:]):
     s=seg(a,b,'park','pin',.02);s['duration_s'],_=model_screen(s,'left',dict(part='pin'));left.append(s)
   for split in [False,True]:
    lanes=dict(left=prepare_lane(left,recipes,split),right=prepare_lane(right,recipes,split));print('FIVE_COORD_BEGIN',row['label'],slot['slot'],parked,split,flush=True);schedule=coordinate(lanes,.04)
    if schedule is None:continue
    last=(np.array(lanes['left'][-1]['b']),lanes['left'][-1].get('gap_finish_m',.02))
    if not all(arm_pair_clear(last,state_at(s,t))for s in press for t in np.linspace(0,s['duration_s'],31)):continue
    candidate=dict(slot=slot['slot'],duration_s=schedule['duration_s']+sum(s['duration_s']for s in press),coordination=schedule,lanes=lanes,button_finish=press,recipes=recipes,left_parks=parked,split_free_motions=split)
    if trial is None or candidate['duration_s']<trial['duration_s']:trial=candidate
   if trial:break
  if trial is None:return dict(label=row['label'],eligible=False,reason='cross_arm_coordination',failed_slot=slot['slot'])
  return_lanes={}
  for side,start,width in [('left',trial['lanes']['left'][-1]['b'],.02),('right',press[-1]['b'],0.)]:
   rp=route(side,start,READY[side],width,limit=450)
   if rp is None:return dict(label=row['label'],eligible=False,reason='return_to_ready',side=side)
   return_lanes[side]=[]
   for a,b in zip(rp[:-1],rp[1:]):
    s=seg(a,b,'park','button',width);s['duration_s'],_=model_screen(s,side,dict(part='button'));return_lanes[side].append(s)
  reset=coordinate(return_lanes,.04,dict(left=.02,right=0.))
  if reset is None:return dict(label=row['label'],eligible=False,reason='return_cross_arm')
  trial.update(return_lanes=return_lanes,return_coordination=reset,gripper_reset_s=.08/.095+.05,complete_reset_cycle_time_s=trial['duration_s']+reset['duration_s']+.08/.095+.05)
  cycles.append(trial);(HERE/(row['label']+'-schedule-progress.json')).write_text(json.dumps(dict(label=row['label'],complete=False,cycles=cycles),indent=2));print('FIVE_COORD_DONE',row['label'],slot['slot'],trial['duration_s'],trial['complete_reset_cycle_time_s'],flush=True)
 result=copy.deepcopy(row);result.update(eligible=True,kinematic_task_comparison_only=True,cycles=cycles,mean_task_time_s=float(np.mean([s['duration_s']for s in cycles])),worst_task_time_s=max(s['duration_s']for s in cycles),mean_reset_cycle_time_s=float(np.mean([s['complete_reset_cycle_time_s']for s in cycles])),five_cycles_total_time_s=sum(s['complete_reset_cycle_time_s']for s in cycles),model_reports=reports,limitations=['Task time ends at button retraction; reset-cycle time includes return to READY and gripper reopening','0.18 second pneumatic clamp latency assumed','Storage feeding hardware is conceptual; ring index expected to overlap other work','Sampled rigid arm paths and imported approximate inertia, not a calibrated FR3 model','Whole-cell PhysX and precision insertion unverified'])
 if any(r['static_gravity_exceeds_95percent_budget']for r in reports+localreports):result.update(eligible=False,reason='model_static_torque_budget')
 (HERE/(row['label']+'-scheduled.json')).write_text(json.dumps(result,indent=2));return result

if __name__=='__main__':
 results=[]
 for arg in sys.argv[1:]:
  result=compare(Path(arg));results.append(result);(HERE/'five-task-comparison.json').write_text(json.dumps(results,indent=2));print('FIVE_TASK_RESULT',result['label'],result['eligible'],result.get('mean_task_time_s'),result.get('reason'),flush=True)
