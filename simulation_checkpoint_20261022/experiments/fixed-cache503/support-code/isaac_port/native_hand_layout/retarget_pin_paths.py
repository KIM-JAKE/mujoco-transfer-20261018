from evaluate_layouts import *
record=json.load(open(HERE/'comparison.json'));base=parts[2];p=prepare_part(base);newL=p['localR'];newP=p['localP'];gap=.0097
for layout in record['layouts']:
 if len(layout.get('recipes',{}))!=3:continue
 recipe=layout['recipes']['pin'];oldL=np.array(recipe['localR']);oldP=np.array(recipe['localP']);mapped={};fail=[]
 for oldq in recipe['q_path']+[recipe['q_exit']]:
  oldq=np.array(oldq);tcp,R=fk('left',oldq);Rp=R@oldL.T;point=tcp+Rp@(newP-oldP);orientation=Rp@newL;q=ik('left',point,orientation,oldq)
  if q is None:raise RuntimeError('Retarget IK')
  okay,why=clear('left',q,gap)
  if not okay:fail.append(why)
  mapped[tuple(oldq)]=q
 def remap(q):return mapped.get(tuple(q),np.array(q)).tolist()
 recipe['q_path']=[remap(q)for q in recipe['q_path']];recipe['q_pick']=recipe['q_path'][0];recipe['q_exit']=remap(recipe['q_exit']);recipe['localP']=newP.tolist();recipe['localR']=newL.tolist();recipe['R']=(np.array(recipe['R'])@oldL.T@newL).tolist();recipe['gap']=gap
 for seg in recipe['segments']:
  seg['a']=remap(seg['a']);seg['b']=remap(seg['b']);seg['gap_m']=gap
  if seg['label']not in ('grasp','clamp','release'):seg['duration_s']=segment_time(seg['a'],seg['b'])
  if seg['label']=='grasp':seg['duration_s']=.18
 for seg in recipe['segments']:
  okay,why=path_clear('left',[np.array(seg['a']),np.array(seg['b'])],.02 if seg['label']in('empty_approach','withdraw','release')else gap,21)
  if not okay:fail.append(dict(stage=seg['label'],why=why))
 recipe['duration_s']=sum(s['duration_s']for s in recipe['segments']);recipe['retarget_checks']=fail;recipe['grasp_contact_bench']='pin-old-band-yaw-0/pin-result.json'
 print('PIN_RETARGET',layout['label'],recipe['duration_s'],fail,flush=True)
(HERE/'comparison-physical-band.json').write_text(json.dumps(record,indent=2))
