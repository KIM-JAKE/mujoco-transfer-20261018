from evaluate_layouts import *
record=json.load(open(HERE/'comparison.json'));summary=[]
for layout in record['layouts']:
 if len(layout.get('recipes',{}))!=3:continue
 recipes=layout['recipes'];button=button_path(recipes['ring1']['q_exit'])
 if button is None:continue
 _,approach,press=button;lanes=dict(left=recipes['ring2']['segments']+recipes['pin']['segments'],right=recipes['ring1']['segments']+approach);candidates=[]
 for side in ['right','left']:
  for delay in [0,.5,1.,1.5,2.,2.5,3.,3.5,4.,4.5,5.,6.,7.,8.]:
   delays={s:delay if s==side else 0 for s in AD};ok,why=cross_clear(lanes,delays,.075);print('SCHEDULE',layout['label'],side,delay,ok,why,flush=True)
   if ok:
    total=max(sum(s['duration_s']for s in lanes[s])+delays[s]for s in AD)+sum(s['duration_s']for s in press);candidates.append(dict(cycle_time_s=total,delays=delays));break
 best=min(candidates,key=lambda r:r['cycle_time_s'])if candidates else None
 row=dict(label=layout['label'],recipes=recipes,lanes=lanes,button_finish=press,best=best,schedule_candidates=candidates);summary.append(row);(HERE/'scheduled-comparison.json').write_text(json.dumps(summary,indent=2))
print('DONE',flush=True)
