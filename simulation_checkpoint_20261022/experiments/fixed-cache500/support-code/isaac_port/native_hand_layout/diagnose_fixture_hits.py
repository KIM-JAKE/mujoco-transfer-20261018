from finalize_fixture_paths import *
record=json.load(open(HERE/'comparison-physical-band.json'));layout=next(x for x in record['layouts']if x['label']=='near_front');frame=next(x for x in json.load(open(HERE/'routed-independent-frames.json'))if x['layout']=='near_front');pieces=fixtures(frame,layout['recipes']);diagnostics=[]
for part in ['ring2','pin']:
 r=layout['recipes'][part];q=np.array(r['q_pick'])if part=='ring2'else np.array(r['q_path'][-1]);R=np.array(r['R']);tcp=np.array(r['tcp']);gap=.08 if part=='ring2'else r['gap']
 if part=='ring2':
  pre=ik('left',tcp+[0,0,-.025],R,q);q=.6*pre+.4*q
 hits=[]
 for name,o in objects('left',q,gap)[0]:
  for label,x in pieces:
   if pairhit(o,x):hits.append([name,label])
 print('HITS',part,hits,flush=True);diagnostics.append(dict(part=part,hits=hits,q=q.tolist()))
(HERE/'fixture-hit-diagnostic.json').write_text(json.dumps(diagnostics,indent=2))
