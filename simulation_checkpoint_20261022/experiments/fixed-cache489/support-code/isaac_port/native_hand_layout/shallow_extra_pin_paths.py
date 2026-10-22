"""Plan centrally placed pin heads together with the two compact ring holders."""
from shallow_layout_plan import *
chosen=json.load(open(HERE/'compact-path-candidates.json'));r1=chosen['ring1'][0];r2=chosen['ring2'][1];pieces=table(-.25,-.3,1.1)+candidate_pieces(r1['candidate'])[0]+candidate_pieces(r2['candidate'])[0];out=[]
cs=[c for c in json.load(open(HERE/'compact-head-candidates.json'))if c['part']=='pin'and c['recipe']['tcp'][1]<-.3 and c['recipe']['tcp'][0]>-.3];cs.sort(key=lambda c:c['score_s'])
for c in cs:
 head,spec=candidate_pieces(c);result,why=recipe_for(c,pieces+head,start=BANDS['ring2']['q_preinsert']);print('EXTRA_PIN_PATH',c['recipe']['tcp'],bool(result),why,flush=True)
 if result:
  row=dict(candidate=c,recipe=result,table=dict(center=[-.25,-.3,1.1],valid=True),geometry=spec);chosen['pin'].append(row);out.append(row);(HERE/'compact-path-candidates.json').write_text(json.dumps(chosen,indent=2))
  if len(out)>=3:break
print('EXTRA_PIN_PATHS',len(out),flush=True)
