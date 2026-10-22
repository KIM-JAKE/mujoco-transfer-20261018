"""Native-hand paths for a compact table and three-point ring rests."""
from shallow_supply_search import *
import compact_layout_plan as cp
cp.ik=robust

def recipe_shallow(c,extra,band=None):
 r=copy.deepcopy(c['recipe']);part=r['part'];b=BANDS[part]if band is None else band;side=r['side'];q=np.array(r['q_pick']);qpre=np.array(r['q_pregrasp']);gap=r['gap'];set_env(extra);qi=np.array(b['q_insert']);pre=np.array(b['q_preinsert']);entry=route(side,READY[side],qpre,.08,limit=300)
 if entry is None:return None,'empty_entry'
 transfer=route(side,qpre,pre,gap,limit=380)
 if transfer is None:return None,'transfer'
 for a,z,op,label in [(qpre,q,.08,'pregrasp'),(q,qpre,gap,'lift'),(pre,qi,gap,'insert'),(qi,pre,.08,'withdraw')]:
  if not path_clear(side,[a,z],op,41)[0]:return None,label
 segs=[seg(a,z,'empty_approach',part,.08)for a,z in zip(entry[:-1],entry[1:])]+[seg(qpre,q,'pregrasp',part,.08),dict(a=q.tolist(),b=q.tolist(),duration_s=max(.18,(.08-gap)/.1+.08),label='grasp',part=part,gap_m=gap),seg(q,qpre,'lift',part,gap)]
 segs += [seg(a,z,'transfer',part,gap)for a,z in zip(transfer[:-1],transfer[1:])]+[seg(pre,qi,'insert',part,gap),dict(a=qi.tolist(),b=qi.tolist(),duration_s=.18,label='clamp',part=part,gap_m=gap),dict(a=qi.tolist(),b=qi.tolist(),duration_s=max(.18,(.08-gap)/.1+.05),label='release',part=part,gap_m=.08),seg(qi,pre,'withdraw',part,.08)]
 T=sum(a['duration_s']for a in segs)
 if T>25:return None,'position_velocity_timing'
 r.update(segments=segs,q_path=[a.tolist()for a in [q,qpre]+transfer[1:]+[qi]],q_exit=pre.tolist(),duration_s=T,pregrasp_delta_m=[0,0,.04]);return r,None

def main():
 candidates=json.load(open(HERE/'shallow-head-candidates.json'));candidates.extend(c for c in json.load(open(HERE/'compact-head-candidates.json'))if c['part']=='pin');candidates.sort(key=lambda c:c['score_s']);chosen={};attempts=[];tablecenter=[-.25,-.3,1.1];desk=table(*tablecenter)
 assert not machine_hits(desk)and endpoint_clear(desk)
 existing=json.load(open(HERE/'compact-path-candidates.json'))if(HERE/'compact-path-candidates.json').exists()else{}
 chosen={p:rows for p,rows in existing.items()if rows and(rows[0]['recipe'].get('holder')=='three_point_rest'or p=='pin')}
 for part in ['ring1','ring2','pin']:
  candidates=json.load(open(HERE/'shallow-head-candidates.json'));candidates.extend(c for c in json.load(open(HERE/'compact-head-candidates.json'))if c['part']=='pin');candidates.sort(key=lambda c:c['score_s'])
  wins=chosen.get(part,[])
  if len(wins)>=3:continue
  for c in [c for c in candidates if c['part']==part]:
   if any(np.linalg.norm(np.array(w['recipe']['q_pick'])-np.array(c['recipe']['q_pick']))<1e-6 for w in wins):continue
   r=c['recipe'];P=np.array(r['tcp'])-np.array(r['R'])@np.array(r['localR']).T@np.array(r['localP']);head,spec=candidate_pieces(c);post=next(a for a in spec if a['name'].endswith('_post'));foot=np.array(post['b'])
   if abs(foot[0]-tablecenter[0])>.235 or abs(foot[1]-tablecenter[1])>.16:continue
   result,why=recipe_shallow(c,desk+head)if part!='pin'else recipe_for(c,desk+head,start=BANDS['ring2']['q_preinsert'])
   print('SHALLOW_RECIPE',part,r['tcp'],r.get('stock_yaw_deg'),bool(result),why,flush=True);attempts.append(dict(part=part,tcp=r['tcp'],yaw=r.get('stock_yaw_deg'),valid=result is not None,reject=why))
   if result:
    wins.append(dict(candidate=c,recipe=result,table=dict(center=tablecenter,valid=True),geometry=spec));chosen[part]=wins;(HERE/'compact-path-candidates.json').write_text(json.dumps(chosen,indent=2))
    if len(wins)>=3:break
  if not wins:break
 (HERE/'shallow-path-attempts.json').write_text(json.dumps(attempts,indent=2));print('SHALLOW_PATHS',{k:len(v)for k,v in chosen.items()},flush=True)
if __name__=='__main__':main()
