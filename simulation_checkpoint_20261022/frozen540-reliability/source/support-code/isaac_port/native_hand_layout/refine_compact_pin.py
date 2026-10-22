"""Use a physically tested lower pin band and a connected axial locator/clip."""
from shallow_supply_search import *
rows=json.load(open(HERE/'lower-pin-contact-summary.json'));best=min((r for r in rows if r['passed']),key=lambda r:r['final_position_error_m']);old=BANDS['pin'];L=np.array(best['localR']);lp=np.array(best['localP']);R=np.array(old['R']);Rp=R@np.array(old['localR']).T;Pgoal=np.array(old['tcp'])-Rp@np.array(old['localP']);tcp=Pgoal+Rp@lp;qi=robust('left',tcp,R,np.array(old['q_insert']));pre=robust('left',tcp+[0,0,-.055],R,np.array(old['q_preinsert']));assert qi is not None and pre is not None
set_env([]);gap=.0097;okay,why=path_clear('left',[pre,qi],gap,51);print('LOWER_PIN_INSERT',okay,why,flush=True);assert okay
clearance=g.clearance(R,tcp,gap)[0];band=dict(part='pin',R=R.tolist(),tcp=tcp.tolist(),q_insert=qi.tolist(),q_preinsert=pre.tolist(),localP=lp.tolist(),localR=L.tolist(),gap=gap,hand_machine_clearance_m=float(clearance),contact_final_position_error_m=best['final_position_error_m']);(HERE/'compact-pin-band.json').write_text(json.dumps(band,indent=2))
source=json.load(open(HERE/'compact-head-candidates.json'));replacement=[];Pref=np.array(best['tcp'])-np.array(best['R'])@L.T@lp
for c in [x for x in source if x['part']=='pin']:
 r=copy.deepcopy(c['recipe']);P=np.array(r['tcp'])-np.array(r['R'])@np.array(r['localR']).T@np.array(r['localP']);newtcp=P+Rp@lp;q=robust('left',newtcp,R,np.array(r['q_pick']))
 if q is None:continue
 r.update(tcp=newtcp.tolist(),R=R.tolist(),localP=lp.tolist(),localR=L.tolist(),gap=gap,q_pick=q.tolist());spec=copy.deepcopy(best['custom_holder_geometry']);shift=P-Pref
 for a in spec:
  for key in ['pos','a','b']:
   if key in a:a[key]=(np.array(a[key])+shift).tolist()
  if a['name'].endswith('_post'):a['b'][2]=1.104
 new=dict(part='pin',recipe=r,geometry=spec,back_m=0,score_s=segment_time(READY['left'],q)+segment_time(q,pre),physical_contact_error_m=best['final_position_error_m']);pieces,spec=candidate_pieces(new);set_env(pieces)
 if clear('left',q,.02)[0]and endpoint_clear(pieces)and not machine_hits(pieces):replacement.append(new);print('LOWER_PIN_CANDIDATE',newtcp.tolist(),flush=True)
source=[c for c in source if c['part']!='pin']+replacement;source.sort(key=lambda r:r['score_s']);(HERE/'compact-head-candidates.json').write_text(json.dumps(source,indent=2))
# Cached old pin paths must not survive a change to the grasp calibration.
p=HERE/'compact-path-candidates.json'
if p.exists():chosen=json.load(open(p));chosen.pop('pin',None);p.write_text(json.dumps(chosen,indent=2))
print('LOWER_PIN_READY',len(replacement),float(clearance),flush=True)
