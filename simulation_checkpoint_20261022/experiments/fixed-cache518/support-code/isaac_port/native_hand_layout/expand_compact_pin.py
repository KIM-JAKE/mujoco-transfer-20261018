"""Additional central/front stock locations for the tested compact pin head."""
from shallow_supply_search import *
source=json.load(open(HERE/'compact-head-candidates.json'));template=next(c for c in source if c['part']=='pin');r0=template['recipe'];R=np.array(r0['R']);L=np.array(r0['localR']);lp=np.array(r0['localP']);P0=np.array(r0['tcp'])-R@L.T@lp;out=[]
for x,y,z in itertools.product([-.20,-.25],[-.24,-.32,-.40],[1.30,1.34]):
 tcp=np.array([x,y,z]);q=robust('left',tcp,R,np.array(BANDS['pin']['q_preinsert']))
 if q is None:continue
 P=tcp-R@L.T@lp;shift=P-P0;r=copy.deepcopy(r0);r.update(tcp=tcp.tolist(),q_pick=q.tolist());spec=copy.deepcopy(template['geometry'])
 for a in spec:
  for key in ['pos','a','b']:
   if key in a:a[key]=(np.array(a[key])+shift).tolist()
  if a['name'].endswith('_post'):a['b'][2]=1.104
 c=dict(part='pin',recipe=r,geometry=spec,back_m=0,score_s=segment_time(READY['left'],q)+segment_time(q,np.array(BANDS['pin']['q_preinsert'])),physical_contact_error_m=template['physical_contact_error_m']);pieces,_=candidate_pieces(c);set_env(table(-.25,-.3,1.1)+pieces)
 if not clear('left',q,.02)[0]:continue
 if machine_hits(pieces)or not endpoint_clear(pieces):continue
 out.append(c);print('EXPAND_PIN',tcp.tolist(),c['score_s'],flush=True)
source.extend(out);source.sort(key=lambda c:c['score_s']);(HERE/'compact-head-candidates.json').write_text(json.dumps(source,indent=2));print('EXPANDED_PIN_COUNT',len(out),flush=True)
