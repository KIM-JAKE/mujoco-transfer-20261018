"""Resume only the pin plans from a persisted, screened ring layout."""
from five_layout_search import *
import sys
path=Path(sys.argv[1]);row=json.load(open(path));pieces=table(*row['table_center_m'])+pieces_from_specs([s for s in row['geometry']if not s.get('render_only')]);source=json.load(open(HERE/'compact-path-candidates.json'))['pin'][0]['recipe'];slots=sorted([s for s in row['stock_parts']if s['part']=='pin'],key=lambda x:x['index']);row['failures']=[]
for stock in slots:
 slot=stock['index']+1
 if any(s['slot']==slot for s in row['pin_slots']):continue
 R=np.array(stock['R'])@np.array(BANDS['pin']['localR']);template=min(row['pin_slots'],key=lambda s:np.linalg.norm(np.array(s['recipe']['tcp'])-stock['P']))['recipe']if row['pin_slots']else source;print('RESUME_PIN_BEGIN',row['label'],slot,flush=True);r,why=pin_recipe(template,stock['P'],pieces,row['recipes']['ring2']['q_exit'],R)
 if r is None:row['failures'].append(dict(part='pin',slot=slot,why=why));print('RESUME_PIN_REJECT',slot,why,flush=True)
 else:row['pin_slots'].append(dict(slot=slot,recipe=r));row['recipes']['pin']=r;print('RESUME_PIN_DONE',slot,r['duration_s'],flush=True)
 path.write_text(json.dumps(row,indent=2))
row['individual_paths_complete']=len(row['pin_slots'])==5 and not row['failures'];path.write_text(json.dumps(row,indent=2));print('RESUME_DONE',row['individual_paths_complete'],flush=True)
