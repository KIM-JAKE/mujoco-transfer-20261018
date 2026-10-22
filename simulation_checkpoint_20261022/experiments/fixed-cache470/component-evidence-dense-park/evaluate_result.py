"""Read-only actual results; missing final verdict is never a success."""
import json,sys
from pathlib import Path
r=Path(sys.argv[1]);p=r/'pipeline-results.json'
if not p.exists():print(json.dumps(dict(run=r.name,final_verdict='NOT_WRITTEN')));raise SystemExit()
d=json.loads(p.read_text());out={k:d.get(k)for k in ['full_task_pass','vision_end_to_end_pass','wall_verified_elapsed_s','simulation_elapsed_s']};out['run']=r.name
prep=d.get('fixed_saved_route_preparation',{});out['RIGHT_domains']=[dict(domain=x.get('domain_label'),static_rows=x.get('static_certified_unique_rows'),offline_proof=x.get('offline_proof'),own=x.get('own_envelope_preparation'))for x in prep.get('independently_compiled_public_G_domains',[])]
for x in d.get('planning_diagnostics',[]):
 if x.get('event')=='large_goal_aware_empty_LEFT_park_admitted':
  out['park']=dict(candidate=x.get('candidate_label'),candidates=x.get('candidate_count'),actual_LEFT_route=x.get('issued_guard'),proofs=[z.get('future_entire_saved_RIGHT_middle')for z in x.get('future_terminal_proofs',[])])
 if x.get('event')=='fixed_saved_canonical_carry_admitted':out['postpark_live_middle']=x
for part in d.get('scattered_pick',{}).get('parts',[]):
 for x in part.get('right_direct',{}).get('events',[]):
  if x.get('event')=='ring1_right_goal_aware_empty_left_park':out['park_actual_timing']=x.get('parking')
print(json.dumps(out,indent=2))
