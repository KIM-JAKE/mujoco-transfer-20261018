from pathlib import Path
import json,sys,numpy as np,ast
R=Path('/home/gpuadmin/workspace/jake/mujoco');P=next((R/'isaac_port/runs').glob('*v530*'));sys.path.insert(0,str(P/'code_snapshot'));from loaded_contact_aperture import LoadedContactAperture
r=json.load(open(P/'pipeline-results.json'));b=next(v for v in r['planning_diagnostics']if v.get('event')=='small_loaded_receiver_encoder_geometry_bound');q=np.array([.03089151531457901]*2);common=dict(side='left',part='ring2',payload_path='small',encoder_samples=b['measured_finger_samples_m'],sample_times_s=b['sample_times_s'],contact=b['fresh_bilateral_CAD_contact'],measured_grasp=b['measured_grasp'],control_dt_s=1/480,capture_age_physics_s=b['capture_age_physics_s']);old=LoadedContactAperture(**common);new=LoadedContactAperture(**common,half_interval_m=.0005);checks={}
try:old.check(q);raise AssertionError('Original boundary should fail')
except RuntimeError:checks['original530_primary_bound_failure_reproduced']=True
new.check(q);checks['expanded_original_exact_lookahead_admits_same_actual']=True
reach=new.speed_cap_m_s*new.dt
for a in [new.lower+reach-1e-9,new.upper-reach+1e-9]:
 try:new.check(a);raise AssertionError('New interval must remain strictly enforced')
 except RuntimeError:pass
checks['beyond_expanded_interval_strictly_rejected_no_acceptance_epsilon']=True
assert old.speed_cap_m_s==new.speed_cap_m_s==.0475 and old.dt==new.dt==1/480;checks['speed_tick_and_coupling_gates_unchanged']=True
p=Path('/tmp/fixed533-small-wide-proof');(p/'critical-tests.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(checks))
