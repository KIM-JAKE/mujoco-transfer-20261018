"""Evaluation only: compare completed fresh CPU PhysX component records."""
from pathlib import Path
import numpy as np,json
BASE=Path('/home/gpuadmin/workspace/jake/mujoco/.local/thread-component-494')
R={n:json.loads((BASE/f'threads-{n}/result.json').read_text())for n in [0,2]}
Z={n:np.load(BASE/f'threads-{n}/evaluation-only-state-trace.npz',allow_pickle=False)for n in [0,2]}
assert R[0]['source_seals']==R[2]['source_seals'] and R[0]['source_block_seals']==R[2]['source_block_seals']
assert R[0]['issued_field_sha256']==R[2]['issued_field_sha256']
assert np.array_equal(Z[0]['command'],Z[2]['command']) and np.array_equal(Z[0]['phase'],Z[2]['phase'])
assert R[0]['tensor_body_paths']==R[2]['tensor_body_paths']
a,b=Z[0]['trace_bodies'].astype(float),Z[2]['trace_bodies'].astype(float)
qa,qb=a[...,3:7],b[...,3:7];qa=qa/np.linalg.norm(qa,axis=-1)[...,None];qb=qb/np.linalg.norm(qb,axis=-1)[...,None]
angles=2*np.arccos(np.clip(abs(np.sum(qa*qb,axis=-1)),0,1));angles=np.where(np.all(a[...,3:7]==b[...,3:7],axis=-1),0.,angles);body_trans=np.linalg.norm(a[...,:3]-b[...,:3],axis=-1)
start,end=2182,3142
ids=R[0]['SDK_names'];finger=[i for i,n in enumerate(ids)if '_native_'in n];arm=[i for i in range(len(ids))if i not in finger]
qdiff=abs(Z[0]['actual_q'].astype(float)-Z[2]['actual_q'].astype(float));vdiff=abs(Z[0]['actual_v'].astype(float)-Z[2]['actual_v'].astype(float))
physics={n:R[n]['timed']['physics_profile']['physics_s']for n in R}
value={'component_only':True,'full_task_qualified':False,'same_input_and_literal_controller_seals':True,'same_exact_command_bytes':True,'all_SDK_q_bit_identical':bool(np.array_equal(Z[0]['actual_q'],Z[2]['actual_q'])),'all_SDK_v_bit_identical':bool(np.array_equal(Z[0]['actual_v'],Z[2]['actual_v'])),'all_body_transform_arrays_bit_identical':bool(np.array_equal(Z[0]['trace_bodies'],Z[2]['trace_bodies'])),'all_link_transform_arrays_bit_identical':bool(np.array_equal(Z[0]['trace_links'],Z[2]['trace_links'])),'contact_rich_both':all(r.get('contact_rich_prefix_qualified',False)for r in R.values()),'errors':{n:r['error']for n,r in R.items()},'timed_sim_s':2.,'physics_wall_s':physics,'physics0_over_physics2':physics[0]/physics[2],'body_max_translation_drift_m':float(body_trans.max()),'body_max_axis_drift_rad':float(angles.max()),'timed_max_arm_q_drift_rad':float(qdiff[start:end,arm].max()),'timed_max_finger_q_drift_m':float(qdiff[start:end,finger].max()),'timed_max_arm_v_drift_rad_s':float(vdiff[start:end,arm].max()),'timed_max_finger_v_drift_m_s':float(vdiff[start:end,finger].max()),'cold_wall_s':{n:r['cold_setup']['wall_s']for n,r in R.items()},'warmup':{n:r['warmup']for n,r in R.items()},'timed':{n:r['timed']for n,r in R.items()},'qualifications':{n:{k:r.get(k)for k in ['small_lift_evaluation_m','closed_contact_controls_after_breakaway','cross_arm_peak_n','motor_velocity_audit','command_audit']}for n,r in R.items()},'caveat':'Two-second contact-rich component only. Different dispatcher settings may produce physical drift; a faster setting requires unchanged full-task acceptance test before use.'}
(BASE/'comparison.json').write_text(json.dumps(value,indent=2)+'\n');print(json.dumps({k:v for k,v in value.items()if k not in ['warmup','timed']},indent=2))
