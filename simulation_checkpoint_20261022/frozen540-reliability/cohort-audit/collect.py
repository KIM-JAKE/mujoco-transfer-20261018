"""Read-only collector for exactly the15 COHORT_PLAN full-task trials.

No physics, services, sources, publication, or optimization is performed.
Run with the repository's existing NumPy-capable host interpreter.
"""
from pathlib import Path
import argparse,ast,csv,hashlib,json,re
import numpy as np
ap=argparse.ArgumentParser();ap.add_argument('--plan',type=Path,default=Path('/home/gpuadmin/workspace/jake/isaac-fixed-c90a297e-20261022/COHORT_PLAN.json'));ap.add_argument('--runs',type=Path,default=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs'));ap.add_argument('--out',type=Path,default=Path(__file__).parent);args=ap.parse_args();args.out.mkdir(parents=True,exist_ok=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
aggregate=lambda m:hashlib.sha256(json.dumps(m,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def read(p):return json.loads(p.read_text())
def compact_pose(p):
 if not isinstance(p,dict):return None
 keys=['part','center_m','axis','uses_depth_sensor','uses_simulator_body_poses','front_back_from_RGBD','rms_m','depth_points','measured_classification_diameter_m','visible_arc_coverage_deg','robust_cap_circle_from_pixels','cap_edge_coverage_deg','cap_circular_edge_inliers','cap_circular_edge_rms_m','lateral_from_previous_RGBD_grasp_and_encoders','postrelease_known_class_plane_roi']
 return {k:p[k]for k in keys if k in p}
def compact_event(e):
 keys=['event','part','canonical_rows','rows','fresh_entry_rows','fresh_exit_rows','original_dt_s','fresh_RGBD_grasp','fresh_public_RGBD_grasp','exact_original_middle','original_actual_encoders_all_widths_both_other_contexts_and_payload_guards','original_full_payload_pair_guard','original_full_current_issued_guard_retained','static_certificate_hits','static_certificate_misses','native_self_certificate_hits','own_payload_certificate_hits','own_certificate_misses','wall_s','reason','clear','all_current_pairs_checked','changed_component_checks_retained','uses_free_body_poses']
 return {k:e[k]for k in keys if k in e}
plan=read(args.plan);trials=plan['trials'];assert len(trials)==15 and len({t['version']for t in trials})==15;assert {x['case']for x in trials}=={'nominal','plus','minus'}
for case in ['nominal','plus','minus']:assert sum(t['case']==case for t in trials)==5
base=next(args.runs.glob('*v540-*'));basecfg=read(base/'trial-config.json');basemanifest=read(base/'frozen-source-manifest.json')['files'];basejson={k:v for k,v in read(base/'code-manifest.json').items()if k.endswith('.json')};assert len(basemanifest)==425 and aggregate(basemanifest)==plan['source_aggregate_sha256'];assert len(basejson)==7
casecfg={};rows=[]
for trial in trials:
 version=trial['version'];matches=list(args.runs.glob(f'*v{version}-*'));row=dict(**trial,status='PENDING',eligible_full_task=None)
 if not matches:row['pending_reason']='run_directory_missing';rows.append(row);continue
 if len(matches)!=1:raise RuntimeError('Ambiguous version'+str(version))
 run=matches[0];row['run']=run.name;rp=run/'pipeline-results.json';cfgp=run/'trial-config.json'
 if cfgp.exists():
  cfg=read(cfgp);diff={k:dict(baseline=basecfg.get(k),actual=cfg.get(k))for k in basecfg.keys()|cfg.keys()if basecfg.get(k)!=cfg.get(k)};row['config']=dict(SHA256=sha(cfgp),differences_from_nominal540=diff,variation_only=set(diff)<= {'fixed_supply_pose_variation'},PIN_saved_canonical_carry=cfg.get('pin_saved_canonical_carry'),fixed_supply_pose_variation=cfg.get('fixed_supply_pose_variation'));casecfg.setdefault(trial['case'],cfg);row['config']['same_case_repeat_exact']=cfg==casecfg[trial['case']];row['eligible_full_task']=not cfg.get('pin_only_component',False)and not cfg.get('diagnostic_rings_only',False);assert row['eligible_full_task'],'Plan unexpectedly containscomponent'+str(version);assert cfg.get('pin_saved_canonical_carry')is False,'PINcarryconfigchanged'+str(version)
 else:row['pending_reason']='config_not_flushed';rows.append(row);continue
 if not rp.exists():row['pending_reason']='authoritative_result_not_flushed';rows.append(row);continue
 try:r=read(rp)
 except json.JSONDecodeError:row['pending_reason']='authoritative_result_incomplete_JSON';rows.append(row);continue
 fm=read(run/'frozen-source-manifest.json');fm=fm.get('files',fm);cm=read(run/'code-manifest.json');jm={k:v for k,v in cm.items()if k.endswith('.json')};mismatch=[k for k,v in cm.items()if not(run/'code_snapshot'/k).is_file()or sha(run/'code_snapshot'/k)!=v];sourceok=(fm==basemanifest and jm==basejson and not mismatch and row['config']['variation_only']and row['config']['same_case_repeat_exact']);row['source_identity']=dict(frozen_python_count=len(fm),frozen_python_aggregate_SHA256=aggregate(fm),matches_plan_python_aggregate=aggregate(fm)==plan['source_aggregate_sha256'],matches540_python_map=fm==basemanifest,fixed_JSON_count=len(jm),fixed_JSON_map_matches540=jm==basejson,captured_snapshot_entries_checked=len(cm),captured_snapshot_byte_mismatches=mismatch,eligible_same_frozen_configuration=sourceok)
 if not sourceok:row['status']='INVALID_FROZEN_SOURCE_OR_CONFIG'
 else:row['status']='PASS'if r.get('full_task_pass')is True and r.get('vision_end_to_end_pass')is True else'FAIL'
 row['authoritative']=dict(state=r.get('state'),full_task_pass=r.get('full_task_pass'),vision_end_to_end_pass=r.get('vision_end_to_end_pass'),wall_s=r.get('wall_verified_elapsed_s'),physics_report_s=r.get('simulation_elapsed_s'),time_goal_under20=r.get('end_to_end_under20'),time_goal_under30=r.get('end_to_end_under30'),hardware_tested=r.get('hardware_tested'),part_attachments=r.get('part_attachments'),workpiece_teleports_after_initialization=r.get('workpiece_teleports_after_initialization'))
 scatter=r.get('scattered_pick',{});pin=scatter.get('pin',r.get('pin_skill_progress',{}));row['PIN']=dict(single_stroke_pass=pin.get('pin_single_stroke_pass'),state=pin.get('state'),completion_source=pin.get('retention_completion_source'),public_SDK_release_withdraw=pin.get('pin_postrelease_SDK_completion'),fresh_postrelease_RGBD=pin.get('pin_postrelease_RGBD_retention'),fresh_postrelease_qualified_observations=pin.get('pin_postrelease_qualified_observations',[]),fresh_full_depth_RGBD_error=pin.get('RGBD_full_depth_check'),fresh_full_depth_current_grasp=pin.get('pin_full_depth_current_grasp'),saved_carry_configured=False,saved_carry_execution_evidence=pin.get('pin_saved_canonical_carry'),terminal_branch=pin.get('pin_canonical_terminal_branch'))
 parts={p['part']:p for p in scatter.get('parts',[])if p.get('part')in['ring1','ring2']};phases=[];tracep=run/'physx-sequence-trace.npz'
 if tracep.exists():
  with np.load(tracep)as tr:phases=np.unique(tr['pin_phase']).tolist();row['last_recorded_trace_physics_s']=float(tr['simulation_time_s'][-1]);row['last_recorded_trace_wall_s']=float(tr['time_s'][-1])
 row['rings']={}
 # Successful structured parts are retained only if the task plugin returned; a late exception loses that local list.
 for part in ['ring1','ring2']:
  p=parts.get(part);final=r.get('final_checks',{}).get(part);stage=dict(structured_completed_part_available=p is not None,whole_final_check_available=final is not None,whole_final_check_EVALUATION_ONLY=final,final_pass_inferred_if_check_missing=False)
  if p:
   stage.update(insertion_fresh_RGBD=compact_pose(p.get('insertion_fresh_RGBD')),released_retention_RGBD=compact_pose(p.get('released_retention_RGBD')))
   if part=='ring1':
    rd=p.get('right_direct',{});stage['right_direct_returned_state']=rd.get('state');stage['right_direct_component_pass']=rd.get('full_task_pass');stage['right_direct_reported_attachments']=rd.get('workpiece_attachments')
  else:
   stage['missing_local_part_report_reason']='Task raised before returning its local parts list; do not manufacture final part checks.';stage['later_PIN_skill_reached']=bool(pin.get('events'));stage['evidence_scope']='Later PIN stage under frozen source and ring1 command-phase markers document progress, not final ring retention.'
  if part=='ring1':stage['recorded_command_stage_markers']=[x for x in phases if x in ['ring1_right_axial_insert','ring1_right_full_depth_RGBD','ring1_right_release_after_full_depth','ring1_right_released_retention_RGBD','ring1_right_world_down_withdraw']]
  row['rings'][part]=stage
 diagnostics=r.get('planning_diagnostics',[]);cache_events=[e for e in diagnostics if e.get('event','')in ['saved_SMALL_donor_proposed','saved_SMALL_donor_admitted','saved_SMALL_donor_rejected','saved_SMALL_LEFT_carry_admitted','saved_SMALL_LEFT_carry_rejected','fixed_saved_canonical_carry_admitted','fixed_saved_canonical_carry_rejected','fixed_saved_fixture_branch_admitted','fixed_saved_fixture_branch_rejected']];row['saved_paths']=dict(rings=[compact_event(e)for e in cache_events],ring2=dict(saved_donor_admitted=any(e.get('event')=='saved_SMALL_donor_admitted'for e in cache_events),saved_LEFT_carry_admitted=any(e.get('event')=='saved_SMALL_LEFT_carry_admitted'for e in cache_events),evidence_scope='Actual original-guard admission events, not an independent new command-byte proof'),ring1=dict(saved_fixture_branch_admitted=any(e.get('event')=='fixed_saved_fixture_branch_admitted'for e in cache_events),saved_canonical_carry_admitted=any(e.get('event')=='fixed_saved_canonical_carry_admitted'for e in cache_events),evidence_scope='Actual original-guard admission events'),PIN_configured=False,PIN_execution_claim=False)
 err=r.get('error','');exception_lines=[x.strip()for x in err.splitlines()if re.match(r'^[A-Za-z_][A-Za-z0-9_.]*(Error|Blocked|Exception):',x.strip())];last=exception_lines[-1]if exception_lines else err[-2500:];pairs=[]
 for query,actor in re.findall(r"'query': '([^']+)'.*?'actor': '([^']+)'",last):
  pair=dict(query=query,actor=actor)
  if pair not in pairs:pairs.append(pair)
 failure_stage=None
 if row['status']=='FAIL':failure_stage='final_empty_RIGHT_retreat_preflight'if'final empty RIGHT retreat'in err else'PIN_precision_or_insertion'if'pin_'in last else'other_original_guard_or_task_failure'
 row['retreat_and_failure']=dict(final_empty_RIGHT_retreat_command_phase_present='final_empty_RIGHT_clearance_retreat'in phases,final_whole_retention_phase_present='loose_all_parts_retention'in phases,authoritative_whole_final_checks_available='final_checks'in r,stage=failure_stage,last_exception=last[:6000]if err else None,reported_collision_pairs=pairs,postrelease_PIN_can_be_confirmed_even_when_whole_task_failed=bool(pin.get('pin_postrelease_RGBD_retention',{}).get('confirmed')and row['status']=='FAIL'))
 row['raw_input_SHA256']={p.name:sha(p)for p in [rp,cfgp,run/'frozen-source-manifest.json',run/'code-manifest.json',run/'rgb-observations.json',run/'executed-actuator-commands.npz',tracep]if p.exists()};rows.append(row)
counts={case:{s:sum(r['case']==case and r['status']==s for r in rows)for s in ['PASS','FAIL','PENDING','INVALID_FROZEN_SOURCE_OR_CONFIG']}for case in ['nominal','plus','minus']};overall={s:sum(r['status']==s for r in rows)for s in ['PASS','FAIL','PENDING','INVALID_FROZEN_SOURCE_OR_CONFIG']}
result=dict(scope='Exactly the15fullthree-part COHORT_PLAN trials. PINcomponents535..541except540 andfuture556+ excluded.',plan_path=str(args.plan),plan_SHA256=sha(args.plan),expected_frozen425_aggregate_SHA256=plan['source_aggregate_sha256'],expected_fixed_JSON_count=7,expected_source_input_count=432,all_policy_changes_forbidden=plan['policy_changes_during_cohort_forbidden'],hardware_tested=False,parallel_timing_not_benchmark=True,timing_interpretation='Wall times include captured/perception/planning waits. Intentionally parallelGPU2/GPU3 trials are descriptive reliability records, not isolated speed benchmarks.',counting='PASS requires authoritative full_task_pass AND vision_end_to_end_pass and matching frozen source/config. Late retreat failure remains whole FAIL even if public PIN retention passed. Missing final ring checks never become invented passes.',per_case_counts=counts,overall_counts=overall,all15_completed=overall['PENDING']==0,all15_same_frozen_source_qualified=all(r.get('source_identity',{}).get('eligible_same_frozen_configuration')for r in rows)if overall['PENDING']==0 else False,trials=rows)
(args.out/'cohort-audit.json').write_text(json.dumps(result,indent=2)+'\n')
with(args.out/'cohort-summary.csv').open('w',newline='')as f:
 cols=['version','case','repetition','gpu','status','wall_s','physics_report_s','PIN_public_retention_confirmed','PIN_position_error_mm','PIN_axis_error_mrad','failure_stage'];w=csv.DictWriter(f,fieldnames=cols);w.writeheader()
 for r in rows:
  p=r.get('PIN',{}).get('fresh_postrelease_RGBD')or{};a=r.get('authoritative',{});w.writerow(dict(version=r['version'],case=r['case'],repetition=r['repetition'],gpu=r['gpu'],status=r['status'],wall_s=a.get('wall_s'),physics_report_s=a.get('physics_report_s'),PIN_public_retention_confirmed=p.get('confirmed'),PIN_position_error_mm=1000*p['position_error_m']if'position_error_m'in p else None,PIN_axis_error_mrad=1000*p['axis_error_rad']if'axis_error_rad'in p else None,failure_stage=r.get('retreat_and_failure',{}).get('stage')))
print(json.dumps(dict(per_case=counts,overall=overall,all15_completed=result['all15_completed'],output=str(args.out/'cohort-audit.json')),indent=2))
