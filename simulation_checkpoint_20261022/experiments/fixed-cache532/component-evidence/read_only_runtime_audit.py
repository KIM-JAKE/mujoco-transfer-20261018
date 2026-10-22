"""Read-only completed-run audit; never imported by controller or physics worker."""
from pathlib import Path
import json,hashlib,sys
ROOT=Path('/home/gpuadmin/workspace/jake/mujoco');OUT=Path(__file__).parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
rows=[]
for version in map(int,sys.argv[1:]or['532','533','534']):
 matches=list((ROOT/'isaac_port/runs').glob(f'*v{version}-*'))
 if not matches:continue
 run=matches[0];cfg=json.loads((run/'trial-config.json').read_text());row=dict(version=version,run=run.name,feature_configured=cfg.get('pin_postrelease_cap_plane_roi'),physical_jobs_launched_by_auditor=0)
 rp=run/'pipeline-results.json'
 if not rp.exists():row['status']='running_or_not_flushed';rows.append(row);continue
 r=json.loads(rp.read_text());row.update(status=r.get('state'),full_task_pass=r.get('full_task_pass'),vision_end_to_end_pass=r.get('vision_end_to_end_pass'),hardware_tested=r.get('hardware_tested'),wall_s=r.get('wall_verified_elapsed_s'),physics_s=r.get('simulation_elapsed_s'),time_goal_met=bool(r.get('end_to_end_under20')),source_manifest_SHA256=sha(run/'frozen-source-manifest.json'),result_SHA256=sha(rp))
 pin=r.get('scattered_pick',{}).get('pin',r.get('pin_skill_progress',{}));row['public_postrelease']=pin.get('pin_postrelease_RGBD_retention');row['postrelease_SDK']=pin.get('pin_postrelease_SDK_completion');row['completion_source']=pin.get('retention_completion_source');row['postrelease_qualified_observations']=pin.get('pin_postrelease_qualified_observations',[])
 obs=json.loads((run/'rgb-observations.json').read_text());frames=[]
 for i,o in enumerate(obs):
  pose=o.get('result',{}).get('pose',{});e=pose.get('postrelease_known_class_plane_roi')
  if e:
   frames.append(dict(index=i+1,capture_wall_s=o.get('captured_wall_s'),ready_wall_s=o.get('ready_wall_s'),delivery_wall_s=o.get('delivered_wall_s'),capture_to_ready_s=o.get('ready_wall_s',0)-o.get('captured_wall_s',0),circle_from_pixels=pose.get('robust_cap_circle_from_pixels'),propagated_XY=pose.get('lateral_from_previous_RGBD_grasp_and_encoders'),coverage_deg=pose.get('cap_edge_coverage_deg'),edge_inliers=pose.get('cap_circular_edge_inliers'),edge_RMS_m=pose.get('cap_circular_edge_rms_m'),center_m=pose.get('center_m'),axis=pose.get('axis'),proposal=e,packet_path=str(Path(Path(o['work']).name)/'rgbd.npz')))
 row['actual_fresh_ROI_frames']=frames;row['actual_branch_exercised']=bool(frames)
 if row['public_postrelease']:
  assert frames and row['public_postrelease']['confirmed']
  for f in frames:assert f['circle_from_pixels']and not f['propagated_XY']and f['coverage_deg']>=240 and f['edge_RMS_m']<=.00006 and 0<=f['capture_to_ready_s']<=3
 row['source_files']={n:sha(run/'code_snapshot'/n)for n in ['direct_pin_skill.py','process_vision.py','realtime_camera_worker.py','rgbd_fast_tracker.py']};rows.append(row)
result=dict(read_only_audit=True,physics_steps=0,new_GPU_jobs=0,hardware_tested=False,cohort_concurrency='532NOM and533PLUS intentionally ran concurrently on assignedGPU2/3; runtime differences are descriptive, not isolated benchmarks',rows=rows);(OUT/'actual-completed-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
