"""Summarize fresh random-bin experiments without treating fast failure as speedup."""
from pathlib import Path
import datetime
import json
import re
import numpy as np

HERE = Path(__file__).resolve().parent
RUNS = HERE.parents[1] / 'runs'
rows = []
stage_events={};active=None
for log_file in sorted(HERE.glob('run_realtime_daemon-native-roas-table-opt-service20261021*.log')):
    for line in log_file.read_text(errors='replace').splitlines():
        if line.startswith('REALTIME_JOB_START '):
            active=line.split()[1];stage_events[active]=[]
        match=re.match(r'SCATTERED_PICK (\S+) ([0-9.]+)',line)
        if match and active:stage_events[active].append(dict(event=match[1],wall_s=float(match[2])))
        match=re.match(r'SCATTERED_PLANNING (\S+) seconds ([0-9.]+)',line)
        if match and active:stage_events[active].append(dict(event=match[1]+'_pickup_planning',duration_s=float(match[2])))
        match=re.match(r'LOOSE_PAIRED_HANDOVER_PLAN (\S+) ([0-9.]+) (True|False)',line)
        if match and active:stage_events[active].append(dict(event=match[1]+'_paired_handover_planning',duration_s=float(match[2]),found=match[3]=='True'))
for folder in sorted(RUNS.glob('native-roas-table-scatter-opt-*')):
    report_file = folder / 'pipeline-results.json'
    if not report_file.exists():
        continue
    report = json.loads(report_file.read_text())
    config = json.loads((folder / 'trial-config.json').read_text())
    if config.get('clearance_diagnostic_only'):continue
    observations = json.loads((folder / 'rgb-observations.json').read_text())
    goal_file=folder/'arm-plan.json'
    goals=json.loads(goal_file.read_text())['goals'] if goal_file.exists() else {}
    seating_frames=[]
    for observation in observations:
        if observation['mode']!='held_left':continue
        pose=observation.get('result',{}).get('pose');part=observation.get('part')
        if not pose or part not in goals:continue
        delta=np.asarray(pose['center_m'])-np.asarray(goals[part]['P'])
        error=float(np.linalg.norm(delta));angle=float(np.arccos(np.clip(pose['axis'][2],-1,1)))
        seating_frames.append(dict(part=part,delivered_wall_s=observation.get('delivered_wall_s'),
            measured_position_error_mm=error*1000,measured_axis_error_deg=float(np.degrees(angle)),
            visual_full_depth_gate=bool(error<.0015 and angle<.05),
            does_not_by_itself_verify_release_or_retention=True))
    pick_frames = []
    for observation in observations:
        if observation['mode'] != 'held' or observation.get('part') not in ['ring1', 'ring2']:
            continue
        pose = observation.get('result', {}).get('pose')
        if pose is None:
            continue
        diameter, half = (.075, .00775) if observation['part'] == 'ring1' else (.062, .007)
        nz = float(pose['axis'][2])
        bottom = pose['center_m'][2] - diameter / 2 * np.sqrt(max(0., 1 - nz*nz)) - half * abs(nz)
        pick_frames.append(dict(part=observation['part'],
            delivered_wall_s=observation.get('delivered_wall_s'),
            ready_wall_s=observation.get('ready_wall_s'),
            observed_center_m=pose['center_m'],
            observed_bottom_clearance_mm=float((bottom - 1.210918206667893) * 1000),
            visible_above_rim=bool(bottom > 1.210918206667893 + .003)))
    row = dict(run=folder.name, seed=config['seed'],
        randomized_ring_count=report.get('randomized_ring_count'),
        full_task_pass=bool(report.get('full_task_pass')),
        under20=bool(report.get('end_to_end_under20')),
        wall_s=report.get('wall_verified_elapsed_s'),
        state=report['state'],
        stop_reason=report.get('error', '').strip().splitlines()[-1] if report.get('error') else None,
        encoder_speed_audit=report.get('encoder_motor_speed_audit'),
        visual_pick_observations=pick_frames,
        sensor_input='CPU Isaac scene RGB-D; ideal depth, not RealSense hardware',
        force_per_pad_n=config.get('native_grip_pad_force_n', 30.),
        native_finger_convex_decomposition=config.get('native_finger_convex_decomposition', False),
        physics_substeps=config.get('scattered_physics_substeps', 2))
    row['implementation_error']=bool(re.search(r'(ValueError|TypeError|AttributeError|NameError|IndexError|KeyError|ModuleNotFoundError|ImportError|UnboundLocalError|SyntaxError):',report.get('error','')))
    row['scope']='one_ring_component' if config.get('one_ring_only') else 'full_task' if config.get('scattered_full_task') else 'pickup_only'
    row['fresh_seating_image_checks']=seating_frames
    row['stage_events']=stage_events.get(folder.name,[])
    row['one_ring_component_accepted']=bool(report.get('one_ring_component_accepted'))
    rows.append(row)
value = dict(updated_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    target_wall_s=20, timing='First RGB-D capture through all-part release, withdrawal and >=0.5 s retention; all perception and planning waits included',
    physics='Isaac Sim CPU PhysX; no GPU',
    randomization='Ten mixed rings (five per size), independent bin XY / SO(3), shuffled gravity drop before first capture',
    controller_has_true_free_body_poses=False,
    tests=len(rows), task_trials=sum(not r['implementation_error'] for r in rows),
    full_task_trials=sum(not r['implementation_error'] and r['scope']=='full_task' for r in rows),
    pickup_only_trials=sum(not r['implementation_error'] and r['scope']=='pickup_only' for r in rows),
    software_error_trials=sum(r['implementation_error'] for r in rows),
    distinct_setup_seeds=sorted({r['seed'] for r in rows if not r['implementation_error']}),
    successful_full_tasks=sum(r['full_task_pass'] for r in rows),
    under20_passes=sum(r['under20'] for r in rows),
    fast_failures_are_not_task_time_improvements=True, hardware_tested=False,
    rows=rows)
# Compare measured stage times only when the ten settled starting transforms
# match exactly. This comparison is not a completed assembly cycle metric.
a,b='native-roas-table-scatter-opt-v36-401','native-roas-table-scatter-opt-v40-401'
if {a,b}<={r['run'] for r in rows}:
    setups=[json.loads((RUNS/run/'randomization-evidence.json').read_text()) for run in [a,b]]
    poses=[{r['path']:r['settled_transform'] for r in setup['records']} for setup in setups]
    if poses[0]!=poses[1]:raise RuntimeError('Stage comparison requires identical initial physical scene')
    def event(run,key,field):
        return next(x[field] for x in stage_events[run] if x['event']==key)
    value['matched_scene_stage_comparison']=dict(before=a,after=b,all_ten_settled_transforms_identical=True,
        pickup_planning_s=[event(run,'ring1_pickup_planning','duration_s') for run in [a,b]],
        lift_motion_finished_wall_s=[event(run,'loose_ring_lift','wall_s') for run in [a,b]],
        both_full_tasks_failed=True,not_a_full_task_speedup=True)
destination = HERE / 'bin-picking/scatter-optimization-20261021.json'
destination.write_text(json.dumps(value, indent=2) + '\n')
print(json.dumps({k: value[k] for k in ['tests', 'successful_full_tasks', 'under20_passes']}, indent=2))
print(destination)
