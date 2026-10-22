"""Saved-v194 collision shadow comparison; does not step physics or move actors."""
from pathlib import Path
from contextlib import contextmanager
import json,time,types,sys,numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from aperture_path_check import path_openings
from isaac_kinematics import frame
from rgbd_geometry import axis_frame


def execute(c):
    root=Path('/workspace/project/mujoco');source=next((root/'isaac_port/runs').glob('*v194-*'))
    results=json.loads((source/'pipeline-results.json').read_text());log=np.load(source/'executed-actuator-commands.npz');names=results['dof_names'];right=[names.index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)];left=[names.index('left_arm_fr3v2_joint'+str(i))for i in range(1,8)]
    phases=log['pin_phase'];indices=np.where(phases=='ring1_right_carry_to_preinsert')[0];first=int(indices[0]);last=int(indices[-1]);before=first-1
    # retime_safe_polyline emits float32-quantized commands. Reconstruct every
    # exact issued curve row plus the separately checked initial encoder pose.
    qs=np.vstack([log['actual_dof_position'][before,right],log['command'][before,right],log['command'][first:last+1][:,right]]).astype(float)
    diagnostic=next(row for row in results['planning_diagnostics']if row.get('event')=='right_ring_checked_executed_curve' and row.get('label')=='carry_to_preinsert');widths=diagnostic['checked_openings_m']
    event=next(row for row in results['scattered_pick']['events']if row['event']=='ring1_right_lifted_contact_RGBD');observations=json.loads((source/'rgb-observations.json').read_text());observation=min(observations,key=lambda row:abs(row.get('captured_wall_s',-1)-event['captured_wall_s']));pose=observation['result']['pose'];G=np.linalg.inv(np.asarray(observation['hand_TCP_world']))@frame(pose['center_m'],axis_frame(pose['axis']))
    leftq=log['actual_dof_position'][before,left];left_width=float(log['actual_dof_position'][before,c['left_fi']].sum());checker=c['checker'];saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative);checker.detach_payload();checker.attach_payload(c['part_collision_templates']['ring1'],frame([0,0,.1034],np.eye(3))@G)
    try:
        with checker.other_arm_at(c['checkers']['left'],leftq,left_width):
            started=time.perf_counter();baseline=[]
            for width in widths:
                ok,why=checker.path(qs,float(width));baseline.append(dict(opening_m=float(width),clear=ok,rejection=why))
                if not ok:break
            baseline_s=time.perf_counter()-started;stats={};started=time.perf_counter();accepted,rejection=path_openings(checker,qs,widths,diagnostics=stats);optimized_s=time.perf_counter()-started
    finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
    same=bool(all(row['clear']for row in baseline)==accepted)
    result=dict(scope='Static collision query regression ONLY; probe does not advance physics or move physical actors',source_run=source.name,exact_float32_command_curve_reconstructed=True,curve_rows=len(qs),same_apertures=widths,full_robot_payload_pair_baseline=baseline,optimized_clear=accepted,optimized_rejection=rejection,clearance_verdicts_equal=same,baseline_wall_s=baseline_s,optimized_wall_s=optimized_s,speedup=baseline_s/max(optimized_s,1e-12),optimization_diagnostics=stats,full_task_pass=False,hardware_tested=False,uses_free_body_pose_for_control=False)
    print('APERTURE_COLLISION_STATIC_REGRESSION',json.dumps(result,default=str),flush=True)
    if not same:raise RuntimeError('Collision aperture optimization changed the clearance verdict')
    return result
