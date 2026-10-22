"""No-motion LEFT GPU proposal validation against current original full checker."""
from pathlib import Path
import json,time,numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from fr3_gpu_ik import model_sha256
from fr3_gpu_route_dual import propose_route
from fr3_path_timing import retime_safe_polyline,retime
from gpu_route_curve_validation import cubic_geometry,issued_geometry_samples

def execute(c):
    started=time.perf_counter();here=Path(__file__).resolve().parent;project=here.parents[2];work=project/'.local/curobo-gpu2';prior=json.loads((work/'robot-route-prior-left.json').read_text());source=here.parents[1]/'runs'/prior['source_run'];part='ring1';arm=c['left_arm'];raw=json.loads((work/'left-native-kinematics.json').read_text());source_arm=type('RobotModel',(),{k:np.asarray(raw[k])for k in ['base','tip','frames','axes','bounds']})()
    calibration_delta={key:float(np.max(abs(np.asarray(getattr(source_arm,key))-np.asarray(getattr(arm,key)))))for key in ['base','tip','frames','axes','bounds']}
    if any(calibration_delta[key]>1e-10 for key in ['frames','axes','bounds'])or any(calibration_delta[key]>1e-6 for key in ['base','tip']):raise RuntimeError('Nominal/current LEFT calibration mismatch '+str(calibration_delta))
    # The nominal USD export and live USD roundtrip can differ sub-micrometre.
    # Every target, payload and collision pose below uses the current live arm;
    # the source camera/start correspondence is independently bounded below.

    x=np.load(source/'executed-actuator-commands.npz');names=json.loads((source/'pipeline-results.json').read_text())['dof_names'];joints=lambda side:[names.index(side+'_arm_fr3v2_joint'+str(i))for i in range(1,8)];fingers=lambda side:[names.index(side+'_native_'+label)for label in ['left','right']];first,last=map(int,prior['source_encoder_rows']);k=max(0,first-1);actual=x['actual_dof_position'];q=actual[k,joints('left')];other_q=actual[k,joints('right')];width=float(actual[k,fingers('left')].sum());other_width=float(actual[k,fingers('right')].sum());raw_goal=x['command'][last,joints('left')].astype(float);bounds=np.asarray(arm.bounds);goal=np.clip(raw_goal,bounds[:,0],bounds[:,1]);correction=float(np.max(abs(goal-raw_goal)))
    if correction>2*np.finfo(np.float32).eps*np.max(np.maximum(1,abs(raw_goal))):raise RuntimeError('Source float32 goal exceeds model bounds beyond representational rounding')
    wall=float(x['time_s'][k]);observations=json.loads((source/'rgb-observations.json').read_text());available=[v for v in observations if v.get('part')==part and v.get('mode')=='held_left'and v['result'].get('pose')and float(v['captured_wall_s'])<=wall]
    if not available:raise RuntimeError('No directly measured held-ring observation before source carry')
    ob=min(available,key=lambda v:np.linalg.norm(np.asarray(v['hand_TCP_world'])[:3,3]-arm.fk(q)[:3,3])+np.linalg.norm(np.asarray(v['hand_TCP_world'])[:3,:3]-arm.fk(q)[:3,:3]));capture_difference=float(np.linalg.norm(np.asarray(ob['hand_TCP_world'])[:3,3]-arm.fk(q)[:3,3]))
    if capture_difference>.01:raise RuntimeError('Held camera does not correspond to carry start')
    pose=ob['result']['pose'];G=np.linalg.inv(ob['hand_TCP_world'])@frame(pose['center_m'],axis_frame(pose['axis']));checker=c['checkers']['left'];other=c['checkers']['right'];guard=getattr(checker,'same_arm_clearance',None)
    if guard is None or getattr(guard,'own_payload_clearance',None)is None:raise RuntimeError('Original same-arm and own-payload guards are mandatory')
    saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative);keys=['gpu_route_enabled','gpu_dual_arm_reference','gpu_reference_model_json','gpu_payload_witness_proposal','gpu_route_socket','gpu_route_timeout_s','gpu_rgbd_stock_points'];old={key:getattr(checker,key,None)for key in keys};checker.gpu_route_enabled=True;checker.gpu_dual_arm_reference=True;checker.gpu_reference_model_json=str(here/'gpu_planning/reference-right-native-kinematics.json');checker.gpu_payload_witness_proposal=True;checker.gpu_route_timeout_s=2.;checker.gpu_route_socket=str(work/'route-dual.sock');checker.gpu_rgbd_stock_points=None
    result=dict(state='static_LEFT_GPU_route_probe_only',source_run=source.name,source_encoder_row=k,nominal_live_calibration_max_component_difference=calibration_delta,source_model_sha256=model_sha256(source_arm),current_model_sha256=model_sha256(arm),robot_motion_executed=False,free_body_pose_input=False,full_task_pass=False,source_float32_goal_projection_magnitude_rad=correction,capture_start_translation_difference_m=capture_difference,proposal_model='optimistic class-surface and native-boundary witnesses; original full checker mandatory')
    try:
        checker.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
        with checker.other_arm_at(other,other_q,other_width):
            route=propose_route(checker,q,goal,width);result['GPU_statistics']=getattr(checker,'gpu_route_stats',{})
            if route is None:return dict(result,approved=False,reason='No bounded GPU proposal')
            result['route']=np.asarray(route).tolist();continuous,evidence=cubic_geometry(route,arm.bounds,step_rad=.010);result['cubic_geometry']=evidence;continuous_clear=False
            if continuous is not None:continuous_clear,why=checker.path(continuous,width);result['continuous_curve_check']=dict(clear=continuous_clear,reason=why)
            if continuous_clear:issued,_,timing=retime(route,.05,c['dt'],np.full(2,width/2),np.full(2,width/2),velocity_fraction=.95,acceleration=9.5,jerk=4750.,finger_command_speed=.0475,optimize=True)
            else:
                clear,why=checker.path(route,width);result['raw_path_check']=dict(clear=clear,reason=why)
                if not clear:return dict(result,approved=False,reason='Original full checker rejected raw and cubic GPU proposal')
                issued,_,timing=retime_safe_polyline(route,c['dt'],np.full(2,width/2),np.full(2,width/2),scale=1.,velocity_fraction=.95,acceleration=9.5,jerk=4750.)
            issued=np.vstack([q,np.asarray(issued[1:],dtype=np.float32)]);geometry,sampling=issued_geometry_samples(issued,step_rad=.003);resolution=getattr(checker,'resolution_rad',.035);checker.resolution_rad=.010
            try:clear,why=checker.path(geometry,width)
            finally:checker.resolution_rad=resolution
            result.update(approved=bool(clear),issued_curve_check=dict(clear=clear,reason=why),continuous_cubic_used=bool(continuous_clear),issued_samples=len(issued),issued_geometry_sampling=sampling,retiming=timing,original_full_native_self_fixed_pair_payload_checks=True,own_payload_vs_proximal_fingers_palm_and_arm_checked=True,same_arm_statistics=dict(guard.stats),own_payload_statistics=dict(guard.own_payload_clearance.stats),full_FR3_numpy_command_validation=True)
    finally:
        checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
        for key,value in old.items():setattr(checker,key,value)
    result['elapsed_s']=time.perf_counter()-started;print('GPU_LEFT_NATIVE_ROUTE_STATIC_PROBE',json.dumps(result),flush=True);return result
