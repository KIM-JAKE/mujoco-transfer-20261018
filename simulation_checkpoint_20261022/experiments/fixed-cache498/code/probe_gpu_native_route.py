"""No-motion proof of an optional GPU carry proposal using original PhysX."""
from pathlib import Path
import json,time,numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame,cloud
from fr3_gpu_ik import model_sha256
from fr3_gpu_route import propose_route
from fr3_path_timing import retime_safe_polyline,retime
from gpu_route_curve_validation import cubic_geometry,issued_geometry_samples

def execute(c):
    start=time.perf_counter();here=Path(__file__).resolve().parent;name=c['config'].get('gpu_route_probe_source_run','native-roas-table-scatter-stable-v240-camera-bank-parallel-all3-446');source=here.parents[1]/'runs'/name;part='ring1'
    raw=json.loads((source/'robot-kinematics.json').read_text());source_arm=type('RobotModel',(),{k:np.asarray(raw[k])for k in ['base','tip','frames','axes','bounds']})()
    if model_sha256(source_arm)!=model_sha256(c['arm']):raise RuntimeError('Source encoder/camera robot calibration differs from current model')
    x=np.load(source/'executed-actuator-commands.npz');report=json.loads((source/'pipeline-results.json').read_text());names=report['dof_names'];indices=np.flatnonzero(x['pin_phase']==part+'_right_carry_to_preinsert')
    if not len(indices):raise RuntimeError('No representative carry command interval')
    k=int(indices[0]-1);last=int(indices[-1]);joints=lambda side:[names.index(side+'_arm_fr3v2_joint'+str(i))for i in range(1,8)];fingers=lambda side:[names.index(side+'_native_'+key)for key in ['left','right']]
    right=x['actual_dof_position'][k,joints('right')];left=x['actual_dof_position'][k,joints('left')];goal=x['command'][last,joints('right')];width=float(x['actual_dof_position'][k,fingers('right')].sum());leftwidth=float(x['actual_dof_position'][k,fingers('left')].sum());t=float(x['time_s'][k]);observations=json.loads((source/'rgb-observations.json').read_text());ob=max([v for v in observations if v.get('part')==part and v['mode']=='held'and v['result'].get('pose')and v['captured_wall_s']<=t],key=lambda v:v['captured_wall_s']);pose=ob['result']['pose'];G=np.linalg.inv(ob['hand_TCP_world'])@frame(pose['center_m'],axis_frame(pose['axis']))
    checker=c['checkers']['right'];other=c['checkers']['left'];saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative);old={key:getattr(checker,key,None)for key in ['gpu_route_enabled','gpu_route_socket','gpu_route_timeout_s','gpu_rgbd_stock_points']};checker.gpu_route_enabled=True;checker.gpu_route_timeout_s=2.;checker.gpu_route_socket=Path(c['config'].get('gpu_route_socket',str(here.parents[2]/'.local/curobo-gpu2/route.sock')))
    packet=np.load(source/'rgb-perception/observation-001-initial/rgbd.npz');points=cloud(packet);rgb=packet['rgb'];bounds=np.asarray(json.loads((here/'bin-picking/asset-audit.json').read_text())['baskets'][0]['outer_bounds_world_m']);gray=rgb.mean(2);mask=np.isfinite(points).all(2)&(np.ptp(rgb.astype(float),axis=2)<45)&(gray>35)&(gray<180)&np.all((points>=bounds[0]-.003)&(points<=bounds[1]+.015),axis=2);points=points[mask];checker.gpu_rgbd_stock_points=points[::max(1,len(points)//12000)]
    result=dict(state='static_GPU_route_PhysX_probe_only',source_run=name,robot_motion_executed=False,free_body_pose_input=False,full_task_pass=False)
    try:
        checker.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
        with checker.other_arm_at(other,left,leftwidth):
            route=propose_route(checker,right,goal,width)
            result['GPU_statistics']=getattr(checker,'gpu_route_stats',{})
            if route is None:return dict(result,approved=False,reason='No bounded GPU proposal')
            result['route']=np.asarray(route).tolist()
            continuous,evidence=cubic_geometry(route,c['arm'].bounds,step_rad=.010);result['cubic_geometry']=evidence
            continuous_clear=False
            if continuous is not None:
                continuous_clear,why=checker.path(continuous,width);result['continuous_curve_check']=dict(clear=continuous_clear,reason=why)
            if continuous_clear:
                q,_,timing=retime(route,.05,c['dt'],np.full(2,width/2),np.full(2,width/2),velocity_fraction=.95,acceleration=9.5,jerk=4750.,finger_command_speed=.0475,optimize=True)
            else:
                ok,why=checker.path(route,width);result['raw_path_check']=dict(clear=ok,reason=why)
                if not ok:return dict(result,approved=False,reason='Original full PhysX rejected raw and continuous proposal')
                q,_,timing=retime_safe_polyline(route,c['dt'],np.full(2,width/2),np.full(2,width/2),scale=1.,velocity_fraction=.95,acceleration=9.5,jerk=4750.)
            # Original retimer differentiated EVERY issued float32 command.
            # Geometry is checked at joint-space resolution with endpoints and
            # exact quantized local extrema, avoiding duplicate stationary ticks.
            issued=np.vstack([right,np.asarray(q[1:],dtype=np.float32)]);samples,sampling_evidence=issued_geometry_samples(issued,step_rad=.003);old_resolution=getattr(checker,'resolution_rad',.035);checker.resolution_rad=.010
            try:ok,why=checker.path(samples,width)
            finally:checker.resolution_rad=old_resolution
            result.update(approved=bool(ok),issued_curve_check=dict(clear=ok,reason=why),continuous_cubic_used=bool(continuous_clear),issued_samples=len(issued),issued_geometry_sampling=sampling_evidence,retiming=timing,original_full_native_fixed_pair_payload_checks=True,full_FR3_numpy_command_validation=True)
    finally:
        checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
        for key,value in old.items():setattr(checker,key,value)
    result['elapsed_s']=time.perf_counter()-start;print('GPU_NATIVE_ROUTE_STATIC_PROBE',json.dumps(result),flush=True);return result
