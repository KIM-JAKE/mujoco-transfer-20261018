"""Opt-in bounded PIN prealignment from fresh RGB-D and actual SDK frames.

Canonical CAD-body -> physical symmetry-axis origin is model data, not a
world/object pose. The field detector fits the CAD body; it is NOT the SIM308
end-circle detector. Hardware precision/convergence is unverified.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from .calibration import rigid

BODY_TO_AXIS_M=np.array([0.,-.00025,0.])
AXIS_OFFSET=np.eye(4);AXIS_OFFSET[:3,3]=BODY_TO_AXIS_M
SHAFT_AXIS_ENDPOINTS_M=(-.0035,.0135)


def options(value):
    if value is None:return None
    if not isinstance(value,dict):raise ValueError('Explicit pin precision options required')
    if value.get('enabled')is False:return None
    if value.get('enabled')is not True:raise ValueError('Explicit pin precision enabled flag required')
    gain=value.get('gain',.5);iterations=value.get('max_iterations',8)
    if type(gain)not in(int,float)or not .3<=gain<=1 or type(iterations)is not int or not 1<=iterations<=8:raise ValueError('Pin feedback gain0.3--1 and bounded1--8 iterations required')
    return dict(gain=float(gain),max_iterations=iterations)


def rotation_error(A,B):
    return float(np.linalg.norm(Rotation.from_matrix(np.asarray(A)@np.asarray(B).T).as_rotvec()))


def errors(observed_body,goal_body):
    measured=rigid(observed_body)@AXIS_OFFSET;goal=rigid(goal_body)@AXIS_OFFSET
    axis=goal[:3,2];observed_axis=measured[:3,2];delta=measured[:3,3]-goal[:3,3]
    radial=float(np.linalg.norm(delta-axis*(delta@axis)))
    angle=float(np.arccos(np.clip(observed_axis@axis,-1,1)))
    shaft=max(float(np.linalg.norm((delta+z*(observed_axis-axis))-axis*((delta+z*(observed_axis-axis))@axis)))for z in SHAFT_AXIS_ENDPOINTS_M)
    remaining=float(-delta@axis)
    result=dict(radial_error_m=radial,axis_error_rad=angle,shaft_axis_max_radial_error_m=shaft,remaining_depth_m=remaining,radial_limit_m=.000075,axis_limit_rad=.003,shaft_limit_m=.00015,physical_axis_pose_origin='canonical pin CAD body + [0,-0.25mm,0]',uses_free_body_poses=False)
    if not .035<remaining<.055:raise RuntimeError('Pin precision must remain35--55mm below calibrated axis goal')
    result['aligned']=bool(radial<=.000075 and angle<=.003 and shaft<=.00015)
    return result


def correction(command_tcp,actual_tcp,G_body,observed_body,goal_body,*,gain=.5):
    C,A,G=map(rigid,(command_tcp,actual_tcp,G_body));goal=rigid(goal_body)@AXIS_OFFSET;observed=rigid(observed_body)@AXIS_OFFSET
    evidence=errors(observed_body,goal_body)
    predicted=A@G@AXIS_OFFSET
    if np.linalg.norm(predicted[:3,3]-observed[:3,3])>.00005 or rotation_error(predicted[:3,:3],observed[:3,:3])>.0005:raise RuntimeError('Fresh image/actual SDK grasp association changed before pin correction')
    bias=float(np.linalg.norm(C[:3,3]-A[:3,3]));bias_angle=rotation_error(C[:3,:3],A[:3,:3])
    if bias>.002 or bias_angle>.01:raise RuntimeError('Pin actual-vs-command SDK TCP bias exceeds2mm/.01rad')
    axis=observed[:3,2];desired_axis=goal[:3,2];cross=np.cross(axis,desired_axis);n=float(np.linalg.norm(cross))
    if axis@desired_axis<0:raise RuntimeError('Pin signed axis is inconsistent with calibrated insertion direction')
    delta_R=Rotation.from_rotvec(cross/max(n,1e-12)*np.arctan2(n,axis@desired_axis)).as_matrix()
    R=delta_R@A[:3,:3];G_axis=G@AXIS_OFFSET
    point=goal[:3,3]-desired_axis*max(.045,evidence['remaining_depth_m'])
    desired=np.eye(4);desired[:3,:3]=R;desired[:3,3]=point-R@G_axis[:3,3]
    target=C@np.linalg.inv(A)@desired
    if not .3<=gain<=1:raise ValueError('Bounded geometric feedback gain required')
    target[:3,3]=C[:3,3]+gain*(target[:3,3]-C[:3,3])
    target[:3,:3]=Rotation.from_rotvec(gain*Rotation.from_matrix(target[:3,:3]@C[:3,:3].T).as_rotvec()).as_matrix()@C[:3,:3]
    translation=float(np.linalg.norm(target[:3,3]-C[:3,3]));angle=rotation_error(target[:3,:3],C[:3,:3])
    if translation>.008 or angle>.1:raise RuntimeError('Pin prealignment correction exceeds8mm/.1rad local bound')
    evidence.update(command_vs_actual_translation_m=bias,command_vs_actual_rotation_rad=bias_angle,geometric_feedback_gain=gain,command_step_translation_m=translation,command_step_rotation_rad=angle)
    return target,evidence


def axial_target(command_tcp,observed_body,goal_body):
    evidence=errors(observed_body,goal_body)
    if not evidence['aligned']:raise RuntimeError('Pin precision center/axis gate blocks axial insertion')
    target=rigid(command_tcp).copy();target[:3,3]+=rigid(goal_body)[:3,2]*evidence['remaining_depth_m']
    return target,evidence


def sdk_sample(value,tracking_limit):
    from .stopped_arm_monitor import validate
    validate(value,value['q_d'],tracking_limit)
    actual=rigid(np.asarray(value['O_T_EE_column_major']).reshape(4,4,order='F'))
    command=rigid(np.asarray(value['O_T_EE_desired_model_column_major']).reshape(4,4,order='F'))
    if np.linalg.norm(actual[:3,3]-command[:3,3])>.002 or rotation_error(actual[:3,:3],command[:3,:3])>.01:raise RuntimeError('Pin precision SDK TCP tracking bias exceeds local bound')
    return actual,command


def encoder_window(samples,tracking_limit):
    if len(samples)<5:return None
    times=np.asarray([s['robot_time_s']for s in samples],float)
    if not np.isfinite(times).all()or np.any(np.diff(times)<=0):raise RuntimeError('Pin precision actual SDK clock must advance')
    poses=[];reference=np.asarray(samples[0]['q_d']);F=samples[0]['F_T_EE_column_major']
    for state in samples:
        A,_=sdk_sample(state,tracking_limit);poses.append(A)
        if np.max(abs(np.asarray(state['q_d'])-reference))>1e-6 or not np.allclose(state['F_T_EE_column_major'],F,atol=1e-12,rtol=0):raise RuntimeError('Pin stopped desired joints/TCP changed during precision window')
    if times[-1]-times[0]<.1:return None
    translations=max(float(np.linalg.norm(t[:3,3]-poses[0][:3,3]))for t in poses)
    angles=max(rotation_error(t[:3,:3],poses[0][:3,:3])for t in poses)
    if translations>.000025 or angles>.0005:return None
    return dict(samples=len(samples),actual_sdk_window_s=float(times[-1]-times[0]),translation_span_m=translations,rotation_span_rad=angles,position_span_limit_m=.000025,rotation_span_limit_rad=.0005,source='actual SDK O_T_EE/robot_time and frozen q_d',unsampled_physical_motion_certified=False)
