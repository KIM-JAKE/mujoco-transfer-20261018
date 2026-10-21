"""Pinned CAD envelopes transformed by actual per-link SDK FK, never TCP inference."""
import numpy as np
from . import collision


def frames(value,points,tcp,F_T_EE):
    if value is None:raise ValueError('Actual native SDK base/joint1..7 frames required')
    f=collision.finite(value)
    if f.ndim!=4 or f.shape[1:]!=(8,4,4):raise ValueError('Actual native SDK base/joint1..7 frames required for model-derived link geometry')
    rotations=f[:,:,:3,:3]
    if (not np.allclose(f[:,:,3],[0,0,0,1],atol=1e-9)or not np.allclose(np.swapaxes(rotations,-1,-2)@rotations,np.eye(3),atol=1e-7)or np.any(np.abs(np.linalg.det(rotations)-1)>1e-7)):
        raise ValueError('Proper actual SDK link rotations required')
    if not np.allclose(f[:,:,:3,3],points[:,:8],atol=1e-8,rtol=0):raise ValueError('SDK per-link frames differ from the checked C2 link-point samples')
    if not np.allclose(f[:,0],np.eye(4),atol=1e-9):raise ValueError('SDK base link frame must be identity')
    flange=tcp@np.linalg.inv(collision.transform(F_T_EE))
    joint7_to_flange=np.linalg.inv(f[:,7])@flange
    expected=np.eye(4);expected[2,3]=.107
    if not np.allclose(joint7_to_flange,expected,atol=1e-5,rtol=0):raise ValueError('Actual SDK joint7/flange convention does not match pinned FR3-v2 CAD')
    return f


def robot_capsules(link_frames,tcp_poses,envelope,point_speeds,joint_velocities,gap,tracking_bounds,tracking_joint_error,prefix):
    rows=[];local=envelope['link_capsules_link_m'];v=np.asarray(joint_velocities);error=np.asarray(tracking_joint_error)
    if v.shape!=(7,)or error.shape!=(7,)or np.any(v<0)or np.any(error<0):raise ValueError('Actual bounded joint speed/error required')
    for spec in local:
        i=spec['link_index'];T=link_frames[:,i]
        if type(i)is not int or not 0<=i<=7:raise ValueError('Pinned CAD link index0..7 required')
        reach=max(np.linalg.norm(spec['a']),np.linalg.norm(spec['b']))+float(spec['radius_m'])
        speed=float(point_speeds[i]+np.sum(v[:i])*reach)
        track=float(tracking_bounds[i]+np.sum(error[:i])*reach)
        row=collision.local_capsules([spec],T,point_speeds[i],np.sum(v[:i]),gap,prefix)[0]
        # Include local radius in rotational sweep bound, not merely centerline endpoints.
        row['r']=float(spec['radius_m'])+speed*gap/2+track
        row.update(name=prefix+spec['name'],geometry_role='arm',link_index=i,speed_bound_m_s=speed)
        rows.append(row)
    tool_rows=envelope['tool_capsules_tcp_m']
    if envelope.get('_aperture_interval_m')is not None:
        from .hand_aperture import tools
        tool_rows=tools(envelope,envelope['_aperture_interval_m'])
    for spec in tool_rows:
        reach=max(np.linalg.norm(spec['a']),np.linalg.norm(spec['b']))+float(spec['radius_m'])
        speed=float(point_speeds[-1]+np.sum(v)*reach);track=float(tracking_bounds[-1]+np.sum(error)*reach)
        row=collision.local_capsules([spec],tcp_poses,point_speeds[-1],np.sum(v),gap,prefix)[0]
        row['r']=float(spec['radius_m'])+speed*gap/2+track
        row.update(name=spec['name'],geometry_role=spec.get('geometry_role','tool'),link_index=8,speed_bound_m_s=speed,hand_body='left_finger'if spec['name'].startswith('left_finger')else'right_finger'if spec['name'].startswith('right_finger')else'palm')
        rows.append(row)
    rows[0]['actual_tcp']=tcp_poses[:,:3,3]
    return rows


def self_connected(a,b):
    """Only same rigid bodies / immediate mechanical neighbors are attached."""
    i,j=a.get('link_index'),b.get('link_index')
    if i is None or j is None:return False
    if i==j:return True
    return abs(i-j)==1
