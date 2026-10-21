"""Retiming of an unchanged joint path under the existing FR3 command limits.

Compare a minimum-jerk progress law with bounded-jerk ramps and a cruise
section. Every candidate is checked at the actual command rate, including
position-dependent velocity bounds and float32 command quantization.
"""
import numpy as np
from scipy.interpolate import CubicSpline
from fr3_motion_control import position_velocity_bounds, LOWER, UPPER


def progress(u, ramp=None):
    if ramp is None:
        return u**3 * (10 - 15*u + 6*u*u)
    r=float(ramp);j=.02;acceleration=1/((r-j)*(1-r));jerk=acceleration/j
    durations=[j,r-2*j,j,1-2*r,j,r-2*j,j]
    jerks=[jerk,0.,-jerk,0.,-jerk,0.,jerk]
    h=np.empty_like(u);start=position=velocity=acc=0.
    for duration,value in zip(durations,jerks):
        if duration<=0:continue
        mask=(u>=start)&(u<=start+duration+1e-12);t=u[mask]-start
        h[mask]=position+velocity*t+acc*t*t/2+value*t*t*t/6
        position+=velocity*duration+acc*duration**2/2+value*duration**3/6
        velocity+=acc*duration+value*duration**2/2;acc+=value*duration;start+=duration
    h[0]=0.;h[-1]=1.
    return h


def retime(qs, requested_s, dt, finger_start, finger_target,
           velocity_fraction=.90, acceleration=9.2, jerk=4400.,
           finger_command_speed=.043, optimize=True):
    spline=CubicSpline(np.linspace(0,1,len(qs)),np.asarray(qs))
    preview=spline(np.linspace(0,1,513))
    if np.any(preview<LOWER+.001) or np.any(preview>UPPER-.001):
        raise RuntimeError('Joint path leaves FR3 position limits; retiming cannot repair geometry')
    finger_distance=float(np.max(abs(np.asarray(finger_target)-finger_start)))
    best=None
    for ramp in ([None,.30,.40,.50] if optimize else [None]):
        peak=1.875 if ramp is None else 1/(1-ramp)
        duration=max(float(requested_s),finger_distance/finger_command_speed*peak,2*dt)
        valid=False
        for _ in range(24):
            if duration>60:break
            count=int(np.ceil(duration/dt));u=np.linspace(0,1,count+1);h=progress(u,ramp)
            q=spline(h).astype(np.float32).astype(float)
            v=np.diff(q,axis=0,prepend=q[:1])/dt
            a=np.diff(v,axis=0,prepend=np.zeros((1,7)))/dt
            j=np.diff(a,axis=0,prepend=np.zeros((1,7)))/dt
            lo,hi=position_velocity_bounds(q)
            finger_peak=finger_distance*float(np.max(np.diff(h)))/dt
            stress=max(float(np.max(abs(v)/np.maximum(velocity_fraction*np.where(v>=0,hi,-lo),1e-9))),
                       float(np.sqrt(abs(a).max()/acceleration)),
                       float(np.cbrt(abs(j).max()/jerk)),finger_peak/finger_command_speed)
            if stress<=.98:valid=True;break
            duration*=max(1.025,stress*1.015)
        if not valid:continue
        if best is None or count<best[0]:best=(count,q,h,ramp)
    if best is None:raise RuntimeError('No FR3-limited timing for the requested joint path')
    count,q,h,ramp=best
    return q,h,dict(duration_s=count*dt,progress_law='minimum_jerk' if ramp is None else 'bounded_jerk_with_cruise',ramp_fraction=ramp,
                   velocity_fraction=velocity_fraction,acceleration_rad_s2=acceleration,jerk_rad_s3=jerk)


def retime_checked_polyline(qs,dt,finger_start,baseline,velocity_fraction=.91,acceleration=9.2,jerk=4000.):
    """Alternative only for a collision-checked polyline and CONSTANT fingers.

    Stop at true corners instead of slowing the whole smooth curve for its
    sharpest bend. Caller must validate the polyline in the live PhysX scene.
    Full concatenated float32 commands are independently differentiated here.
    """
    qs=np.asarray(qs,float);points=[qs[0]]
    for i in range(1,len(qs)-1):
        a=qs[i]-points[-1];b=qs[i+1]-qs[i];na=np.linalg.norm(a);nb=np.linalg.norm(b)
        if na<1e-12:continue
        if nb>1e-12 and np.linalg.norm(a/na-b/nb)<1e-10:continue
        points.append(qs[i])
    points.append(qs[-1]);pieces=[np.asarray(points[:1])]
    for index,(a,b)in enumerate(zip(points[:-1],points[1:])):
        q,_,_=retime([a,b],.05,dt,finger_start,finger_start,velocity_fraction=velocity_fraction,acceleration=acceleration,jerk=jerk,optimize=True)
        pieces.append(q[1:])
        if index<len(points)-2:pieces.append(np.repeat(q[-1:],2,axis=0))
    q=np.concatenate(pieces)
    if len(q)>=len(baseline[0]):return baseline
    v=np.diff(q,axis=0,prepend=q[:1])/dt;a=np.diff(v,axis=0,prepend=np.zeros((1,7)))/dt;j=np.diff(a,axis=0,prepend=np.zeros((1,7)))/dt
    lo,hi=position_velocity_bounds(q)
    stress=max(float(np.max(abs(v)/np.maximum(velocity_fraction*np.where(v>=0,hi,-lo),1e-9))),float(np.sqrt(abs(a).max()/acceleration)),float(np.cbrt(abs(j).max()/jerk)))
    if stress>.98 or np.any(q<LOWER+.001) or np.any(q>UPPER-.001):return baseline
    timing=dict(baseline[2],duration_s=(len(q)-1)*dt,progress_law='PhysX_checked_polyline_stops',corner_stops=max(0,len(points)-2),full_concatenated_command_check=True)
    return q,np.linspace(0,1,len(q)),timing


def retime_safe_polyline(qs,dt,finger_start,finger_target,scale=1.,
                         velocity_fraction=.90,acceleration=9.2,jerk=4400.):
    """Preserve checked joint edges when a cubic would overshoot a limit.

    Every true corner stops. No clipping, skipped edge or joint-limit change.
    The full issued float32 sequence, including joins, is checked again.
    """
    qs=np.asarray(qs,float);points=[qs[0]]
    for q in qs[1:]:
        if np.max(abs(q-points[-1]))>1e-8:points.append(q)
    if len(points)==1:points.append(points[0])
    compact=[points[0]]
    for i in range(1,len(points)-1):
        a=points[i]-compact[-1];b=points[i+1]-points[i];na=np.linalg.norm(a);nb=np.linalg.norm(b)
        if na>1e-12 and nb>1e-12 and np.linalg.norm(a/na-b/nb)<1e-9:continue
        compact.append(points[i])
    compact.append(points[-1]);points=compact
    scale=max(1.,float(scale));finger_start=np.asarray(finger_start);finger_target=np.asarray(finger_target)
    for attempt in range(8):
        pieces=[np.asarray(points[:1])]
        for index,(a,b)in enumerate(zip(points[:-1],points[1:])):
            segment,_,timing=retime([a,b],.05,dt,finger_start,finger_start,velocity_fraction=velocity_fraction,acceleration=acceleration,jerk=jerk,optimize=True)
            if scale>1. or attempt:
                segment,_,_=retime([a,b],timing['duration_s']*scale*(1.04**attempt),dt,finger_start,finger_start,velocity_fraction=velocity_fraction,acceleration=acceleration,jerk=jerk,optimize=False)
            pieces.append(segment[1:])
            if index<len(points)-2:pieces.append(np.repeat(segment[-1:],3,axis=0))
        q=np.concatenate(pieces);h=progress(np.linspace(0,1,len(q)))
        v=np.diff(q,axis=0,prepend=q[:1])/dt;a=np.diff(v,axis=0,prepend=np.zeros((1,7)))/dt;j=np.diff(a,axis=0,prepend=np.zeros((1,7)))/dt;lo,hi=position_velocity_bounds(q)
        peak_v=float(np.max(abs(v)/np.maximum(velocity_fraction*np.where(v>=0,hi,-lo),1e-9)));peak_a=float(abs(a).max());peak_j=float(abs(j).max());finger_speed=float(np.max(abs(finger_target-finger_start)))*float(np.diff(h).max())/dt
        if peak_v<=.98 and peak_a<=acceleration and peak_j<=jerk and finger_speed<=.043:
            return q,h,dict(duration_s=(len(q)-1)*dt,progress_law='checked_joint_edges_with_corner_stops',corner_stops=max(0,len(points)-2),velocity_fraction=velocity_fraction,acceleration_rad_s2=acceleration,jerk_rad_s3=jerk,geometry_overshoot_avoided=True,full_concatenated_command_check=True,peak_velocity_fraction_of_reserved_bound=peak_v,max_acceleration_rad_s2=peak_a,max_jerk_rad_s3=peak_j)
    raise RuntimeError('No FR3-limited timing for checked polyline joins')
