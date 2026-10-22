"""Geometry samples for a bounded GPU cubic; no commands or collision waiver."""
import numpy as np
from scipy.interpolate import CubicSpline

def cubic_geometry(route,bounds,step_rad=.010):
    points=np.asarray(route,float);bounds=np.asarray(bounds,float)
    if points.ndim!=2 or points.shape[1]!=7 or len(points)<2 or not np.isfinite(points).all():raise ValueError('Invalid finite GPU joint route')
    knots=np.linspace(0,1,len(points));spline=CubicSpline(knots,points);roots=spline.derivative().roots(extrapolate=False);extra=np.concatenate([np.asarray(v,float).ravel()for v in roots]);u=np.unique(np.r_[knots,extra[np.isfinite(extra)&(extra>=0)&(extra<=1)]])
    # Include every true joint extremum before subdivision; equal interval ends
    # cannot hide an internal reversal. This is the same geometric cubic retime uses.
    for _ in range(24):
        q=spline(u)
        if np.any(q<bounds[:,0])or np.any(q>bounds[:,1]):return None,dict(reason='Cubic joint extremum outside original bounds')
        bad=np.flatnonzero(np.max(abs(np.diff(q,axis=0)),axis=1)>step_rad)
        if not len(bad):return q,dict(cubic_samples=len(q),joint_extrema_included=True,maximum_sample_joint_step_rad=float(np.max(abs(np.diff(q,axis=0)))))
        u=np.unique(np.r_[u,(u[bad]+u[bad+1])/2])
        if len(u)>16384:break
    return None,dict(reason='Cubic subdivision budget exceeded')

def issued_geometry_samples(curve,step_rad=.003):
    q=np.asarray(curve,float)
    if q.ndim!=2 or q.shape[1]!=7 or not np.isfinite(q).all()or len(q)<2:raise ValueError('Invalid issued float32 curve')
    keep={0,len(q)-1}
    for j in range(7):
        d=np.diff(q[:,j]);nz=np.flatnonzero(d!=0)
        turns=np.flatnonzero(np.sign(d[nz[1:]])!=np.sign(d[nz[:-1]]))
        for k in turns:keep.update([int(nz[k]+1),int(nz[k+1])])
    last=0
    for k in range(1,len(q)):
        if k in keep or np.max(abs(q[k]-q[last]))>=step_rad:keep.add(k);last=k
    indices=np.array(sorted(keep),int)
    return q[indices],dict(full_issued_samples=len(q),issued_geometry_samples=len(indices),first_last_and_float32_extrema_preserved=True,maximum_retained_joint_step_rad=float(np.max(abs(np.diff(q[indices],axis=0)))))
