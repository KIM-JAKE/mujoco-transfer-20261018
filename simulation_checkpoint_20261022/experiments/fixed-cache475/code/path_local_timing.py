"""Local speed allocation along an unchanged, already checked joint spline.

Linear path acceleration constraints set a forward speed envelope. A smoothed
monotone time law is then independently differentiated at the actual control
rate, with float32 commands, and slowed until ALL existing bounds pass.
"""
import warnings
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import linprog, OptimizeWarning
from scipy.sparse import coo_matrix
from fr3_motion_control import position_velocity_bounds, LOWER, UPPER


def retime_local(qs,requested,dt,finger_start,finger_target,vfrac,acceleration,jerk,finger_speed,baseline):
    if len(qs)<3:return baseline
    curve=CubicSpline(np.linspace(0,1,len(qs)),qs);nodes=129;u=np.linspace(0,1,nodes);du=u[1];q=curve(u);dq=curve(u,1);lo,hi=position_velocity_bounds(q)
    cap=vfrac*np.where(dq>=0,hi,-lo);speed=np.min(np.where(abs(dq)>1e-10,cap/np.maximum(abs(dq),1e-10),1e5),axis=1)
    finger_distance=float(np.max(abs(np.asarray(finger_target)-finger_start)))
    if finger_distance>1e-12:speed=np.minimum(speed,finger_speed/finger_distance)
    mid=(u[1:]+u[:-1])/2;p=curve(mid,1);pp=curve(mid,2);left=.5*pp-p/(2*du);right=.5*pp+p/(2*du)
    rows=[];cols=[];values=[];limits=[]
    for i in range(nodes-1):
        for j in range(7):
            for sign in [-1,1]:
                k=len(limits);rows.extend([k,k]);cols.extend([i,i+1]);values.extend([sign*left[i,j],sign*right[i,j]]);limits.append(acceleration*.80)
    matrix=coo_matrix((values,(rows,cols)),shape=(len(limits),nodes)).tocsr();bounds=[(0.,float(max(0.,v*v)))for v in speed];bounds[0]=bounds[-1]=(0.,0.)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore',OptimizeWarning)
        result=linprog(-np.ones(nodes),A_ub=matrix,b_ub=np.array(limits),bounds=bounds,method='highs-ds',options={'threads':1,'time_limit':.15})
    if not result.success:return baseline
    path_speed=np.sqrt(np.maximum(0.,result.x));segment_time=2*du/np.maximum(path_speed[1:]+path_speed[:-1],1e-12);times=np.r_[0.,np.cumsum(segment_time)];base=float(times[-1])
    if not np.isfinite(base) or base>baseline[2]['duration_s']*1.5:return baseline
    scale=max(1.,float(requested)/base)
    for attempt in range(8):
        count=int(np.ceil(base*scale/dt));tt=np.arange(count+1)*dt/scale;tt=np.minimum(tt,base);ix=np.minimum(np.searchsorted(times,tt,side='right')-1,nodes-2);ix=np.maximum(ix,0);local=tt-times[ix]
        ap=(path_speed[ix+1]**2-path_speed[ix]**2)/(2*du);s=u[ix]+path_speed[ix]*local+.5*ap*local**2;s=np.maximum.accumulate(np.clip(s,0,1));s[0]=0.;s[-1]=1.
        # Convolving progress increments changes only timing, never the path.
        # Zero padding provides smooth starts/stops and preserves integral 1.
        sigma=max(2.,.010/dt);radius=int(np.ceil(4*sigma));kernel=np.exp(-.5*(np.arange(-radius,radius+1)/sigma)**2);kernel/=kernel.sum();increments=np.convolve(np.diff(s),kernel,mode='full');h=np.r_[0.,np.cumsum(increments)];h/=h[-1];h[-1]=1.
        issued=curve(h).astype(np.float32).astype(float);v=np.diff(issued,axis=0,prepend=issued[:1])/dt;a=np.diff(v,axis=0,prepend=np.zeros((1,7)))/dt;j=np.diff(a,axis=0,prepend=np.zeros((1,7)))/dt;vlo,vhi=position_velocity_bounds(issued)
        stress=max(float(np.max(abs(v)/np.maximum(vfrac*np.where(v>=0,vhi,-vlo),1e-9))),float(np.sqrt(abs(a).max()/acceleration)),float(np.cbrt(abs(j).max()/jerk)),finger_distance*float(np.diff(h).max())/dt/finger_speed)
        if stress<=.98 and np.all(issued>=LOWER+.001) and np.all(issued<=UPPER-.001):
            if len(issued)>=len(baseline[0]):return baseline
            return issued,h,dict(duration_s=(len(issued)-1)*dt,progress_law='locally_constrained_path_speed_with_smoothed_progress',velocity_fraction=vfrac,acceleration_rad_s2=acceleration,jerk_rad_s3=jerk,unchanged_geometric_spline=True,full_float32_issued_field_check=True,saved_s=(len(baseline[0])-len(issued))*dt)
        scale*=max(1.04,stress*1.025)
    return baseline
