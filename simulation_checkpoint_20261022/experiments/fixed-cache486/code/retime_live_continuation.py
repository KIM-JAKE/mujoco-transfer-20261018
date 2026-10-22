"""Locally slow changed robot targets until finite-difference limits pass.

Pure command scheduling: no body poses, collision settings, or limits change.
"""
import copy,numpy as np
from isaac_motion import Audit
from fr3_motion_control import position_velocity_bounds

def prepare(cmd,control,left_clock,right_clock,audit,indices,dt=1/480):
    q=np.array(cmd,np.float32);c=np.array(control,float);lc=np.array(left_clock);rc=np.array(right_clock);history=[]
    fi=[i for i in range(q.shape[1])if i not in audit.indices]
    def metrics(q):
        aidx=audit.indices;group=q[1:,aidx].astype(float);v=np.diff(q[:,aidx].astype(float),axis=0)/dt;a=np.diff(np.vstack([audit.v,v]),axis=0)/dt;j=np.diff(np.vstack([audit.a,a]),axis=0)/dt
        lo,hi=position_velocity_bounds(group.reshape(-1,2,7));cap=.95*np.where(v.reshape(-1,2,7)>=0,hi,-lo).reshape(-1,14)
        rv=(abs(v)/np.maximum(cap,1e-8)).max(1);ra=abs(a).max(1);rj=abs(j).max(1);rf=abs(np.diff(q[:,fi].astype(float),axis=0)/dt).max(1)
        scale=np.maximum.reduce([rv/.985,np.sqrt(ra/9.2),np.cbrt(rj/4500.),rf/.0465]);return scale,dict(velocity_fraction=float(rv.max()),acceleration=float(ra.max()),jerk=float(rj.max()),finger_speed=float(rf.max()))
    for iteration in range(35):
        scale,peaks=metrics(q);history.append(dict(iteration=iteration,duration_s=(len(q)-1)*dt,**peaks))
        if scale.max()<=1.0001:
            check=copy.deepcopy(audit)
            for row in q[1:]:check.update(row[None,:])
            return q,c,lc,rc,dict(history=history,added_s=(len(q)-len(cmd))*dt,command_audit=check.report())
        slowdown=np.r_[1.,np.maximum(1.,scale*1.025)];width=25;pad=np.pad(slowdown,(width,width),mode='edge');slowdown=np.maximum.reduce([pad[j:j+len(slowdown)]for j in range(2*width+1)])
        radius=36;x=np.arange(-radius,radius+1);kernel=np.exp(-.5*(x/12.)**2);kernel/=kernel.sum();slowdown=np.convolve(np.pad(slowdown,(radius,radius),mode='edge'),kernel,mode='valid')
        # The caller is already executing; preserve the initial derivative.
        u=np.clip(np.arange(len(q))/36.,0,1);slowdown=1+(slowdown-1)*(u**3*(10-15*u+6*u*u))
        old=np.arange(len(q))*dt;warped=np.r_[0,np.cumsum((slowdown[:-1]+slowdown[1:])*.5*dt)];ticks=np.arange(int(np.ceil(warped[-1]/dt))+1)*dt;clock=np.interp(ticks,warped,old)
        q=np.column_stack([np.interp(clock,old,q[:,j])for j in range(q.shape[1])]).astype(np.float32);c=np.column_stack([np.interp(clock,old,c[:,j])for j in range(c.shape[1])]);lc=np.interp(clock,old,lc);rc=np.interp(clock,old,rc)
        if ticks[-1]>(len(cmd)-1)*dt+6:raise RuntimeError('Live continuation requires more than 6 seconds extra: '+str(history))
    raise RuntimeError('Live continuation did not converge: '+str(history))
