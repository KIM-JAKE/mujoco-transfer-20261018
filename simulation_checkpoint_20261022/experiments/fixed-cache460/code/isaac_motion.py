"""95% FR3 position-dependent command limits, evaluated inside Isaac Sim."""
import numpy as np
from fr3_motion_control import position_velocity_bounds

def duration(path,requested):
    path=np.asarray(path,float);u=np.linspace(0,1,2001);h=u**3*(10-15*u+6*u*u);x=h*(len(path)-1);j=np.minimum(len(path)-2,x.astype(int));w=x-j;q=path[j]*(1-w[:,None])+path[j+1]*w[:,None]
    v=np.gradient(q,u,axis=0);a=np.gradient(v,u,axis=0);jerk=np.gradient(a,u,axis=0);lo,hi=position_velocity_bounds(q);cap=.95*np.where(v>=0,hi,-lo)
    if np.any((cap<1e-7)&(abs(v)>1e-5)):raise RuntimeError('Trajectory enters FR3 zero-speed joint-limit band; choose another IK solution')
    return max(float(requested),float(np.max(abs(v)/np.maximum(cap,1e-8)))*1.2,float(np.sqrt(np.max(abs(a))/9.5))*1.2,float(np.cbrt(np.max(abs(jerk))/4750))*1.2)

class Audit:
    def __init__(self,cmd,indices,dt):
        self.indices=np.concatenate(list(indices.values()));self.dt=dt;self.prev=cmd[0,self.indices].astype(float).copy();self.v=np.zeros(14);self.a=np.zeros(14);self.peak_ratio=0.;self.peak_a=0.;self.peak_j=0.;self.samples=0
    def update(self,cmd):
        q=cmd[0,self.indices].astype(float);v=(q-self.prev)/self.dt;a=(v-self.v)/self.dt;j=(a-self.a)/self.dt
        lo,hi=position_velocity_bounds(q.reshape(2,7));cap=.95*np.where(v.reshape(2,7)>=0,hi,-lo);ratio=float(np.max(abs(v.reshape(2,7))/np.maximum(cap,1e-8)));self.peak_ratio=max(self.peak_ratio,ratio);self.peak_a=max(self.peak_a,float(np.max(abs(a))));self.peak_j=max(self.peak_j,float(np.max(abs(j))));self.samples+=1
        self.prev=q.copy();self.v=v;self.a=a
        if ratio>1.002 or np.max(abs(a))>9.7 or np.max(abs(j))>4800:raise RuntimeError('FR3 command limit violation: velocity fraction='+str(ratio)+' acceleration='+str(float(np.max(abs(a))))+' jerk='+str(float(np.max(abs(j)))))
    def report(self):return dict(samples=self.samples,velocity_fraction_of_95_percent_PBV=self.peak_ratio,max_acceleration_rad_s2=self.peak_a,max_jerk_rad_s3=self.peak_j,acceleration_limit_rad_s2=9.5,jerk_limit_rad_s3=4750,command_limit_checks=True)
