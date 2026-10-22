"""Imported-model gravity-excluded torque/rate screen for scalar path retiming.

For q=q(s), tau_dyn=A(s)*sddot+B(s)*sdot**2. Its time derivative
is A*sjerk+(A'+2B)*sdot*sddot+B'*sdot**3. Coefficients include the
imported arm/gripper but exclude contact, carried payload and motor friction.
This model screen cannot certify the physical robot.
"""
import numpy as np
import mujoco
from scipy.interpolate import CubicSpline
from source_model import load_model


class ModelTorqueScreen:
    def __init__(self, profile, names, start, end, step=.002, rate_limit=900.):
        self.rate_limit=float(rate_limit)
        if not 0<self.rate_limit<=1000.:
            raise ValueError('Model rate limit must be in (0,1000] Nm/s')
        self.model=load_model();self.model.opt.gravity[:]=0
        self.model.body_gravcomp[:]=0
        data=mujoco.MjData(self.model)
        qa=np.array([int(self.model.joint(n).qposadr[0])for n in names])
        arms=[f'{s}_arm_fr3v2_joint{j}'for s in ['left','right']for j in range(1,8)]
        va=np.array([int(self.model.joint(n).dofadr[0])for n in arms])
        times=np.linspace(start,end,max(101,int((end-start)/step)+1))
        linear=[];quadratic=[];torque=np.zeros(self.model.nv)
        for s in times:
            data.qpos[qa]=profile.full_path(s)
            data.qvel[:]=0.;data.qacc[:]=0.
            mujoco.mj_kinematics(self.model,data)
            mujoco.mj_comPos(self.model,data)
            data.qacc[va]=profile.path(s,1)
            mujoco.mj_comVel(self.model,data)
            mujoco.mj_rne(self.model,data,1,torque)
            linear.append(torque[va].copy())
            data.qvel[va]=profile.path(s,1)
            data.qacc[va]=profile.path(s,2)
            mujoco.mj_comVel(self.model,data)
            mujoco.mj_rne(self.model,data,1,torque)
            quadratic.append(torque[va].copy())
        self.a=CubicSpline(times,np.array(linear),axis=0)
        self.b=CubicSpline(times,np.array(quadratic),axis=0)

    def torque_and_rate(self,s,sd,sdd,sjerk):
        sd,sdd,sjerk=[np.asarray(x)[...,None]for x in [sd,sdd,sjerk]]
        a,b=self.a(s),self.b(s)
        torque=a*sdd+b*sd**2
        rate=a*sjerk+(self.a(s,1)+2*b)*sd*sdd+self.b(s,1)*sd**3
        return torque,rate

    def ratios(self,s,sd,sdd,sjerk):
        _,rate=self.torque_and_rate(s,sd,sdd,sjerk)
        return np.cbrt(np.max(abs(rate),axis=-1)/self.rate_limit)
