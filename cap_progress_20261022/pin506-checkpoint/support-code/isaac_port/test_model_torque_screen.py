"""Check the path torque identity against independent RNE time differences."""
from pathlib import Path
from types import SimpleNamespace
import json,unittest
import numpy as np
import mujoco
from scipy.interpolate import CubicSpline
from model_torque_screen import ModelTorqueScreen


class TorqueIdentityTests(unittest.TestCase):
    def test_scalar_path_torque_and_rate_match_rne(self):
        root=Path(__file__).resolve().parent
        names=json.loads((root/'assets/reference-dof-names.json').read_text())
        armnames=[f'{s}_arm_fr3v2_joint{j}'for s in ['left','right']for j in range(1,8)]
        ids=[names.index(n)for n in armnames]
        times=np.linspace(0,1,301)
        full=np.zeros((len(times),len(names)))
        values=np.tile([0.,-.4,0.,-1.6,0.,1.6,0.],2)+np.sin(times[:,None]*1.7)*np.linspace(.02,.12,14)
        full[:,ids]=values
        profile=SimpleNamespace(path=CubicSpline(times,values,axis=0),full_path=CubicSpline(times,full,axis=0))
        screen=ModelTorqueScreen(profile,names,0.,1.,step=.001)
        model=screen.model;data=mujoco.MjData(model)
        qa=[int(model.joint(n).qposadr[0])for n in names]
        va=[int(model.joint(n).dofadr[0])for n in armnames]
        def rne(t):
            s=.5+.2*np.sin(2*t);sd=.4*np.cos(2*t);sdd=-.8*np.sin(2*t)
            data.qpos[qa]=profile.full_path(s);data.qvel[:]=0;data.qacc[:]=0
            data.qvel[va]=profile.path(s,1)*sd
            data.qacc[va]=profile.path(s,2)*sd**2+profile.path(s,1)*sdd
            mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data);mujoco.mj_comVel(model,data)
            torque=np.zeros(model.nv);mujoco.mj_rne(model,data,1,torque)
            return torque[va].copy()
        t=.37;s=.5+.2*np.sin(2*t);sd=.4*np.cos(2*t);sdd=-.8*np.sin(2*t);sj=-1.6*np.cos(2*t)
        torque,rate=screen.torque_and_rate(s,sd,sdd,sj)
        np.testing.assert_allclose(torque,rne(t),atol=1e-6,rtol=1e-5)
        h=1e-5
        np.testing.assert_allclose(rate,(rne(t+h)-rne(t-h))/(2*h),atol=1e-4,rtol=1e-3)


if __name__=='__main__':unittest.main()
