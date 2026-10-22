"""Retarget the real collision-pad edge, replacing the old virtual fingertip."""
import hashlib
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

class ButtonControl:
 def __init__(self,parent):
  self.parent=parent;self.model=parent.model;self.ref=mujoco.MjData(self.model)
  self.body=self.model.body('right_grip_right_pad').id;self.hand=self.model.body('right_grip_base').id
  geom=self.model.geom('right_grip_right_pad1');self.local=geom.pos+np.array([0,geom.size[1],geom.size[2]])
  self.ref.qpos[:]=parent.source['qpos'][-1];mujoco.mj_kinematics(self.model,self.ref)
  end=self.ref.xpos[self.body]+self.ref.xmat[self.body].reshape(3,3)@self.local
  self.target=np.array([.3559690421575,-.24925+(.00195 if getattr(parent,'fast_cycle',False)else .0021),1.527]);self.delta=self.target-end
  self.sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();self.file=str(Path(__file__).resolve())
  print('PHYSICAL_BUTTON_PAD_RETARGET',self.local.tolist(),self.delta.tolist(),self.sha256,flush=True)
 def update(self,t,data,indices,offset):
  p=self.parent;self.ref.qpos[:]=[np.interp(t,p.ts,p.source['qpos'][:,i])for i in range(self.model.nq)];mujoco.mj_kinematics(self.model,self.ref)
  if p.motion_limited and getattr(p,'motion_following_error',0)>.006:return
  u=float(np.clip((p.clock('right',t)-7.4)/.35,0,1));blend=u*u*(3-2*u)
  target=self.ref.xpos[self.body]+self.ref.xmat[self.body].reshape(3,3)@self.local+blend*self.delta
  point=data.xpos[self.body]+data.xmat[self.body].reshape(3,3)@self.local
  pe=target-point;re=(Rotation.from_matrix(self.ref.xmat[self.hand].reshape(3,3))*Rotation.from_matrix(data.xmat[self.hand].reshape(3,3)).inv()).as_rotvec()
  jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp);mujoco.mj_jac(self.model,data,jp,jr,point,self.body)
  dofs=[self.model.joint(p.names[j]).dofadr[0]for j in indices];J=np.r_[jp[:,dofs],.15*jr[:,dofs]];e=np.r_[pe,.15*re]
  dq=J.T@np.linalg.solve(J@J.T+.005**2*np.eye(6),e);gain=.04 if p.motion_limited else .25;cap=.00012 if p.motion_limited else .0015;offset[indices]+=np.clip(gain*dq,-cap,cap);offset[indices]=np.clip(offset[indices],-.25,.25)
