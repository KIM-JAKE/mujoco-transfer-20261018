"""Small kinematic-only model for Cartesian seeds; never a dynamics engine.

The CAD collision model remains independent. Final execution is measured in
Isaac Sim PhysX. Omitting static CAD bodies here makes local IK inexpensive.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
spec=json.load(open(HERE/'placement-spec-candidate.json'))
model=mujoco.MjModel.from_xml_path(str(HERE.parent/'torque-native-model.xml'));data=mujoco.MjData(model)
model.body_pos[model.body('official_FR3_Duo').id]=spec['duo_world_position']
AD={};LO={};HI={};FL={}
for side in ['left','right']:
    ids=[model.joint(f'{side}_arm_fr3v2_joint{j}').id for j in range(1,8)]
    AD[side]=model.jnt_qposadr[ids];LO[side]=model.jnt_range[ids,0]+.005;HI[side]=model.jnt_range[ids,1]-.005
    FL[side]=model.body(f'{side}_arm_fr3v2_link8').id

def solve(side,tcp,R,seed):
    seed=np.asarray(seed);tcp=np.asarray(tcp);R=np.asarray(R)
    p=tcp-R@np.array([0,0,.1034]);r=Rotation.from_matrix(R@Rotation.from_euler('z',45,degrees=True).as_matrix())
    def residual(q):
        data.qpos[AD[side]]=q;mujoco.mj_kinematics(model,data)
        return np.r_[10*(data.xpos[FL[side]]-p),(Rotation.from_matrix(data.xmat[FL[side]].reshape(3,3))*r.inv()).as_rotvec(),1e-5*(q-seed)]
    fit=least_squares(residual,np.clip(seed,LO[side],HI[side]),bounds=(LO[side],HI[side]),max_nfev=100,ftol=1e-10,xtol=1e-10,gtol=1e-10)
    error=residual(fit.x)
    return fit.x if np.linalg.norm(error[:3])<1e-4 and np.linalg.norm(error[3:6])<1e-4 else None

def alternatives(side,tcp,R,seed):
    """Nearby redundant postures with the same flange pose, closest first."""
    q=solve(side,tcp,R,seed)
    if q is None:return
    yield q
    data.qpos[AD[side]]=q;mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
    jp=np.zeros((3,model.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(model,data,jp,jr,FL[side])
    vids=[model.jnt_dofadr[model.joint(f'{side}_arm_fr3v2_joint{j}').id]for j in range(1,8)]
    _,_,V=np.linalg.svd(np.vstack([jp[:,vids],jr[:,vids]]));null=V[-1]
    for offset in [.025,-.025,.05,-.05,.10,-.10,.16,-.16]:
        result=solve(side,tcp,R,q+offset*null)
        if result is not None and np.max(abs(result-seed))<.2:yield result
