"""Offline IK preparation; final poses must be measured again in CPU PhysX."""
from pathlib import Path
import sys,json,time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent))
import compact_compare as cc
import compact_table_search as ct
from shallow_supply_search import freeik
spec=json.load(open(HERE/'placement-spec.json'))
targets=[]
source=json.load(open(cc.PORT/'generated/contact_task/contact-source.json'))
for p,b in ct.BANDS.items():
    side='right'if p=='ring1'else'left'
    P,R=cc.fk(side,np.array(b['q_insert']));targets.append(dict(part=p,stage='insert',side=side,tcp=P.tolist(),R=R.tolist(),seed=b['q_insert'],gap=b['gap']))
    Rp=cc.g.checker.rot(source['parts'][p]['quaternion']);R=Rp@np.array(b['localR']);P=np.array(source['parts'][p]['position'])+Rp@np.array(b['localP'])
    targets.append(dict(part=p,stage='pickup',side=side,tcp=P.tolist(),R=R.tolist(),seed=b['q_preinsert'],gap=b['gap']))
body=cc.model.body('official_FR3_Duo').id
cc.model.body_pos[body]=spec['duo_world_position'];cc.mujoco.mj_forward(cc.model,cc.data)
results=[]
for target in targets:
    side=target['side'];tcp=np.array(target['tcp']);R=np.array(target['R']);rgoal=Rotation.from_matrix(R@Rotation.from_euler('z',45,degrees=True).as_matrix());goal=tcp-R@np.array([0,0,.1034])
    def residual(q):
        cc.data.qpos[cc.AD[side]]=q;cc.mujoco.mj_kinematics(cc.model,cc.data)
        return np.r_[10*(cc.data.xpos[cc.FL[side]]-goal),(Rotation.from_matrix(cc.data.xmat[cc.FL[side]].reshape(3,3))*rgoal.inv()).as_rotvec()]
    best=None;rng=np.random.default_rng(58)
    seeds=[np.array(target['seed']),cc.READY[side],np.array([0,-.785,0,-2.35,0,1.57,.785])]
    seeds += [rng.uniform(cc.LOWER+.05,cc.UPPER-.05)for _ in range(12)]
    for seed in seeds:
        fit=least_squares(residual,np.clip(seed,cc.LOWER+.005,cc.UPPER-.005),bounds=(cc.LOWER+.005,cc.UPPER-.005),max_nfev=160,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        error=residual(fit.x);score=float(np.linalg.norm(error))
        if best is None or score<best['residual']:best=dict(q=fit.x.tolist(),residual=score,flange_position_error_m=float(np.linalg.norm(error[:3])/10),orientation_error_rad=float(np.linalg.norm(error[3:])))
        if best['flange_position_error_m']<.0001 and best['orientation_error_rad']<.0003:break
    best.update(target,offline_ik_pass=best['flange_position_error_m']<.0001 and best['orientation_error_rad']<.0003)
    best.pop('seed');results.append(best)
    (HERE/'prepared-ik-targets.json').write_text(json.dumps(dict(scope='Offline joint targets for independent PhysX verification; not a simulation pass',placement=spec,targets=results),indent=2))
    print('ROAS_OFFLINE_IK',target['part'],target['stage'],best['offline_ik_pass'],best['flange_position_error_m'],best['orientation_error_rad'],flush=True)
