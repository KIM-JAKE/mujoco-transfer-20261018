"""Offline candidates for moving the correctly sized robot table on the floor.
Preserves every M80 and supply pose; candidates require PhysX verification.
"""
from pathlib import Path
import json,math,time
import numpy as np,mujoco,fcl
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
HERE=Path(__file__).resolve().parent;PORT=HERE.parents[1]
spec=json.load(open(HERE/'placement-spec.json'));prepared=json.load(open(HERE/'prepared-ik-targets.json'))
source=json.load(open(PORT/'generated/contact_task/contact-source.json'));arr=np.load(PORT/'generated/contact_task/collision.npz')
m=mujoco.MjModel.from_xml_path(str(HERE.parent/'torque-native-model.xml'));d=mujoco.MjData(m);body=m.body('official_FR3_Duo').id
AD={s:np.array([m.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0]for j in range(1,8)])for s in ['left','right']};FL={s:m.body(s+'_arm_fr3v2_link8').id for s in AD}
limits=np.array([m.jnt_range[m.joint(f'left_arm_fr3v2_joint{j}').id]for j in range(1,8)]);low=limits[:,0]+.005;high=limits[:,1]-.005
meshes={}
def geometry_hits(center,size):
    center=np.asarray(center);size=np.asarray(size)
    shape=fcl.CollisionObject(fcl.Box(*size),fcl.Transform(np.eye(3),center));hits=[]
    lo=center-size/2;hi=center+size/2
    for row in source['geoms']:
        if row['body']!='m80' or row['name']=='instance_0159':continue
        R=Rotation.from_quat(np.array(row['quaternion'])[[1,2,3,0]]).as_matrix();P=np.array(row['position']);extent=np.abs(R)@np.array(row['size'])*1.001
        if np.any(P+extent<lo)or np.any(P-extent>hi):continue
        if row['kind']==7:
            key=row['mesh']
            if key not in meshes:
                v=arr[key+'_vertices'];f=arr[key+'_faces'];mshape=fcl.BVHModel();mshape.beginModel(len(v),len(f));mshape.addSubModel(v,f);mshape.endModel();meshes[key]=mshape
            other=meshes[key]
        elif row['kind']==6:other=fcl.Box(*(np.array(row['size'])*2))
        elif row['kind']in[3,5]:other=fcl.Cylinder(row['size'][0],row['size'][1]*2)
        else:continue
        obj=fcl.CollisionObject(other,fcl.Transform(R,P))
        if fcl.collide(shape,obj,fcl.CollisionRequest(),fcl.CollisionResult()):hits.append(row['name'])
    return hits

def table_hits(center):return geometry_hits([*center,.36],[.68,.66,.6])

def solve(target,position):
    m.body_pos[body]=position;mujoco.mj_forward(m,d);side=target['side'];R=np.array(target['R']);tcp=np.array(target['tcp']);goal=tcp-R@np.array([0,0,.1034]);rot=Rotation.from_matrix(R@Rotation.from_euler('z',45,degrees=True).as_matrix())
    def fun(q):
        d.qpos[AD[side]]=q;mujoco.mj_kinematics(m,d);return np.r_[10*(d.xpos[FL[side]]-goal),(Rotation.from_matrix(d.xmat[FL[side]].reshape(3,3))*rot.inv()).as_rotvec()]
    best=None;rng=np.random.default_rng(71)
    for seed in [np.array(target['q'])]+[rng.uniform(low,high)for _ in range(5)]:
        fit=least_squares(fun,np.clip(seed,low,high),bounds=(low,high),max_nfev=130,ftol=1e-7,xtol=1e-7,gtol=1e-7);e=fun(fit.x);score=float(np.linalg.norm(e))
        if best is None or score<best['residual']:best=dict(q=fit.x.tolist(),residual=score,flange_position_error_m=float(np.linalg.norm(e[:3])/10),orientation_error_rad=float(np.linalg.norm(e[3:])))
        if best['flange_position_error_m']<.0001 and best['orientation_error_rad']<.0003:break
    return dict(**{k:v for k,v in target.items()if k not in best and k!='offline_ik_pass'},**best,offline_ik_pass=best['flange_position_error_m']<.0001 and best['orientation_error_rad']<.0003)

if __name__=='__main__':
    report=dict(scope='Offline solid cabinet CAD overlap and IK preparation, not PhysX or full path success',cases=[])
    for cy in [-.6,-.575,-.55,-.525,-.5,-.475,-.45,-.425,-.4]:
        position=[spec['duo_world_position'][0],cy,spec['duo_world_position'][2]];hits=table_hits(position[:2]);case=dict(duo_position=position,table_machine_intersections=hits,targets=[])
        if not hits:
            for target in prepared['targets']:case['targets'].append(solve(target,position))
            case['all_endpoint_ik_pass']=all(t['offline_ik_pass']for t in case['targets'])
        report['cases'].append(case);(HERE/'placement-search.json').write_text(json.dumps(report,indent=2));print('ROAS_PLACEMENT',cy,'hits',hits,'IK',[(t['part'],t['stage'],t['offline_ik_pass'])for t in case['targets']],flush=True)
