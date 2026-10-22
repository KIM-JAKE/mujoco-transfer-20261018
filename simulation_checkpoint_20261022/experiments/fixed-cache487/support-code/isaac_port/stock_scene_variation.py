"""Perturbed pickup initial conditions; M80 and insertion fixtures stay fixed."""
from pathlib import Path
import copy,json
import numpy as np
from scipy.spatial.transform import Rotation


def matrix(part):
    if 'T_cell_part'in part:
        t=np.asarray(part['T_cell_part'],float)
    else:
        q=np.asarray(part['quaternion'],float);p=np.asarray(part['position'],float)
        if q.shape!=(4,) or abs(np.linalg.norm(q)-1)>1e-5 or p.shape!=(3,):raise ValueError('Bad stock pose')
        t=np.eye(4);t[:3,:3]=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();t[:3,3]=p
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1],atol=1e-8) or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6) or abs(np.linalg.det(t[:3,:3])-1)>1e-6:raise ValueError('Bad stock transform')
    return t


def prepare_source(source,scene):
    scene=json.loads(Path(scene).read_text())if isinstance(scene,(str,Path))else scene
    updated=copy.deepcopy(source);deltas={}
    if set(scene['parts'])!=set(['ring1','ring2','pin']):raise ValueError('All three initial part poses required')
    for name,row in scene['parts'].items():
        old=matrix(source['parts'][name]);new=matrix(row);delta=new@np.linalg.inv(old)
        if np.linalg.norm(new[:3,3]-old[:3,3])>.035 or Rotation.from_matrix(delta[:3,:3]).magnitude()>.2:raise ValueError('Stock variation outside tested planning envelope')
        updated['parts'][name]['position']=new[:3,3].tolist();updated['parts'][name]['quaternion']=Rotation.from_matrix(new[:3,:3]).as_quat()[[3,0,1,2]].tolist();deltas[name]=delta
    support=deltas['pin']if scene.get('move_pin_holder_with_pin',False)else np.eye(4)
    for row in updated['geoms']:
        if row['name']=='cap_supply_pin_socket_0':
            rot=Rotation.from_quat(np.asarray(row['quaternion'])[[1,2,3,0]]).as_matrix();row['position']=(support[:3,:3]@row['position']+support[:3,3]).tolist();row['quaternion']=Rotation.from_matrix(support[:3,:3]@rot).as_quat()[[3,0,1,2]].tolist()
    return updated,support,dict(m80_machine_fixed=True,insertion_goals_fixed=True,pickup_initial_conditions=updated['parts'],pin_holder_moved_with_pin=bool(scene.get('move_pin_holder_with_pin',False)),pin_support_delta=support.tolist())


def apply(stage,source,scene):
    from pxr import UsdGeom,Gf
    updated,support,report=prepare_source(source,scene)
    names={'cap_supply_pin_socket_0','pin_socket_floor','PinStockSleeve'}
    selected=[p for p in stage.Traverse()if p.GetName()in names and p.IsA(UsdGeom.Xformable)]
    cache=UsdGeom.XformCache();world={str(p.GetPath()):cache.GetLocalToWorldTransform(p)for p in selected};delta=Gf.Matrix4d(*map(float,support.T.reshape(-1)))
    moved=[]
    if not np.allclose(support,np.eye(4)):
        for prim in selected:
            xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();xf.SetResetXformStack(True);xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble,'observedStock').Set(world[str(prim.GetPath())]*delta);moved.append(str(prim.GetPath()))
        if len(moved)<3:raise RuntimeError('Missing source pin support shapes')
    report['moved_support_paths']=moved
    return updated,report
