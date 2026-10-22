"""Finite-force pneumatic jaw drives, controlled without per-tick USD edits.

An invisible fixed base replaces the existing world end of each prismatic
actuator. Workpieces remain independent dynamic bodies; no attachment is made.
"""
import numpy as np
from pxr import UsdGeom,UsdPhysics,PhysxSchema

def author(stage):
    result=[]
    for part in ['ring1','ring2','pin']:
        joints=[UsdPhysics.PrismaticJoint(p)for p in stage.Traverse()if str(p.GetPath()).startswith('/World/Clamps/'+part+'/')and p.IsA(UsdPhysics.PrismaticJoint)]
        for i,joint in enumerate(joints):
            assert not joint.GetBody0Rel().GetTargets()
            path=f'/World/ClampAnchors/{part}_{i}';base=UsdGeom.Xform.Define(stage,path).GetPrim();UsdPhysics.RigidBodyAPI.Apply(base);mass=UsdPhysics.MassAPI.Apply(base);mass.CreateMassAttr(1.);mass.CreateDiagonalInertiaAttr((1.,1.,1.));PhysxSchema.PhysxRigidBodyAPI.Apply(base).CreateDisableGravityAttr(True)
            root=UsdPhysics.FixedJoint.Define(stage,path+'/Mount');root.CreateBody1Rel().SetTargets([base.GetPath()]);UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim());PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim());joint.CreateBody0Rel().SetTargets([base.GetPath()]);limit=float(joint.GetLowerLimitAttr().Get()if part=='pin'else joint.GetUpperLimitAttr().Get());force=float(UsdPhysics.DriveAPI(joint.GetPrim(),'linear').GetMaxForceAttr().Get());result.append(dict(part=part,root=str(root.GetPath()),anchor=path,limit=limit,max_force_n=force))
    return result

class TensorClamps:
    def __init__(self,view,spec):
        self.spec=spec;self.art=view.create_articulation_view([r['root']for r in spec]);paths=list(self.art.prim_paths);assert self.art.count==len(spec);self.indices=np.arange(self.art.count,dtype=np.uint32);self.targets=np.zeros((self.art.count,1),np.float32)
        for row in spec:
            matches=[i for i,p in enumerate(paths)if str(p)==row['root']or str(p)==row['anchor']];assert len(matches)==1,(paths,row);row['index']=matches[0]
        actual=self.art.get_dof_max_forces()
        for row in spec:assert abs(float(actual[row['index'],0])-row['max_force_n'])<1e-5
    def command(self,fractions):
        state=dict(zip(['ring1','ring2','pin'],fractions))
        for row in self.spec:self.targets[row['index'],0]=row['limit']*float(state[row['part']])
        self.art.set_dof_position_targets(self.targets,self.indices)
