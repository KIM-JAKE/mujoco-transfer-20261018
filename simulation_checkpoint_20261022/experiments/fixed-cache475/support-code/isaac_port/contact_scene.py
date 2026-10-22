"""Native PhysX task scene. Workpieces are unrestrained dynamic rigid bodies."""
from pathlib import Path
import json,math,os,hashlib,shutil
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf,Sdf
HERE=Path(__file__).resolve().parent

def pose(xf,p,q):
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float,p)))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))))

def build(out,full_environment=True,loop_gripper=True):
    source=json.loads((HERE/'generated/contact_task/contact-source.json').read_text())
    cache=HERE/'runs/.contact-scene-cache';cache.mkdir(exist_ok=True)
    key=hashlib.sha256((Path(__file__).read_bytes()+
        (HERE/'generated/contact_task/contact-source.json').read_bytes()+
        (HERE/'generated/contact_task/robot-filters.json').read_bytes()+
        f'{full_environment}-{loop_gripper}'.encode())).hexdigest()[:20]
    cached=cache/(key+'.usda');cached_meta=cache/(key+'.json')
    if cached.is_file() and cached_meta.is_file():
        # Both output and cache are one directory below runs/, preserving the
        # relative reference to the immutable converted visual asset.
        shutil.copyfile(cached,out/'scene.usda')
        record=json.loads(cached_meta.read_text())
        return source,record['body_paths'],record['adaptations']
    arrays=np.load(HERE/'generated/contact_task/collision.npz')
    converted=json.loads((HERE/'generated/cap_button/conversion-status.json').read_text())
    stage=Usd.Stage.CreateNew(str(out/'scene.usda'))
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,'/World').GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    cell=UsdGeom.Xform.Define(stage,'/World/Cell').GetPrim()
    cell.GetReferences().AddReference(os.path.relpath(HERE/converted['usd_path'],out))
    cell.GetVariantSet('Physics').SetVariantSelection('physx')
    # Remove instancing only in this task layer so imported collision attributes
    # can be disabled. All contact shapes are authored explicitly below.
    for prim in list(stage.Traverse()):
        if prim.IsInstance():prim.SetInstanceable(False)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
    names=set(source['body_names'][1:]);body_paths={p.GetName():str(p.GetPath())for p in stage.Traverse()if p.IsA(UsdGeom.Xform) and p.GetName() in names}
    assert len(body_paths)==len(names)
    for b in source['bodies']:
        prim=stage.GetPrimAtPath(body_paths[b['name']]);mass=UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr(max(b['mass'],1e-6));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*[max(v,1e-9)for v in b['inertia']]))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*b['com']));q=b['principal'];mass.CreatePrincipalAxesAttr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
        if b['name'].startswith(('left_','right_')):PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rb.CreateSolverPositionIterationCountAttr(64);rb.CreateSolverVelocityIterationCountAttr(16)
    from robot_collision_filters import apply as apply_robot_filters
    filter_adapt=apply_robot_filters(stage,HERE,body_paths)
    materials={}
    for name,friction in [('steel',.6),('rubber',1.0)]:
        material=UsdShade.Material.Define(stage,'/World/Materials/'+name)
        api=UsdPhysics.MaterialAPI.Apply(material.GetPrim());api.CreateStaticFrictionAttr(friction);api.CreateDynamicFrictionAttr(friction*.85);api.CreateRestitutionAttr(0)
        materials[name]=material
    shape_count=0
    def shape(parent,row,dynamic=False):
        nonlocal shape_count
        name=row['name'].replace('-','neg_');path=parent+'/'+name
        kind=row['kind']
        if kind==7:
            mesh=UsdGeom.Mesh.Define(stage,path);prim=mesh.GetPrim();key=row['mesh']
            mesh.CreatePointsAttr(arrays[key+'_vertices'].astype(np.float32));f=arrays[key+'_faces']
            mesh.CreateFaceVertexCountsAttr([3]*len(f));mesh.CreateFaceVertexIndicesAttr(f.reshape(-1));mesh.CreateSubdivisionSchemeAttr('none')
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull' if dynamic else 'none')
        elif kind==6:
            cube=UsdGeom.Cube.Define(stage,path);prim=cube.GetPrim();cube.CreateSizeAttr(2)
            UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(*row['size']))
        elif kind in [3,5]:
            cylinder=UsdGeom.Cylinder.Define(stage,path);prim=cylinder.GetPrim();cylinder.CreateRadiusAttr(row['size'][0]);cylinder.CreateHeightAttr(row['size'][1]*2);cylinder.CreateAxisAttr('Z')
        elif kind==2:
            sphere=UsdGeom.Sphere.Define(stage,path);prim=sphere.GetPrim();sphere.CreateRadiusAttr(row['size'][0])
        elif kind==0:
            cube=UsdGeom.Cube.Define(stage,path);prim=cube.GetPrim();cube.CreateSizeAttr(2)
            UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(8,8,.01));row=dict(row,position=[0,0,-.01])
        else:raise ValueError(('Unsupported collision primitive',row))
        pose(UsdGeom.Xformable(prim),row['position'],row['quaternion'])
        # The scale must be last in the transform stack (scale in local axes).
        xf=UsdGeom.Xformable(prim);ops=xf.GetOrderedXformOps();xf.SetXformOpOrder([p for p in ops if p.GetOpType()!=UsdGeom.XformOp.TypeScale]+[p for p in ops if p.GetOpType()==UsdGeom.XformOp.TypeScale])
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        api=PhysxSchema.PhysxCollisionAPI.Apply(prim);api.CreateContactOffsetAttr(.00005);api.CreateRestOffsetAttr(0)
        material=materials['rubber' if '_pad' in row['name'] else 'steel']
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
        UsdGeom.Imageable(prim).CreateVisibilityAttr('invisible');shape_count+=1
        return prim
    UsdGeom.Xform.Define(stage,'/World/Environment')
    for row in source['geoms']:
        if row.get('body')=='m80' and not full_environment and row['name'] not in ['instance_0538','instance_0539','instance_0540','instance_0541','instance_0545','instance_0546','instance_0547','instance_0548']:continue
        if row['name']=='instance_0159':continue # spring plunger and housing authored separately
        if row['static']:
            parent='/World/Environment/'+row['body'];UsdGeom.Xform.Define(stage,parent)
        else:parent=body_paths[row['body']]+'/Contact';UsdGeom.Xform.Define(stage,parent)
        shape(parent,row,not row['static'])
    for name,part in source['parts'].items():
        path=body_paths['workpiece_'+name];prim=stage.GetPrimAtPath(path)
        for child in prim.GetChildren():
            if child.IsA(UsdPhysics.Joint):child.SetActive(False)
        rb=UsdPhysics.RigidBodyAPI.Apply(prim);rb.CreateKinematicEnabledAttr(False);rb.CreateRigidBodyEnabledAttr(True)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(False)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateEnableCCDAttr(True)
        mass=UsdPhysics.MassAPI.Apply(prim);mass.CreateMassAttr(part['mass']);mass.CreateCenterOfMassAttr(Gf.Vec3f(0));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*part['inertia']));mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
        for row in part['colliders']:shape(path+'/Contact',row,True)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0)
    # Vendor's closed four-bar linkages. These constraints couple robot links,
    # never a workpiece to a hand or fixture.
    if loop_gripper:
        for i,row in enumerate(source['connect']):
            joint=UsdPhysics.SphericalJoint.Define(stage,'/World/GripperLoops/closure_'+str(i))
            joint.CreateBody0Rel().SetTargets([body_paths[row['body0']]]);joint.CreateBody1Rel().SetTargets([body_paths[row['body1']]])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*row['anchor0']));joint.CreateLocalPos1Attr(Gf.Vec3f(*row['anchor1']))
            joint.CreateExcludeFromArticulationAttr(True)
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(body_paths[row['body0']])).CreateFilteredPairsRel().AddTarget(body_paths[row['body1']])
    joint_prims={p.GetName():p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
    for side in ['left','right']:
        target=joint_prims[side+'_grip_right_driver_joint'];reference=joint_prims[side+'_grip_left_driver_joint']
        # Remove converter's unresolved Newton relation in this PhysX-only layer.
        target.RemoveAPI('NewtonMimicAPI')
        if loop_gripper:
            mimic=PhysxSchema.PhysxMimicJointAPI.Apply(target,'rotX')
            mimic.CreateReferenceJointRel().SetTargets([reference.GetPath()]);mimic.CreateGearingAttr(-1);mimic.CreateOffsetAttr(0)
            mimic.CreateNaturalFrequencyAttr(200);mimic.CreateDampingRatioAttr(1)
    # Explicit support floor beneath the original open-bottom pin socket.
    pin=source['parts']['pin'];z0=1.28225
    floor=dict(name='pin_socket_floor',kind=6,position=[-.555,-.16,z0-.002],quaternion=[1,0,0,0],size=[.014,.014,.002])
    shape('/World/Environment',floor)
    # Spring-return button, 2mm stroke and finite stiffness. Its measured joint
    # travel, contact impulse and dwell will drive the green signal.
    button=UsdGeom.Xform.Define(stage,'/World/Button').GetPrim();pose(UsdGeom.Xformable(button),[.3559690421575,-.24775,1.527],[math.sqrt(.5),math.sqrt(.5),0,0])
    UsdPhysics.RigidBodyAPI.Apply(button);mass=UsdPhysics.MassAPI.Apply(button);mass.CreateMassAttr(.015)
    PhysxSchema.PhysxRigidBodyAPI.Apply(button).CreateDisableGravityAttr(True)
    PhysxSchema.PhysxContactReportAPI.Apply(button).CreateThresholdAttr(0)
    cap=UsdGeom.Cylinder.Define(stage,'/World/Button/Cap');cap.CreateRadiusAttr(.014);cap.CreateHeightAttr(.003);cap.CreateAxisAttr('Z');cap.CreateDisplayColorAttr([Gf.Vec3f(.04,.55,.1)])
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim());PhysxSchema.PhysxCollisionAPI.Apply(cap.GetPrim()).CreateContactOffsetAttr(.00005)
    button_joint=UsdPhysics.PrismaticJoint.Define(stage,'/World/ButtonTravel');button_joint.CreateBody1Rel().SetTargets([button.GetPath()]);button_joint.CreateAxisAttr('Y')
    button_joint.CreateLocalPos0Attr(Gf.Vec3f(.3559690421575,-.24775,1.527));button_joint.CreateLocalRot1Attr(Gf.Quatf(math.sqrt(.5),Gf.Vec3f(-math.sqrt(.5),0,0)))
    button_joint.CreateLowerLimitAttr(0);button_joint.CreateUpperLimitAttr(.0022)
    drive=UsdPhysics.DriveAPI.Apply(button_joint.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(1500);drive.CreateDampingAttr(4);drive.CreateTargetPositionAttr(0);drive.CreateMaxForceAttr(15)
    # Housing back stop. The fixed CAD includes a solid cap; the dynamic cap
    # replaces its central contact surface rather than colliding with itself.
    shape('/World/Environment',dict(name='button_back',kind=5,position=[.355969,-.2395,1.527],quaternion=[math.sqrt(.5),math.sqrt(.5),0,0],size=[.018,.002,0]))
    scene=UsdPhysics.Scene.Define(stage,'/PhysicsScene');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
    api=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());api.CreateTimeStepsPerSecondAttr(480);api.CreateSolverTypeAttr('TGS');api.CreateEnableCCDAttr(True);api.CreateEnableGPUDynamicsAttr(False)
    stage.GetRootLayer().Save()
    adaptations=dict(**filter_adapt,colliders=shape_count,loop_gripper=loop_gripper,full_environment=full_environment,button_stiffness_n_m=1500,button_max_travel_m=.0022,pin_floor_z=z0)
    shutil.copyfile(out/'scene.usda',cached)
    cached_meta.write_text(json.dumps(dict(body_paths=body_paths,adaptations=adaptations),indent=2)+'\n')
    return source,body_paths,adaptations
