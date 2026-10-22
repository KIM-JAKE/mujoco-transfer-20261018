"""Offline/pre-attach ROAS column geometry authoring; no simulation or robot I/O.

Use ONLY in a new geometry version after the pinned original repeat cohort.
Never call this against an attached/running stage. Original taskworld/CAD/parts
stayfixed. The Duo hierarchy, its exact static-query geometry and fixed world
anchor move together; arms retain their existing local kinematics and angles.
"""
from math import radians,cos,sin

VERSION='roas-column248-plate12-v1-20261022'

def apply(stage,paths,config):
    from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
    if config.get('geometry_version')!=VERSION:
        raise ValueError('Explicit separate geometry version required')
    if config.get('stage_is_attached')is not False:
        raise ValueError('Author only a detached stage, before physics attachment')
    if stage.GetPrimAtPath('/World/ROASColumn248'):
        raise ValueError('Geometry already authored; refusing a second application')
    top=float(config['table_top_world_z_m']);plate=float(config['plate_thickness_m']);height=float(config['column_height_m']);mount_z=float(config.get('mount_face_local_z_m',0.))
    if not(abs(top-.660)<1e-9 and abs(plate-.012)<1e-9 and abs(height-.248)<1e-9 and abs(mount_z)<1e-5):
        raise ValueError('This geometry version requires660mm tabletop,12mm plate,248mm column andverifiednominalrootmountface')
    root=stage.GetPrimAtPath(paths['official_FR3_Duo']);static=stage.GetPrimAtPath('/World/StaticDuo');jointprim=stage.GetPrimAtPath('/World/RobotMountJoint')
    if not root or not static or not jointprim:
        raise ValueError('Expected original Duo root, StaticDuo query meshes and fixed mount joint')
    joint=UsdPhysics.FixedJoint(jointprim)
    if joint.GetBody0Rel().GetTargets():
        raise ValueError('Expected a world anchor with emptybody0; explicitframeconversionrequiredotherwise')
    if [str(x)for x in joint.GetBody1Rel().GetTargets()]!=[str(root.GetPath())]:
        raise ValueError('Mount joint must anchor the exact Duo root')
    cache=UsdGeom.XformCache(Usd.TimeCode.Default());before=cache.GetLocalToWorldTransform(root);p=before.ExtractTranslation();newz=top+plate+height-mount_z;dz=newz-float(p[2]);old_static=cache.GetLocalToWorldTransform(static);j=joint.GetLocalPos0Attr().Get()
    reset_descendants=[]
    for ancestor in [root,static]:
        for prim in Usd.PrimRange(ancestor):
            if prim!=ancestor and UsdGeom.Xformable(prim) and UsdGeom.Xformable(prim).GetResetXformStack():
                reset_descendants.append(prim)
    def shift_world(prim):
        xf=UsdGeom.Xformable(prim);reset=xf.GetResetXformStack()
        world=cache.GetLocalToWorldTransform(prim);q=world.ExtractTranslation();world.SetTranslateOnly(Gf.Vec3d(float(q[0]),float(q[1]),float(q[2])+dz));parent=cache.GetLocalToWorldTransform(prim.GetParent());local=world if reset else world*parent.GetInverse();xf.ClearXformOpOrder();xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble,'column248Pose').Set(local);xf.SetResetXformStack(reset)
    # World X/Y are taken from the actual inputstage, never staleplacement-spec.
    # Column/plate yaw follows the current Duo world heading; fixed z-up required.
    R=before.ExtractRotationMatrix()
    if max(abs(float(R[0][2])),abs(float(R[1][2])),abs(float(R[2][0])),abs(float(R[2][1])),abs(float(R[2][2])-1.))>1e-5:
        raise ValueError('Expected a z-up Duo mounting/referenceframe')
    shift_world(root);shift_world(static)
    for prim in reset_descendants:shift_world(prim)
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(float(j[0]),float(j[1]),float(j[2])+dz))
    geometry=UsdGeom.Xform.Define(stage,'/World/ROASColumn248');geometry.GetPrim().CreateAttribute('geometry:version',Sdf.ValueTypeNames.String).Set(VERSION)
    def cube(name,centerz,size):
        c=UsdGeom.Cube.Define(stage,'/World/ROASColumn248/'+name);c.CreateSizeAttr(1.);xf=UsdGeom.Xformable(c);xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(float(p[0]),float(p[1]),centerz));xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(before.ExtractRotationQuat());xf.AddScaleOp().Set(Gf.Vec3f(*size));c.CreateDisplayColorAttr([Gf.Vec3f(.58,.60,.63)]);UsdPhysics.CollisionAPI.Apply(c.GetPrim());material=UsdShade.Material.Get(stage,'/World/Materials/steel')
        if material:UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
        return str(c.GetPath())
    platepath=cube('Plate180x200x12',top+plate/2,(.180,.200,.012));columnpath=cube('Column80x120x248',top+plate+height/2,(.080,.120,.248))
    aftercache=UsdGeom.XformCache(Usd.TimeCode.Default());a=aftercache.GetLocalToWorldTransform(root).ExtractTranslation();b=aftercache.GetLocalToWorldTransform(static).ExtractTranslation();oldsp=old_static.ExtractTranslation()
    if abs(float(a[0])-float(p[0]))>1e-10 or abs(float(a[1])-float(p[1]))>1e-10 or abs(float(a[2])-newz)>1e-10:
        raise RuntimeError('Duo world poseauthoringfailed')
    if abs(float(b[2])-float(oldsp[2])-dz)>1e-10:
        raise RuntimeError('Static query geometrydidnotfollowDuo')
    return dict(geometry_version=VERSION,previous_Duo_root_world_m=list(map(float,p)),new_Duo_root_world_m=list(map(float,a)),Duo_origin_root_offset_m=[.037,0.,.050],new_Duo_origin_world_z_m=newz+.050,world_height_delta_m=dz,legacy_actual_world_XY_preserved=True,mount_face_local_z_m=mount_z,plate=platepath,column=columnpath,column_cross_section_m=[.080,.120],column_height_m=.248,plate_thickness_m=.012,plate_planform_m=[.180,.200],column_is_solid_conservative_envelope=True,profile_internal_slots_not_modelled=True,static_Duo_query_geometry_shifted=True,world_mount_anchor_shifted=True,reset_stack_descendants_world_shifted=[str(p.GetPath())for p in reset_descendants],old_motion_and_collision_proofs_invalidated_by_geometry_change=True,requires_new_kinematics_scene_queries_and_proofs=True,controller_force_friction_speed_or_acceptance_changes=False,physics_run_performed=False,geometry_motion_success_not_tested=True)
