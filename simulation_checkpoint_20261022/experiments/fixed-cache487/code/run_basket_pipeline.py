"""Continuous physical pipeline test using recorded joint commands, never object-state replay."""
from pathlib import Path
import os,sys,json,shutil,traceback,math
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Gf
import omni.kit.app
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));PORT=HERE.parents[1];OUT=PORT/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True);source=PORT/'runs'/'native-roas-table-bin-quick07';sys.path.insert(0,str(PORT))
try:
    stage=Usd.Stage.Open(str(source/'scene-authored.usda'));paths=json.load(open(source/'body-paths.json'));hands=paths.pop('native_hands');initial=json.load(open(source/'initial-robot-pose.json'));plan=json.load(open(source/'arm-plan.json'));old=json.load(open(source/'physx-pickup-results.json'));plan['parts']={'pin':dict(side='right')};plan['initial_apertures_m']={'left':.08,'right':.022};plan['handoffs']['pin']['insert_q_seeds'].insert(0,[-.1869964,1.0797545,1.1027995,-.9652948,1.5667019,2.1647879,-.2077391]);plan['live_demo_control']=True
    def pose(path,p,q):
        xf=UsdGeom.Xformable(stage.GetPrimAtPath(path));xf.ClearXformOpOrder();xf.SetResetXformStack(True);x,y,z,w=map(float,q);M=Gf.Matrix4d().SetRotate(Gf.Quatd(w,Gf.Vec3d(x,y,z)));M.SetTranslateOnly(Gf.Vec3d(*map(float,p)));xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(M)
    pose(paths['workpiece_pin'],[.19,-.79,.91605],[1,0,0,0])
    for row in old.get('clutter_settled_objects',[]):
        if True:pose(row['path'],row['transform'][:3],row['transform'][3:])
    asset=np.load(HERE/'native-official-finger.npz')
    for side in ['left','right']:
        for label in ['left','right']:
            path=hands[side]+'/'+side+'_'+label+'_finger'
            for name in ['mount','sledge','diagonal','tip']:stage.RemovePrim(path+'/'+name)
            m=UsdGeom.Mesh.Define(stage,path+'/tip');m.CreatePointsAttr(asset['vertices'].astype(np.float32));f=asset['faces'];m.CreateFaceVertexCountsAttr([3]*len(f));m.CreateFaceVertexIndicesAttr(f.ravel());m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(.08,.08,.10)]);UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(UsdShade.Material.Get(stage,'/World/Materials/rubber'),UsdShade.Tokens.strongerThanDescendants,'physics');pc=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());pc.CreateContactOffsetAttr(.00005);pc.CreateRestOffsetAttr(0.);pc.CreateTorsionalPatchRadiusAttr(.003);pc.CreateMinTorsionalPatchRadiusAttr(.002)
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if not path.startswith('/World/Clamps/')or not prim.IsA(UsdPhysics.PrismaticJoint):continue
        j=UsdPhysics.PrismaticJoint(prim);pin=path.startswith('/World/Clamps/pin/');UsdPhysics.DriveAPI(prim,'linear').GetTargetPositionAttr().Set(0.)
        if pin:
            a=j.GetLocalPos0Attr().Get();j.GetLocalPos0Attr().Set(Gf.Vec3f(a[0],a[1],1.511));body=str(j.GetBody1Rel().GetTargets()[0]);bp=UsdGeom.Xformable(stage.GetPrimAtPath(body)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation();q=[0,0,0,1]if body.endswith('0')else[0,0,1,0];pose(body,[bp[0],bp[1],1.511],q)
    from simple_pin_collision import apply as simplify_pin
    simple_pin_model=simplify_pin(stage,paths['workpiece_pin'])
    drives={};qcurrent={}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in prim.GetName():
            n=prim.GetName();qcurrent[n]=initial['ready_q'][n.split('_')[0]][int(n[-1])-1];drives[n]=UsdPhysics.DriveAPI(prim,'angular')
    # Material and clamp assumptions match the successful independent pin trial.
    for j in [0,1]:
        prim=stage.GetPrimAtPath('/World/Clamps/pin/jaw_'+str(j)+'/Jaw')
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpType()==UsdGeom.XformOp.TypeScale:op.Set(Gf.Vec3f(.001,.0015,.003))
    for name,roots,k in [('PinElastic',[paths['workpiece_pin']],100000.),('SpringDetents',['/World/Environment/m80/instance_'+v for v in ['0560','0562','0564','0566']],10000.)]:
        m=UsdShade.Material.Define(stage,'/World/Materials/'+name);ma=UsdPhysics.MaterialAPI.Apply(m.GetPrim());ma.CreateStaticFrictionAttr(.4);ma.CreateDynamicFrictionAttr(.3);ma.CreateRestitutionAttr(0.)
        pa=PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim());pa.CreateCompliantContactStiffnessAttr(k);pa.CreateCompliantContactDampingAttr(15.);pa.CreateCompliantContactAccelerationSpringAttr(False)
        for pr in stage.Traverse():
            if any(str(pr.GetPath()).startswith(path)for path in roots)and pr.HasAPI(UsdPhysics.CollisionAPI):UsdShade.MaterialBindingAPI.Apply(pr).Bind(m,UsdShade.Tokens.strongerThanDescendants,'physics')
    # All rings have the same metal material. Identity comes from measured size.
    from basket_size import measure_stage
    stock=json.load(open(source/'clutter-body-paths.json'))
    classified=measure_stage(stage,list(stock.values()))
    for path in stock.values():
        for pr in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if pr.IsA(UsdGeom.Mesh):UsdGeom.Mesh(pr).CreateDisplayColorAttr([Gf.Vec3f(.62,.65,.69)])
    report=dict(engine='Isaac Sim 6.1 Omni PhysX CPU',part_attachments=False,full_task_pass=False,continuous_cycle_pass=False,hardware_tested=False,vision_end_to_end_pass=False,pin_collision_approximation=simple_pin_model,size_classification=classified,scene_ring_color='identical steel',starts_with_rings_already_installed=False)
    (OUT/'arm-plan.json').write_text(json.dumps(plan,indent=2));(OUT/'body-paths.json').write_text(json.dumps(dict(**paths,native_hands=hands),indent=2));stage.GetRootLayer().Export(str(OUT/'scene-authored.usda'))
    options=json.load(open(OUT/'pipeline-options.json'if(OUT/'pipeline-options.json').exists()else HERE/'bin-picking/pipeline-options.json'))
    if options.get('motion_file'):
        from execute_retimed_motion import execute
    else:
        from execute_basket_pipeline import execute
    execute(stage,paths,hands,drives,initial,report,OUT,HERE)
    omni.kit.app.get_app().post_quit(0)
except Exception:
    (OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
