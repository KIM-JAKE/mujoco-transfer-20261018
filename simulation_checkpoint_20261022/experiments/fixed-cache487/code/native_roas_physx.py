"""Author and simulate the original M80 supply with native Hand and ROAS table.

Executed in Isaac Sim's installed CPU Omni PhysX runtime. No GPU/RTX, mocap
robot, part attachments, or hardware connections. Reach tests precede any
claim about a complete assembly cycle.
"""
from pathlib import Path
import sys,json,math,hashlib,traceback,os
import numpy as np
import omni.physx,omni.kit.app
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf,UsdUtils,Sdf,PhysicsSchemaTools
HERE=Path(__file__).resolve().parent;PORT=HERE.parents[1];sys.path.insert(0,str(PORT))
CASE=os.environ.get('M80_ROAS_CASE','baseline');SUFFIX='-candidate'if CASE=='candidate'else''
OUT=PORT/'runs'/os.environ.get('M80_ROAS_RUN','native-roas-table-03'if os.environ.get('M80_PICKUP_PROBE')=='1'else'native-roas-table-02'if SUFFIX else'native-roas-table-01');OUT.mkdir(exist_ok=True)
spec=json.load(open(HERE/('placement-spec'+SUFFIX+'.json')))

def pose(prim,p,q=(1,0,0,0)):
    xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,'roasPose').Set(Gf.Vec3d(*map(float,p)));xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble,'roasPose').Set(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))))

def box(stage,path,p,size,color=(.2,.25,.28)):
    c=UsdGeom.Cube.Define(stage,path);c.CreateSizeAttr(1);pose(c.GetPrim(),p);c.AddScaleOp().Set(Gf.Vec3f(*map(float,size)));c.CreateDisplayColorAttr([Gf.Vec3f(*color)]);UsdPhysics.CollisionAPI.Apply(c.GetPrim());return c

def cpu_payload_colliders(stage,paths):
    """Preserve ring holes/pin relief using CAD-derived convex sectors.

    SDF generation requires a CUDA cooking context even for CPU dynamics. These
    explicit sectors remove that dependency without substituting a solid hull.
    """
    archive=np.load(HERE/'cpu-convex-parts.npz');rows=json.load(open(HERE/'cpu-convex-parts.json'))
    for part,pieces in rows.items():
        parent=paths['workpiece_'+part]
        for suffix in ['Contact','PrecisionContact','PrecisionPinShell']:
            pr=stage.GetPrimAtPath(parent+'/'+suffix)
            if pr:pr.SetActive(False)
        for item in pieces:
            key=item['key'];mesh=UsdGeom.Mesh.Define(stage,parent+'/CPUConvexContact/'+key);mesh.CreatePointsAttr(archive[key+'_v'].astype(np.float32));faces=archive[key+'_f'];mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.reshape(-1));mesh.CreateSubdivisionSchemeAttr('none');pose(mesh.GetPrim(),list(map(float,item['position'].split())),list(map(float,item['quaternion'].split())))
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull');ca=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim());ca.CreateContactOffsetAttr(.00005);ca.CreateRestOffsetAttr(0)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material.Get(stage,'/World/Materials/steel'),UsdShade.Tokens.strongerThanDescendants,'physics');UsdGeom.Imageable(mesh.GetPrim()).CreateVisibilityAttr('invisible')
    return {p:len(v)for p,v in rows.items()}

def exact_static_duo(stage,paths):
    # The Duo is bolted down. Its concave brackets can use exact static CAD
    # triangles; dynamic convex hulls incorrectly fill the arm mount openings.
    contact=stage.GetPrimAtPath(paths['official_FR3_Duo']+'/Contact')
    if contact:contact.SetActive(False)
    root=UsdGeom.Xform.Define(stage,'/World/StaticDuo');pose(root.GetPrim(),spec['duo_world_position'],(math.sqrt(.5),0,0,math.sqrt(.5)))
    source=json.load(open(PORT/'generated/contact_task/contact-source.json'));archive=np.load(PORT/'generated/contact_task/collision.npz')
    neighbors=[paths['official_FR3_Duo']]+[paths[s+b]for s in ['left','right']for b in ['_arm_base','_arm_fr3v2_link0']]
    for row in source['geoms']:
        if row['body']!='official_FR3_Duo':continue
        mesh=UsdGeom.Mesh.Define(stage,'/World/StaticDuo/'+row['name']);mesh.CreatePointsAttr(archive[row['mesh']+'_vertices'].astype(np.float32));faces=archive[row['mesh']+'_faces'];mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.reshape(-1));mesh.CreateSubdivisionSchemeAttr('none');pose(mesh.GetPrim(),row['position'],row['quaternion']);UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none');UsdPhysics.FilteredPairsAPI.Apply(mesh.GetPrim()).CreateFilteredPairsRel().SetTargets(neighbors);UsdGeom.Imageable(mesh.GetPrim()).CreateVisibilityAttr('invisible')

def native_hand(stage,side,paths):
    link=paths[side+'_arm_fr3v2_link8'];handpath=link+'/'+side+'_NativeHand';hand=UsdGeom.Xform.Define(stage,handpath).GetPrim()
    angle=-math.pi/4;q=(math.cos(angle/2),0,0,math.sin(angle/2));pose(hand,[0,0,0],q)
    rb=UsdPhysics.RigidBodyAPI.Apply(hand);mass=UsdPhysics.MassAPI.Apply(hand);mass.CreateMassAttr(.7);mass.CreateCenterOfMassAttr(Gf.Vec3f(0,.01,.03));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.001,.0025,.0017));PhysxSchema.PhysxRigidBodyAPI.Apply(hand).CreateDisableGravityAttr(True)
    fixed=UsdPhysics.FixedJoint.Define(stage,'/World/NativeJoints/'+side+'_native_mount');fixed.CreateBody0Rel().SetTargets([link]);fixed.CreateBody1Rel().SetTargets([handpath]);fixed.CreateLocalPos0Attr(Gf.Vec3f(0));fixed.CreateLocalPos1Attr(Gf.Vec3f(0));fixed.CreateLocalRot0Attr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])));fixed.CreateLocalRot1Attr(Gf.Quatf(1))
    neighbors=[paths[side+'_arm_fr3v2_link7'],paths[side+'_arm_fr3v2_link8']]
    UsdPhysics.FilteredPairsAPI.Apply(hand).CreateFilteredPairsRel().SetTargets(neighbors)
    assets=np.load(HERE/'native-hand-assets.npz');mesh=UsdGeom.Mesh.Define(stage,handpath+'/Palm');mesh.CreatePointsAttr(assets['vertices'].astype(np.float32));faces=assets['faces'];mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.reshape(-1));mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDisplayColorAttr([Gf.Vec3f(.83,.84,.86)]);UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
    finger_joints=[]
    for sign,label in [(1,'left'),(-1,'right')]:
        path=handpath+'/'+side+'_'+label+'_finger';finger=UsdGeom.Xform.Define(stage,path).GetPrim();fq=(1,0,0,0)if sign==1 else(0,0,0,1);pose(finger,[0,sign*.04,.0584],fq)
        UsdPhysics.RigidBodyAPI.Apply(finger);ma=UsdPhysics.MassAPI.Apply(finger);ma.CreateMassAttr(.015);ma.CreateCenterOfMassAttr(Gf.Vec3f(0,.01,.025));ma.CreateDiagonalInertiaAttr(Gf.Vec3f(.00001));PhysxSchema.PhysxRigidBodyAPI.Apply(finger).CreateDisableGravityAttr(True)
        UsdPhysics.FilteredPairsAPI.Apply(finger).CreateFilteredPairsRel().SetTargets(neighbors)
        for name,p,size,rx,rz in [('mount',[0,.0185,.011],[.022,.015,.020],0,0),('sledge',[0,.0068,.0022],[.022,.0088,.0038],0,0),('diagonal',[0,.0159,.02835],[.0175,.007,.0235],sign*math.pi/6,0 if sign==1 else math.pi),('tip',[0,.00758,.04525],[.0175,.0152,.0185],0,0)]:
            geom=box(stage,path+'/'+name,p,size,(.08,.08,.1))
            if rx or rz:
                rot=Gf.Rotation(Gf.Vec3d(1,0,0),math.degrees(rx))*Gf.Rotation(Gf.Vec3d(0,0,1),math.degrees(rz));qq=rot.GetQuat();pose(geom.GetPrim(),p,(qq.GetReal(),*qq.GetImaginary()));geom.AddScaleOp().Set(Gf.Vec3f(*size))
            mat=UsdShade.Material.Get(stage,'/World/Materials/rubber');UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics');PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim()).CreateContactOffsetAttr(.00005)
        joint=UsdPhysics.PrismaticJoint.Define(stage,'/World/NativeJoints/'+side+'_native_'+label);joint.CreateBody0Rel().SetTargets([handpath]);joint.CreateBody1Rel().SetTargets([path]);joint.CreateAxisAttr('Y');joint.CreateLocalPos0Attr(Gf.Vec3f(0,0,.0584));joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateLocalRot0Attr(Gf.Quatf(fq[0],Gf.Vec3f(*fq[1:])));joint.CreateLocalRot1Attr(Gf.Quatf(1));joint.CreateLowerLimitAttr(0);joint.CreateUpperLimitAttr(.04)
        PhysxSchema.JointStateAPI.Apply(joint.GetPrim(),'linear').CreatePositionAttr(.04)
        # 95 mm/s total aperture, shared by the two moving fingers.
        PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateMaxJointVelocityAttr(.0475)
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(5000);drive.CreateDampingAttr(100);drive.CreateMaxForceAttr(30);drive.CreateTargetPositionAttr(.04)
        finger_joints.append(joint)
    return handpath,finger_joints

def add_roas(stage,include_supply=True):
    root='/World/ROAS';UsdGeom.Xform.Define(stage,root);cx,cy=spec['duo_world_position'][:2];w=.68;d=.66;t=.06;base=.06;top=.66
    for ix,sx in enumerate([-1,1]):
        for iy,sy in enumerate([-1,1]):
            x=cx+sx*(w-t)/2;y=cy+sy*(d-t)/2
            box(stage,f'{root}/Post{ix}{iy}',[x,y,(base+top)/2],[t,t,.6])
            wheel=UsdGeom.Cylinder.Define(stage,f'{root}/Caster{ix}{iy}');wheel.CreateRadiusAttr(.025);wheel.CreateHeightAttr(.025);wheel.CreateAxisAttr('X');pose(wheel.GetPrim(),[x,y,.025]);UsdPhysics.CollisionAPI.Apply(wheel.GetPrim());box(stage,f'{root}/CasterMount{ix}{iy}',[x,y,.0525],[.045,.045,.015])
    for iz,z in enumerate([base+.03,top-.03]):
        for iy,sy in enumerate([-1,1]):box(stage,f'{root}/RailX{iz}{iy}',[cx,cy+sy*(d-t)/2,z],[w-2*t,t,t])
        for ix,sx in enumerate([-1,1]):box(stage,f'{root}/RailY{iz}{ix}',[cx+sx*(w-t)/2,cy,z],[t,d-2*t,t])
    for i in range(9):
        y=cy-.27+.03+i*.06;box(stage,f'{root}/TopProfile{i}',[cx,y,top-.03],[.56,.06,.06])
        if i!=4:box(stage,f'{root}/BottomProfile{i}',[cx,y,base+.03],[.56,.06,.06])
    for sign,label in [(-1,'Front'),(1,'Rear')]:box(stage,root+'/Door'+label,[cx,cy+sign*.323,.36],[.56,.003,.47],(.76,.78,.8))
    for sign,label in [(-1,'Left'),(1,'Right')]:box(stage,root+'/Door'+label,[cx+sign*.333,cy,.36],[.003,.54,.47],(.76,.78,.8))
    if not include_supply:return
    # Separate slim supply table preserves the old heads without legs crossing
    # the ROAS cabinet; it is not attached to either M80 or the robot table.
    sx,sy,sz=spec['supply_table']['center'];box(stage,'/World/SupplyTable/Top',[sx,sy,sz-.0125],[.8,.15,.025],(.05,.24,.27))
    for i,dx in enumerate([-.375,.375]):
        for j,dy in enumerate([-.05,.05]):box(stage,f'/World/SupplyTable/Leg{i}{j}',[sx+dx,sy+dy,(sz-.025)/2],[.025,.025,sz-.025],(.05,.24,.27))
    for part,x in [('ring1',-.02),('ring2',-.45)]:
        if sz<1.04:box(stage,'/World/SupplyTable/'+part+'PostExtension',[x,-.16,(sz+1.04)/2],[.024,.024,1.04-sz],(.1,.6,.55))
    box(stage,'/World/SupplyTable/PinPost',[-.603,-.16,(sz+1.26625)/2],[.012,.012,1.26625-sz],(.05,.6,.5))
    box(stage,'/World/SupplyTable/PinRailCrossbar',[-.603,-.16,1.27025],[.030,.036,.008],(.05,.6,.5))

def main():
    from contact_scene import build
    source,paths,adapt=build(OUT,True,False)
    stage=Usd.Stage.Open(str(OUT/'scene.usda'));stage.SetEditTarget(stage.GetRootLayer())
    # The source cache contains a relative reference valid for a runs/ child.
    assert stage.GetPrimAtPath(paths['official_FR3_Duo'])
    from sensor_geometry import apply as apply_sensors
    from precision_collisions import apply as apply_precision
    adapt.update(apply_sensors(stage));adapt.update(apply_precision(stage,PORT,paths));convex_counts=cpu_payload_colliders(stage,paths)
    allprims=list(stage.Traverse());removed=[]
    for prim in allprims:
        if not prim.IsValid():continue
        name=prim.GetName();path=str(prim.GetPath())
        if name.startswith('duo_common_')or name in ('left_adapter','right_adapter'):
            prim.SetActive(False);removed.append(path)
        elif prim.IsA(UsdPhysics.Joint):
            joint=UsdPhysics.Joint(prim);refs=[str(x)for x in list(joint.GetBody0Rel().GetTargets())+list(joint.GetBody1Rel().GetTargets())]
            if any('_grip_'in p or '_adapter'in p for p in refs):prim.SetActive(False);removed.append(path)
    duo=stage.GetPrimAtPath(paths['official_FR3_Duo']);q=(math.sqrt(.5),0,0,math.sqrt(.5));pose(duo,spec['duo_world_position'],q)
    fixed=[]
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.FixedJoint):
            joint=UsdPhysics.FixedJoint(prim)
            anchors=list(joint.GetBody0Rel().GetTargets())
            if list(joint.GetBody1Rel().GetTargets())==[Sdf.Path(paths['official_FR3_Duo'])]and (not anchors or all(not stage.GetPrimAtPath(a).HasAPI(UsdPhysics.RigidBodyAPI)for a in anchors)):
                fixed.append(str(prim.GetPath()));prim.SetActive(False)
    assert fixed,'Duo world anchor not found'
    exact_static_duo(stage,paths)
    duo.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    mount=UsdPhysics.FixedJoint.Define(stage,'/World/RobotMountJoint');mount.CreateBody1Rel().SetTargets([paths['official_FR3_Duo']]);mount.CreateLocalPos0Attr(Gf.Vec3f(*spec['duo_world_position']));mount.CreateLocalPos1Attr(Gf.Vec3f(0));mount.CreateLocalRot0Attr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])));mount.CreateLocalRot1Attr(Gf.Quatf(1));mount.CreateExcludeFromArticulationAttr(False);UsdPhysics.ArticulationRootAPI.Apply(mount.GetPrim());PhysxSchema.PhysxArticulationAPI.Apply(mount.GetPrim()).CreateEnabledSelfCollisionsAttr(True)
    hands={};fingers={}
    for side in ['left','right']:hands[side],fingers[side]=native_hand(stage,side,paths)
    bin_task=os.environ.get('M80_BIN_TASK')=='1'
    add_roas(stage,include_supply=not bin_task)
    prepared=json.load(open(HERE/('prepared-ik-targets'+SUFFIX+'.json')))
    probe=os.environ.get('M80_PICKUP_PROBE')=='1'
    initial_path=Path(os.environ['M80_BIN_INPUT'])/'initial-robot-pose.json'if bin_task else HERE/('initial-robot-pose-pickup.json'if probe else'initial-robot-pose'+SUFFIX+'.json')
    initial_pose=json.load(open(initial_path));ready=initial_pose['ready_q'];drives={};qcurrent={}
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr(True)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)and 'official_FR3_Duo'in str(prim.GetPath()):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(.01)
        if prim.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in prim.GetName():
            name=prim.GetName();side=name.split('_')[0];index=int(name[-1])-1;value=float(ready[side][index]);qcurrent[name]=value
            PhysxSchema.JointStateAPI.Apply(prim,'angular').CreatePositionAttr(math.degrees(value));drive=UsdPhysics.DriveAPI.Apply(prim,'angular');drive.CreateTypeAttr('force');drive.CreateTargetPositionAttr(math.degrees(value));drive.CreateStiffnessAttr(12000*math.pi/180);drive.CreateDampingAttr(600*math.pi/180);drive.CreateMaxForceAttr((87 if index<4 else 12)*.95);drives[name]=drive
    assert len(drives)==14
    # New nested rigid links need explicit world transforms/reset stacks before
    # USD writeback. Preserve every body's current world pose while flattening.
    rigid=[pr for pr in stage.Traverse()if pr.HasAPI(UsdPhysics.RigidBodyAPI)]
    worlds=[UsdGeom.Xformable(pr).ComputeLocalToWorldTransform(Usd.TimeCode.Default())for pr in rigid]
    for pr,mat in zip(rigid,worlds):
        name=pr.GetName();model_name=name.replace('_NativeHand','_native_hand')
        if name.endswith('_finger'):
            model_name=name.removesuffix('_finger').replace('_left','_native_left').replace('_right','_native_right')
        record=initial_pose['body_world_poses'].get(model_name)
        if record is not None:
            q=record['quaternion'];mat=Gf.Matrix4d().SetRotate(Gf.Quatd(q[0],Gf.Vec3d(*q[1:])));mat.SetTranslateOnly(Gf.Vec3d(*record['position']))
        xf=UsdGeom.Xformable(pr);xf.ClearXformOpOrder();xf.SetResetXformStack(True);xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble,'initialWorld').Set(mat)
    physics=PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath('/PhysicsScene'));physics.CreateEnableGPUDynamicsAttr(False);physics.CreateBroadphaseTypeAttr('MBP');physics.CreateTimeStepsPerSecondAttr(480)
    if bin_task:
        from bin_arm_task import configure
        configure(stage,paths)
    stage.GetRootLayer().Export(str(OUT/'scene-authored.usda'))
    (OUT/'body-paths.json').write_text(json.dumps(dict(**paths,native_hands=hands),indent=2))
    report=dict(engine='Isaac Sim 6.1 bundled Omni PhysX 110.3.2',physics_device='cpu',gpu_devices_exposed=False,rendering_enabled=False,placement=spec,removed_old_grippers_and_placeholder_stand=removed,world_anchor_updates=fixed,native_finger_joints=4,free_workpieces=True,full_task_pass=False,cycle_time_s=None,scope='Measured articulated-robot endpoint reach trials before complete task execution',payload_collision='CAD-derived local convex sectors, not SDF',convex_piece_counts=convex_counts,palm_collision='Conservative convex hull of official native palm CAD',trials=[])
    if bin_task:
        from bin_arm_task import execute
        execute(stage,paths,hands,drives,qcurrent,initial_pose,report,OUT,HERE)
        return
    if probe:
        from native_pickup_probe import execute
        execute(stage,paths,hands,drives,qcurrent,initial_pose,report,OUT,HERE)
        return
    cache=UsdUtils.StageCache.Get();ident=cache.Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);dt=1/480;elapsed=0.
    sim.simulate(dt,elapsed);sim.fetch_results();elapsed+=dt
    import omni.physics.tensors as tensors
    view=tensors.create_simulation_view('numpy',ident);art=view.create_articulation_view('/World/RobotMountJoint');idx=np.array([0],dtype=np.uint32)
    names=list(art.shared_metatype.dof_names);linknames=list(art.shared_metatype.link_names);print('NATIVE_DOF_NAMES',names,'LINK_NAMES',linknames,flush=True)
    initial=np.array([[qcurrent[n]if n in qcurrent else .04 for n in names]],dtype=np.float32)
    print('NATIVE_ROOT_BEFORE_INIT',art.get_root_transforms().tolist(),flush=True)
    art.set_dof_positions(initial,idx);art.set_dof_velocities(np.zeros_like(initial),idx);art.set_dof_position_targets(initial,idx)
    art.set_dof_stiffnesses(np.array([[12000 if n in drives else 5000 for n in names]],dtype=np.float32),idx);art.set_dof_dampings(np.array([[600 if n in drives else 100 for n in names]],dtype=np.float32),idx);art.set_dof_max_forces(np.array([[(87 if int(n[-1])<=4 else 12)*.95 if n in drives else 30 for n in names]],dtype=np.float32),idx)
    hidx={side:linknames.index(side+'_NativeHand')for side in hands}
    contacts={}
    def contact_callback(headers,points):
        for h in headers:
            if h.num_contact_data==0:continue
            actors=[str(PhysicsSchemaTools.intToSdfPath(h.actor0)),str(PhysicsSchemaTools.intToSdfPath(h.actor1))]
            if not any('official_FR3_Duo'in x for x in actors):continue
            pair=tuple(sorted([str(PhysicsSchemaTools.intToSdfPath(h.collider0)),str(PhysicsSchemaTools.intToSdfPath(h.collider1))]));force=sum(float(np.linalg.norm(points[i].impulse))/dt for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
            if force>.05:
                old=contacts.get(pair,dict(colliders=list(pair),first_time_s=elapsed,peak_force_n=0.));old['peak_force_n']=max(old['peak_force_n'],force);contacts[pair]=old
    subscription=sim.subscribe_contact_report_events(contact_callback)
    def hand_pose(side):
        tr=art.get_link_transforms()[0,hidx[side]];x,y,z,w=tr[3:]
        R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        return tr[:3]+R@np.array([0,0,.1034]),R
    def step():
        nonlocal elapsed
        sim.simulate(dt,elapsed);sim.fetch_results();elapsed+=dt
        root=art.get_root_transforms()
        if not np.isfinite(root).all()or np.linalg.norm(root[0,:3]-np.array(spec['duo_world_position']))>.02:raise RuntimeError('Invalid articulated initialization or base drift; no trial accepted: '+str(root.tolist()))
        if not np.isfinite(art.get_dof_positions()).all()or not np.isfinite(art.get_link_transforms()).all():raise RuntimeError('Non-finite articulated state; no reach/task result accepted')
    for _ in range(240):step()
    report['initial_hand_tcp_m']={side:hand_pose(side)[0].tolist()for side in hands};report['actual_dof_names']=names;report['base_transform']=art.get_root_transforms().tolist();report['duo_collision']='Exact static CAD triangles; original mount/cover openings retained';report['initial_robot_contacts']=list(contacts.values())
    print('NATIVE_ROAS_ATTACHED',report['initial_hand_tcp_m'],flush=True)
    for target in prepared['targets']:
        side=target['side'];names=[f'{side}_arm_fr3v2_joint{i}'for i in range(1,8)];a=np.array([qcurrent[n]for n in names]);b=np.array(target['q']);T=8.
        cmd=art.get_dof_positions().copy()
        for k in range(round(T/dt)):
            u=min(1.,(k+1)*dt/T);h=u**3*(10-15*u+6*u*u)
            for name,value in zip(names,a+(b-a)*h):cmd[0,list(art.shared_metatype.dof_names).index(name)]=value
            art.set_dof_position_targets(cmd,idx)
            step()
        qcurrent.update(dict(zip(names,b.tolist())))
        for _ in range(240):step()
        tcp,R=hand_pose(side)
        poserror=float(np.linalg.norm(tcp-np.array(target['tcp'])));rot=R@np.array(target['R']).T;angle=float(math.acos(np.clip((np.trace(rot)-1)/2,-1,1)))
        actual=art.get_dof_positions()[0];actualq=np.array([actual[list(art.shared_metatype.dof_names).index(name)]for name in names])
        trial=dict(part=target['part'],stage=target['stage'],side=side,offline_ik_pass=target['offline_ik_pass'],target_tcp_m=target['tcp'],measured_tcp_m=tcp.tolist(),measured_position_error_m=poserror,measured_orientation_error_deg=math.degrees(angle),pose_reached=poserror<.0015 and angle<math.radians(1),commanded_joint_target_rad=b.tolist(),measured_joint_position_rad=actualq.tolist(),max_joint_target_error_rad=float(np.max(abs(actualq-b))),command_motion_time_s=T)
        report['trials'].append(trial);report['robot_contact_pairs']=list(contacts.values());(OUT/'physx-reach-results.json').write_text(json.dumps(report,indent=2));print('NATIVE_ROAS_REACH',json.dumps(trial),flush=True)
    report['all_required_endpoints_reached']=all(t['pose_reached']for t in report['trials'])
    report['state']='endpoint_reach_pass_full_task_pending'if report['all_required_endpoints_reached']else'endpoint_reach_failed_no_valid_cycle_time'
    stage.GetRootLayer().Export(str(OUT/'scene-final-reach.usda'));(OUT/'physx-reach-results.json').write_text(json.dumps(report,indent=2));sim.detach_stage();print('NATIVE_ROAS_PHYSX_DONE',report['state'],flush=True)

if __name__=='__main__':
    try:
        main();omni.kit.app.get_app().post_quit(0)
    except Exception:
        OUT.mkdir(exist_ok=True);(OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
