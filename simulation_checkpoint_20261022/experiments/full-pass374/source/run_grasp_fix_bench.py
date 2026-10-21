"""Native Hand bin contact bench in Isaac Sim's real CPU Omni PhysX.

Kinematic palm isolates grasp accessibility from arm reach. Fingers have real
prismatic joints, finite 30 N drive force and 95 mm/s total aperture speed.
Every part is dynamic; no weld, attach, pose reset during a trial, or teleport.
This is NOT a full-arm task, camera inference, or a measured assembly cycle.
"""
from pathlib import Path
import sys,json,math,traceback,os
import numpy as np
import omni.physx,omni.kit.app
import omni.physics.tensors as tensors
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf,UsdUtils,PhysicsSchemaTools
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from native_roas_physx import native_hand,pose,box,cpu_payload_colliders
ASSETS=HERE/'bin-picking';OUT=ASSETS/'grasp-fix';OUT.mkdir(exist_ok=True)

def rot(q):
    x,y,z,w=map(float,q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def quaternion(R):
    # Gf matrices use row vectors; numpy rotations here use column vectors.
    q=Gf.Matrix4d(Gf.Matrix3d(*np.asarray(R).T.ravel().tolist()),Gf.Vec3d(0)).ExtractRotationQuat()
    return np.array([*q.GetImaginary(),q.GetReal()])

def transform(p,R):return np.r_[p,quaternion(R)].astype(np.float32)

def rigid(stage,path,P,part,R=np.eye(3)):
    pr=UsdGeom.Xform.Define(stage,path).GetPrim();q=quaternion(R);pose(pr,P,q[[3,0,1,2]])
    UsdPhysics.RigidBodyAPI.Apply(pr);api=PhysxSchema.PhysxRigidBodyAPI.Apply(pr);api.CreateEnableCCDAttr(True);api.CreateSolverPositionIterationCountAttr(64);api.CreateSolverVelocityIterationCountAttr(16)
    spec=json.load(open(HERE.parents[1]/'generated/contact_task/contact-source.json'))['parts'][part]
    mass=UsdPhysics.MassAPI.Apply(pr);mass.CreateMassAttr(spec['mass']);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*spec['inertia']));mass.CreateCenterOfMassAttr(Gf.Vec3f(0));PhysxSchema.PhysxContactReportAPI.Apply(pr).CreateThresholdAttr(.01)
    # The exact same CAD-derived convex sectors as the full cell.
    dummy={f'workpiece_{p}':path if p==part else '/World/Unused/'+p for p in ['ring1','ring2','pin']}
    cpu_payload_colliders(stage,dummy)
    for p in ['ring1','ring2','pin']:
        if p!=part:stage.RemovePrim('/World/Unused/'+p)
    return path

def holder(stage,exposure):
    # 120 x 35 x 18 mm, three axial sockets. Sector boxes preserve the holes.
    radius=.0027;outer=.008;top=.018
    for i,x in enumerate([-.035,0,.035]):
        for k in range(32):
            a=2*math.pi*k/32;b=2*math.pi*(k+1)/32
            v=np.array([[x+r*math.cos(t),r*math.sin(t),z]for z in [0,top]for r,t in [(radius,a),(outer,a),(outer,b),(radius,b)]],np.float32)
            faces=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
            m=UsdGeom.Mesh.Define(stage,f'/World/Holder/Socket{i}_{k}');m.CreatePointsAttr(v);m.CreateFaceVertexCountsAttr([3]*len(faces));m.CreateFaceVertexIndicesAttr(faces.ravel());UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
        box(stage,f'/World/Holder/Datum{i}',[x,0,top-exposure-.001],[.012,.012,.002],(.1,.6,.2))
    # Continuous green block away from socket centers; no implicit solid cap.
    for j,y in enumerate([-.012,.012]):box(stage,f'/World/Holder/Rail{j}',[0,y,.009],[.12,.011,.018],(.1,.6,.2))
    for j,x in enumerate([-.056,-.0175,.0175,.056]):box(stage,f'/World/Holder/Web{j}',[x,0,.009],[.008,.013,.018],(.1,.6,.2))
    box(stage,'/World/Tray',[0,0,-.002],[.3,.2,.004],(.7,.7,.7))

def trial(config):
    name=config['name'];part=config['part'];pin=part=='pin'
    stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    sc=UsdPhysics.Scene.Define(stage,'/PhysicsScene');sc.CreateGravityMagnitudeAttr(9.81);sc.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));api=PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim());api.CreateEnableGPUDynamicsAttr(False);api.CreateBroadphaseTypeAttr('MBP');api.CreateSolverTypeAttr('TGS');api.CreateEnableCCDAttr(True);api.CreateTimeStepsPerSecondAttr(480)
    for label,mu in [('rubber',config.get('finger_friction',.8)),('steel',config.get('part_friction',.6))]:
        m=UsdShade.Material.Define(stage,'/World/Materials/'+label);ma=UsdPhysics.MaterialAPI.Apply(m.GetPrim());ma.CreateStaticFrictionAttr(mu);ma.CreateDynamicFrictionAttr(mu*.85);ma.CreateRestitutionAttr(0)
    bin_arrays=np.load(ASSETS/'existing-baskets.npz');key='instance_0670'
    if not pin:
        m=UsdGeom.Mesh.Define(stage,'/World/ExistingBasket');m.CreatePointsAttr(bin_arrays[key+'_v'].astype(np.float32));f=bin_arrays[key+'_f'];m.CreateFaceVertexCountsAttr([3]*len(f));m.CreateFaceVertexIndicesAttr(f.ravel());UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('none')
    else:holder(stage,config.get('insertion_depth',.009))
    base=UsdGeom.Xform.Define(stage,'/World/Base').GetPrim();pose(base,[0,0,.4]);UsdPhysics.RigidBodyAPI.Apply(base).CreateKinematicEnabledAttr(True)
    hand,joints=native_hand(stage,'right',dict(right_arm_fr3v2_link8='/World/Base',right_arm_fr3v2_link7='/World/Base'))
    stage.RemovePrim('/World/NativeJoints/right_native_mount')
    asset=np.load(HERE/'native-official-finger.npz')
    for label in ['left','right']:
        path=hand+'/right_'+label+'_finger'
        for mesh_name in ['mount','sledge','diagonal','tip']:stage.RemovePrim(path+'/'+mesh_name)
        m=UsdGeom.Mesh.Define(stage,path+'/tip');m.CreatePointsAttr(asset['vertices'].astype(np.float32));f=asset['faces'];m.CreateFaceVertexCountsAttr([3]*len(f));m.CreateFaceVertexIndicesAttr(f.ravel());m.CreateSubdivisionSchemeAttr('none');UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(UsdShade.Material.Get(stage,'/World/Materials/rubber'),UsdShade.Tokens.strongerThanDescendants,'physics');pc=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());pc.CreateContactOffsetAttr(.00005);pc.CreateRestOffsetAttr(0.);pc.CreateTorsionalPatchRadiusAttr(.003);pc.CreateMinTorsionalPatchRadiusAttr(.002)

    # A floating articulation keeps the small finger inertias stable. The palm
    # root is prescribed at each time step, without changing any part pose.
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(hand)).CreateKinematicEnabledAttr(False)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(hand))
    PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(hand)).CreateEnabledSelfCollisionsAttr(True)
    # Fixed hand pose for parse; finger world transforms match the initial pose.
    R=np.diag([1.,-1.,-1.]);angle=math.radians(config.get('yaw_deg',0));rz=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1.]])
    R=rz@R;opening=config.get('opening',.080 if not pin else .022)
    startP=np.array(config.get('position',[0,0,.008 if part=='ring1'else .0073]))
    if pin:startP=np.array([0,0,.018-config.get('insertion_depth',.009)+.01775])
    paths={part:rigid(stage,'/World/Selected',startP,part)}
    for i,other in enumerate(config.get('others',[])):
        paths['other'+str(i)]=rigid(stage,'/World/Other'+str(i),other['P'],other.get('part',part))
    hand_origin=startP+[0,0,.20]-R@np.array([0,0,.1034]);q=quaternion(R)
    hp=stage.GetPrimAtPath(hand);xf=UsdGeom.Xformable(hp);xf.ClearXformOpOrder();xf.SetResetXformStack(True);pose(hp,hand_origin,q[[3,0,1,2]])
    for sign,label in [(1,'left'),(-1,'right')]:
        fp=stage.GetPrimAtPath(hand+'/right_'+label+'_finger');fq=np.eye(3)if sign==1 else np.diag([-1.,-1.,1.]);fq=R@fq;xf=UsdGeom.Xformable(fp);xf.ClearXformOpOrder();xf.SetResetXformStack(True);qq=quaternion(fq);pose(fp,hand_origin+R@np.array([0,sign*opening/2,.0584]),qq[[3,0,1,2]])
        PhysxSchema.PhysxContactReportAPI.Apply(fp).CreateThresholdAttr(.01)
    PhysxSchema.PhysxContactReportAPI.Apply(hp).CreateThresholdAttr(.01)
    for joint in joints:UsdPhysics.DriveAPI(joint.GetPrim(),'linear').CreateTargetPositionAttr(opening/2)
    stage.GetRootLayer().Export(str(OUT/(name+'-initial.usda')))
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);dt=1/480;t=0.;phase='settle';contacts={};trace=[];current={}
    def callback(headers,data):
        for h in headers:
            if not h.num_contact_data:continue
            a=[str(PhysicsSchemaTools.intToSdfPath(v))for v in [h.actor0,h.actor1]]
            if not any('NativeHand'in x for x in a):continue
            c=[str(PhysicsSchemaTools.intToSdfPath(v))for v in [h.collider0,h.collider1]];force=sum(float(np.linalg.norm(data[i].impulse))/dt for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
            if force<.05:continue
            k=tuple(sorted(c));row=contacts.setdefault(k,dict(actors=a,colliders=c,peak_force_n=0.,phases=[]));row['peak_force_n']=max(row['peak_force_n'],force)
            if phase not in row['phases']:row['phases'].append(phase)
            current[k]=(a,force)
    subscription=sim.subscribe_contact_report_events(callback)
    sim.simulate(dt,t);sim.fetch_results();t+=dt
    view=tensors.create_simulation_view('numpy',ident);art=view.create_articulation_view(hand);hv=view.create_rigid_body_view(hand);pv={p:view.create_rigid_body_view(path)for p,path in paths.items()};fv={label:view.create_rigid_body_view(hand+'/right_'+label+'_finger')for label in ['left','right']};idx=np.array([0],np.uint32)
    finger_cmd=np.array([[opening/2,opening/2]],np.float32);art.set_dof_positions(finger_cmd,idx);art.set_dof_velocities(np.zeros_like(finger_cmd),idx);art.set_dof_position_targets(finger_cmd,idx)
    palm_command=transform(hand_origin,R)[None]
    previous_palm=palm_command.copy()
    def target_palm(value):palm_command[:]=value
    def step():
        nonlocal t
        current.clear();art.set_root_transforms(previous_palm,idx)
        velocity=np.zeros((1,6),np.float32);velocity[0,:3]=(palm_command[0,:3]-previous_palm[0,:3])/dt
        art.set_root_velocities(velocity,idx);art.set_dof_position_targets(finger_cmd,idx);sim.simulate(dt,t);sim.fetch_results();t+=dt;previous_palm[:]=palm_command
        if not np.isfinite(art.get_link_transforms()).all()or np.max(abs(art.get_link_transforms()[0,:,:3]))>3:raise RuntimeError('Invalid finger/palm dynamics; no pickup claim')
        if round(t/dt)%8==0:trace.append(dict(t=t,phase=phase,hand=hv.get_transforms()[0].copy(),fingers=art.get_dof_positions()[0].copy(),parts=np.array([v.get_transforms()[0]for v in pv.values()])))
    def wait(seconds,label):
        nonlocal phase
        phase=label
        for _ in range(round(seconds/dt)):step()
    wait(.7,'settle')
    settled={p:v.get_transforms()[0].copy()for p,v in pv.items()};target=settled[part].copy()
    # Oracle pose is an explicit isolation of the physical grasp bottleneck.
    # Ring roll/pitch measured after free settling is respected.
    Rp=rot(target[3:]);normal=Rp[:,2]
    if normal[2]<0:normal=-normal
    if not pin:
        zaxis=-normal;xaxis=np.array([math.cos(angle),math.sin(angle),0.]);xaxis-=zaxis*np.dot(xaxis,zaxis);xaxis/=np.linalg.norm(xaxis);yaxis=np.cross(zaxis,xaxis);R=np.column_stack([xaxis,yaxis,zaxis])
    if 'localR'in config:R=Rp@np.array(config['localR'])
    tcp=target[:3]+[0,0,config.get('grasp_z_offset',.003 if not pin else .006)]
    if 'localP'in config:tcp=target[:3]+Rp@np.array(config['localP'])
    goal=tcp-R@np.array([0,0,.1034]);start=hv.get_transforms()[0,:3].copy();q=quaternion(R)
    phase='approach'
    for k in range(960):
        u=(k+1)/960;h=u**3*(10-15*u+6*u*u);target_palm(transform(start+(goal-start)*h,R)[None]);step()
    wait(.2,'approach_hold');before=pv[part].get_transforms()[0].copy();phase='close'
    gap=config.get('gap',.075 if part=='ring1'else .062 if part=='ring2'else config.get('pin_gap',.0097));finalgap=gap-config.get('compression',.008)
    duration=(opening-finalgap)/.095+.2
    for k in range(math.ceil(duration/dt)):
        width=max(finalgap,opening-.095*(k+1)*dt)
        finger_cmd[:]=width/2
        for joint in joints:UsdPhysics.DriveAPI(joint.GetPrim(),'linear').CreateTargetPositionAttr(width/2)
        step()
    wait(.4,'grasp_hold');grasp=pv[part].get_transforms()[0].copy();hgrasp=hv.get_transforms()[0].copy();relative=rot(hgrasp[3:]).T@(grasp[:3]-hgrasp[:3]);relativeR=rot(hgrasp[3:]).T@rot(grasp[3:])
    print('BIN_FINGER_DIAGNOSTIC',name,'hand',hgrasp.tolist(),'fingers',{k:v.get_transforms()[0].tolist()for k,v in fv.items()},'drive',[UsdPhysics.DriveAPI(j.GetPrim(),'linear').GetTargetPositionAttr().Get()for j in joints],flush=True)
    phase='lift';lift=.160
    for k in range(384):
        u=(k+1)/384;h=u**3*(10-15*u+6*u*u);target_palm(transform(goal+[0,0,lift*h],R)[None]);step()
    wait(.7,'lift_hold');final=pv[part].get_transforms()[0];hf=hv.get_transforms()[0];slip=float(np.linalg.norm(rot(hf[3:]).T@(final[:3]-hf[:3])-relative));rr=rot(hf[3:]).T@rot(final[3:])@relativeR.T;angular=math.degrees(math.acos(np.clip((np.trace(rr)-1)/2,-1,1)))
    intended=[c for c in contacts.values()if '/World/Selected'in c['actors']and any(a.endswith('_finger')for a in c['actors'])];fingers={a for c in intended for a in c['actors']if a.endswith('_finger')}
    bad=[c for c in contacts.values()if not('/World/Selected'in c['actors']and any(a.endswith('_finger')for a in c['actors']))and c['peak_force_n']>3]
    others={p:float(pv[p].get_transforms()[0,2]-tr[2])for p,tr in settled.items()if p!=part};rise=float(final[2]-before[2]);result=dict(config=config,passed=rise>.150 and slip<.003 and angular<3 and len(fingers)==2 and not bad and all(v<.020 for v in others.values()),selected_rise_m=rise,relative_slip_m=slip,relative_rotation_slip_deg=angular,bilateral_finger_contact=len(fingers)==2,unexpected_contacts=bad,other_part_rises_m=others,contacts=list(contacts.values()),settled_transforms={p:v.tolist()for p,v in settled.items()},grasp_transform=grasp.tolist(),final_transform=final.tolist(),grasp_tcp_m=tcp.tolist(),grasp_R=R.tolist(),grasp_pose_source='simulation_ground_truth_ISOLATED_CONTACT_DIAGNOSTIC',full_arm_simulated=False,vision_tested=False,full_task_pass=False,diagnostic_time_s=t)
    np.savez_compressed(OUT/(name+'-trace.npz'),time_s=[x['t']for x in trace],phase=[x['phase']for x in trace],hand=[x['hand']for x in trace],fingers=[x['fingers']for x in trace],parts=[x['parts']for x in trace],part_names=list(paths))
    (OUT/(name+'-result.json')).write_text(json.dumps(result,indent=2));sim.detach_stage();UsdUtils.StageCache.Get().Erase(Usd.StageCache.Id.FromLongInt(ident))
    print('BIN_CONTACT_RESULT',name,result['passed'],rise,slip,angular,'BAD',[(c['actors'],c['peak_force_n'])for c in bad],flush=True)
    return result

def main():
    cases=[dict(name=p+'-flat-center',part=p)for p in ['ring1','ring2']]
    cases += [dict(name=p+'-near-wall',part=p,position=[.075,0,.008 if p=='ring1'else .0073],yaw_deg=90)for p in ['ring1','ring2']]
    cases += [dict(name=p+'-overlap-top',part=p,position=[.025,0,.030],others=[dict(P=[-.025,0,.008],part=p)])for p in ['ring1','ring2']]
    cases += [dict(name='pin-depth'+str(int(d*1000)),part='pin',insertion_depth=d)for d in [.006,.009,.012]]
    if (OUT/'bench-cases.json').exists():cases=json.load(open(OUT/'bench-cases.json'))
    report=dict(engine='Isaac Sim 6.1 bundled Omni PhysX CPU',basket_geometry='Original CAD instance_0670 exact static triangle mesh, normalized floor frame',palm_motion='Prescribed kinematic; isolated contact bench, not full robot',workpieces_free=True,part_attachments=False,hardware_tested=False,vision_tested=False,full_task_pass=False,cycle_time_s=None,trials=[])
    for c in cases:
        report['trials'].append(trial(c));(OUT/'contact-suite.json').write_text(json.dumps(report,indent=2))
    print('BIN_CONTACT_SUITE_DONE',sum(r['passed']for r in report['trials']),len(cases),flush=True)

if __name__=='__main__':
    try:main();omni.kit.app.get_app().post_quit(0)
    except Exception:(OUT/'contact-error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
