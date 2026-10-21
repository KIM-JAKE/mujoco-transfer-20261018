"""Configure the original M80 cell for native-Hand basket pickup in PhysX."""
from pathlib import Path
import json,math,os
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf,Sdf,PhysxSchema
HERE=Path(__file__).resolve().parent

def configure(stage,paths):
    from native_roas_physx import pose
    plan=json.load(open(Path(os.environ['M80_BIN_INPUT'])/'arm-plan.json'))
    if plan.get('simulation_hz'):PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath('/PhysicsScene')).CreateTimeStepsPerSecondAttr(int(plan['simulation_hz']))
    for name,eye,target in [('BasketCamera',[.75,-1.25,1.65],[.50,-.725,1.10]),('GlobalCamera',[-.25,-2.2,2.1],[0,0,1.25])]:
        camera=UsdGeom.Camera.Define(stage,'/World/Cameras/'+name);camera.CreateFocalLengthAttr(24);camera.CreateHorizontalApertureAttr(32)
        if name=='BasketCamera':camera.CreateFocalLengthAttr(48)
        view=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,1,0));UsdGeom.Xformable(camera).AddTransformOp().Set(view.GetInverse())
    for name,path in paths.items():
        if name=='cap_supply_fixture'or name.startswith('cap_stock_'):stage.GetPrimAtPath(path).SetActive(False)
    for root in ['/World/Environment/cap_supply_fixture','/World/Environment/pin_socket_floor']:
        p=stage.GetPrimAtPath(root)
        if p:p.SetActive(False)
    for p in list(stage.Traverse()):
        if p.IsValid()and str(p.GetPath()).startswith('/World/Environment/cap_stock_'):p.SetActive(False)
    scene_parts=plan.get('scene_parts',plan['parts'])
    for part in ['ring1','ring2','pin']:
        p=stage.GetPrimAtPath(paths['workpiece_'+part])
        if part not in scene_parts:p.SetActive(False);continue
        row=scene_parts[part];R=np.array(row['source_R']);q=Gf.Matrix4d(Gf.Matrix3d(*R.T.ravel().tolist()),Gf.Vec3d(0)).ExtractRotationQuat();pose(p,row['source_P'],[q.GetReal(),*q.GetImaginary()]);UsdGeom.Xformable(p).SetResetXformStack(True)
    if plan.get('proposed_pin_holder'):
        frozen_holder=Path(os.environ['M80_BIN_INPUT'])/'three-pin-holder-mesh.npz';a=np.load(frozen_holder if frozen_holder.exists()else HERE/'bin-picking/three-pin-holder-mesh.npz');m=UsdGeom.Mesh.Define(stage,'/World/ProposedThreePinHolder');m.CreatePointsAttr(a['v']);f=a['f'];m.CreateFaceVertexCountsAttr([3]*len(f));m.CreateFaceVertexIndicesAttr(f.ravel());m.CreateDisplayColorAttr([Gf.Vec3f(.1,.65,.2)]);pose(m.GetPrim(),[.170,-.8075,.8843]);UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('none')
        yaw=math.radians(plan.get('pin_holder_yaw_deg',0));holderR=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1.]]);holderq=Gf.Matrix4d(Gf.Matrix3d(*holderR.T.ravel().tolist()),Gf.Vec3d(0)).ExtractRotationQuat();pose(m.GetPrim(),plan.get('pin_holder_origin_m',[.170,-.8075,.8843]),[holderq.GetReal(),*holderq.GetImaginary()])
        for i,dx in ([]if plan.get('pin_count')==1 else[(1,-.035),(2,.035)]):
            extra=UsdGeom.Xform.Define(stage,f'/World/SparePin{i}').GetPrim();extra.GetReferences().AddInternalReference(paths['workpiece_pin']);R=np.array(scene_parts['pin']['source_R']);q=Gf.Matrix4d(Gf.Matrix3d(*R.T.ravel().tolist()),Gf.Vec3d(0)).ExtractRotationQuat();pose(extra,np.array(scene_parts['pin']['source_P'])+[dx,0,0],[q.GetReal(),*q.GetImaginary()]);UsdGeom.Xformable(extra).SetResetXformStack(True)
            pose(extra,np.array(scene_parts['pin']['source_P'])+holderR@np.array([dx,0,0]),[q.GetReal(),*q.GetImaginary()])
    if plan.get('continue_sequence'):
        from fixture_clamps import apply
        rows,meta=apply(stage,paths,False);stage.GetPrimAtPath('/World/Environment/PinStockSleeve').SetActive(False)
    if plan.get('clutter_items'):
        stock_paths={item['id']:(paths['workpiece_'+item['part']]if item.get('main')else'/World/BinStock/'+item['id'])for item in plan['clutter_items']}
        (Path(os.environ['M80_BIN_INPUT'])/'clutter-body-paths.json').write_text(json.dumps(stock_paths,indent=2))
        for item in plan['clutter_items']:
            if item.get('main'):continue
            path='/World/BinStock/'+item['id'];extra=UsdGeom.Xform.Define(stage,path).GetPrim();extra.GetReferences().AddInternalReference(paths['workpiece_'+item['part']]);R=np.array(item['source_R']);q=Gf.Matrix4d(Gf.Matrix3d(*R.T.ravel().tolist()),Gf.Vec3d(0)).ExtractRotationQuat();pose(extra,item['source_P'],[q.GetReal(),*q.GetImaginary()]);UsdGeom.Xformable(extra).SetResetXformStack(True)
        for root in stock_paths.values():
            for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
                if prim.IsA(UsdGeom.Mesh):UsdGeom.Mesh(prim).CreateDisplayColorAttr([Gf.Vec3f(.58,.60,.63)])

def execute(stage,paths,hands,drives,qcurrent,initial_pose,report,OUT,HERE):
    # Reuse the measured articulated-robot driver, with an explicit plan input.
    from native_pickup_probe import execute as probe
    plan_path=Path(os.environ['M80_BIN_INPUT'])/'arm-plan.json'
    plan=json.load(open(plan_path))
    if plan.get('adapt_grasp')and(not plan.get('clutter_items')or plan.get('fixed_demo_grasp')):
        from isaac_kinematics import Arm
        report['kinematics_engine']='Isaac USD joint frames; numpy inverse kinematics inside Isaac Sim'
        for part,r in plan['parts'].items():
            side=r['side'];kin=Arm(stage,side,drives,initial_pose['ready_q'][side]);report.setdefault('USD_kinematics_validation',{})[side]=kin.validation
            adapted=plan['adapt_grasp'];Rp=np.array(r['source_R']);P=np.array(r['source_P']);R=Rp@np.array(adapted['localR']);lp=np.array(adapted['localP']);tcp=P+Rp@lp
            seed=np.array(initial_pose['ready_q'][side]);pre=kin.solve(tcp+[0,0,plan['lift_m']],R,seed)
            if pre is None:
                rng=np.random.default_rng(801)
                for candidate in [np.array([0,-.4,0,-1.8,0,1.7,.8])]+[rng.uniform(kin.bounds[:,0]+.02,kin.bounds[:,1]-.02)for _ in range(16)]:
                    pre=kin.solve(tcp+[0,0,plan['lift_m']],R,candidate)
                    if pre is not None:break
            if pre is None:raise RuntimeError('Isaac USD IK cannot reach the new grasp approach')
            if part=='pin':
                # Scene queries must run after PhysX attaches and settles.
                plan['pin_approach_requires_query']=True
                plan['initial_transition_q']=[seed.tolist(),pre.tolist()]
            descend=[pre]
            for h in np.linspace(plan['lift_m'],0,74)[1:]:
                q=kin.solve(tcp+[0,0,h],R,descend[-1])
                if q is None:raise RuntimeError('Isaac USD IK cannot complete the new grasp descent')
                descend.append(q)
            r['recipe'].update(localP=adapted['localP'],localR=adapted['localR'],gap=adapted['gap']);r['cartesian_lift_q']=[q.tolist()for q in reversed(descend)];plan['initial_apertures_m'][side]=adapted['opening'];plan['initial_arm_transition']=True
        plan_path=OUT/'isaac-generated-grasp-plan.json';plan_path.write_text(json.dumps(plan,indent=2))
    report['task_environment']='Original M80, original CAD basket positions, ROAS cabinet, no added supply table or source rods'
    probe(stage,paths,hands,drives,qcurrent,initial_pose,report,OUT,HERE,plan_path=plan_path)

def select_clutter_grasp(stage,paths,view,art,aidx,drives,plan,report,OUT):
    """Ideal-pose physical feasibility selection; never a camera-policy claim."""
    from isaac_kinematics import Arm,exp,log
    from isaac_collision_queries import Checker
    from native_pickup_probe import rotation
    candidates=[];snapshots=[]
    mapping_file=OUT/'clutter-body-paths.json';mapping=json.load(open(mapping_file))if mapping_file.exists()else{}
    from basket_size import measure_stage
    measured=measure_stage(stage,list(mapping.values()))
    size_labels={row['path']:row['part']for row in measured}
    report['ring_size_classification']=measured
    report['ring_selection_uses_color']=False
    for item in plan['clutter_items']:
        path=mapping.get(item['id'],paths['workpiece_'+item['part']]if item.get('main')else'/World/BinStock/'+item['id']);v=view.create_rigid_body_view(path);t=v.get_transforms()[0];R=rotation(t[3:]);record=dict(id=item['id'],part=item['part'],path=path,transform=t.tolist());snapshots.append(record)
        if size_labels.get(path)==next(iter(plan['parts']))and R[2,2]>.7:candidates.append((float(t[2]),path,t,R))
    initial=not report.get('initial_mixed_scene_validated',False)
    report.update(clutter_count=len(snapshots),clutter_counts=plan['clutter_counts'],perception_mode='Ideal Isaac body-pose candidate selection for physical feasibility; not a validated vision policy',vision_end_to_end_pass=False)
    if initial:report['clutter_settled_objects']=snapshots
    else:report.setdefault('clutter_observation_updates',[]).append(snapshots)
    basket=json.load(open(HERE/'bin-picking/asset-audit.json'))['baskets'][0];O=np.array(basket['floor_origin_world_m']);B=np.array(basket['basket_to_world_R']);inside=[]
    for row in snapshots:
        local=B.T@(np.array(row['transform'][:3])-O);inside.append(abs(local[0])<.121 and abs(local[1])<.191 and -.01<local[2]<.15)
    if initial:
        report['all_ten_rings_inside_original_basket_after_settling']=all(inside)and len(inside)==10
        if not report['all_ten_rings_inside_original_basket_after_settling']:raise RuntimeError('The ten-ring clutter spawn did not settle inside the original basket')
        report['initial_mixed_scene_validated']=True
    candidates=[row for row in candidates if abs((B.T@(row[2][:3]-O))[0])<.121 and abs((B.T@(row[2][:3]-O))[1])<.191]
    side='right';kin=Arm(stage,side,drives,art.get_dof_positions()[0,aidx[side]]);report.setdefault('USD_kinematics_validation',{})[side]=kin.validation
    checker=Checker(stage,kin,side);startq=art.get_dof_positions()[0,aidx[side]].astype(float);valid,why=checker.clear(startq,.08)
    report['Isaac_scene_query_initial_validation']=dict(clear=valid,why=why)
    if not valid:raise RuntimeError('Isaac collision query disagrees with the parked robot: '+str(why))
    attempts=[]
    for _,path,t,Rp in sorted(candidates,reverse=True,key=lambda row:row[0]-.35*row[2][0]):
        P=t[:3];n=Rp[:,2];n/=np.linalg.norm(n);base=np.array([1.,0,0]);base-=n*(base@n);base/=np.linalg.norm(base);other=np.cross(n,base)
        folded=np.array([0.,-.5,0.,-2.,0.,2.,.8]);sideways=np.array([1.,.3,-1.,-2.,1.,2.,0.]);options=[(a,0,seed)for seed in [startq,folded,sideways]for a in [0,math.pi,.26,-.26]]
        for angle,tilt,seed in options:
            part=next(iter(plan['parts']));rim=.034 if part=='ring1'else .02735
            x=np.array([math.cos(angle),math.sin(angle),0]);down=np.array([0.,0.,-1.]);R=np.column_stack([x,np.cross(down,x),down]);offset=.014 if part=='ring1'else .007;tcp=P+n*.007+np.array([0,np.clip(O[1]-P[1],-offset,offset),0]);rise=max(.085,1.265-float(P[2]));pre=kin.solve(tcp+[0,0,rise],R,seed);opening=.001
            if pre is None:attempts.append(dict(path=path,angle=angle,stage='pregrasp_IK'));continue
            valid,why=checker.clear(pre,opening,path)
            if not valid:attempts.append(dict(path=path,angle=angle,stage='pregrasp_collision',why=why));continue
            descending=[pre]
            for h in np.linspace(rise,0,75)[1:]:
                q=kin.solve(tcp+[0,0,h],R,descending[-1])
                if q is None:break
                descending.append(q)
            if len(descending)!=75:attempts.append(dict(path=path,angle=angle,stage='descent_IK'));continue
            valid,why=checker.path(descending,opening,path)
            if not valid:attempts.append(dict(path=path,angle=angle,stage='descent_collision',why=why));continue
            transition=[startq,pre];valid,why=checker.path(transition,opening,path)
            if not valid:
                first=kin.fk(startq);last=kin.fk(pre);a=first[:3,3];Ra=first[:3,:3];rv=log(last[:3,:3]@Ra.T);transition=[startq]
                for u in np.linspace(0,1,max(10,int(np.linalg.norm(last[:3,3]-a)/.008)+1))[1:]:
                    q=kin.solve(a+(last[:3,3]-a)*u,exp(rv*u)@Ra,transition[-1])
                    if q is None:break
                    transition.append(q)
                valid=np.linalg.norm(kin.fk(transition[-1])[:3,3]-last[:3,3])<.0001
                if valid:valid,why=checker.path(transition,opening,path)
                if not valid:attempts.append(dict(path=path,angle=angle,stage='approach_collision',why=why));continue
            part=next(iter(plan['parts']));r=plan['parts'][part];radius=.06337/2 if part=='ring1'else .05051/2;gap=2*(math.sqrt(radius**2-.00875**2)-.01518);r.update(source_P=P.tolist(),source_R=Rp.tolist(),cartesian_lift_q=[q.tolist()for q in reversed(descending)]);r['recipe'].update(localP=(Rp.T@(tcp-P)).tolist(),localR=(Rp.T@R).tolist(),gap=gap,grasp_mode='internal_expansion');plan.update(lift_m=rise,minimum_lift_m=.05,initial_arm_transition=True,initial_transition_q=[q.tolist()for q in transition]);plan['initial_apertures_m'][side]=opening;report['clutter_selection']=dict(selected_path=path,part=part,actual_part_pose=t.tolist(),grasp_mode='internal_expansion',attempts=attempts,Isaac_PhysX_queries_screened=True)
            print('ISAAC_CLUTTER_SELECTED',path,'rise',rise,flush=True);return{part:dict(path=path)}
    (OUT/'clutter-selection-failure.json').write_text(json.dumps(dict(attempts=attempts,settled_objects=snapshots),indent=2));raise RuntimeError('No reachable exposed ring grasp in the mixed ten-ring scene')
