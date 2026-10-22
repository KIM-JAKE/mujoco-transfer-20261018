"""Render recorded Isaac states using Isaac PhysX CPU cameras; no new test.

All replay bodies are kinematic and joints are disabled. Replay poses are not
assembly evidence: the source physical trial and its report supply that.
"""
from pathlib import Path
import os,sys,json,traceback,time,re
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdUtils
import omni.kit.app,omni.physx
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));PORT=HERE.parents[1];OUT=PORT/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
from fast_physx_camera import render
try:
    request=json.load(open(HERE/'bin-picking/video-request.json'));source=PORT/'runs'/request['source_run'];resultfile=source/'physx-pickup-results.json';result=json.load(open(resultfile if resultfile.exists()else source/'pipeline-results.json'));paths=json.load(open(source/'body-paths.json'));stage=Usd.Stage.Open(str(source/'scene-authored.usda'));seqfile=source/'physx-sequence-trace.npz';pickupfile=source/'physx-pickup-trace.npz';pickup=np.load(pickupfile)if pickupfile.exists()else None;seq=np.load(seqfile)if seqfile.exists()else None;linknames=result['link_names'];primnames={p.GetName():str(p.GetPath())for p in stage.Traverse()if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    if request.get('require_full_task_pass')and not result.get('full_task_pass'):raise RuntimeError('Video requires a passed fresh physical full-task report')
    if request.get('visual_meshes'):
        from quality_physx_camera import render
    width=int(request.get('render_width',320));height=int(request.get('render_height',240))
    def set_pose(path,value):
        prim=stage.GetPrimAtPath(path)
        if not prim:return
        x,y,z,w=map(float,value[3:]);xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();xf.SetResetXformStack(True);M=Gf.Matrix4d().SetRotate(Gf.Quatd(w,Gf.Vec3d(x,y,z)));M.SetTranslateOnly(Gf.Vec3d(*map(float,value[:3])));xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(M)
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdPhysics.Joint):prim.SetActive(False);continue
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True);PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
        if request.get('visual_meshes'):
            if prim.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
            if prim.IsA(UsdGeom.Mesh)and UsdGeom.Mesh(prim).ComputeVisibility()!='invisible'and 'PlanningQueries'not in str(prim.GetPath()):
                UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True);UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
    clamps=stage.GetPrimAtPath('/World/Clamps')
    if clamps:clamps.SetActive(False)
    for row in result.get('clutter_settled_objects',[]):set_pose(row['path'],row['transform'])
    def camera(name,eye,target,focal):
        c=UsdGeom.Camera.Define(stage,'/World/Cameras/'+name);c.CreateFocalLengthAttr(focal);c.CreateHorizontalApertureAttr(32);xf=UsdGeom.Xformable(c);xf.ClearXformOpOrder();xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse());return str(c.GetPath())
    globalcam=camera('DemoGlobal',[-1.50,-1.85,1.95],[.0,-.18,1.18],35);closecam=camera('DemoCloseup',[-.5,-.6,1.7],[0,0,1.3],38)
    cache=UsdUtils.StageCache.Get();ident=cache.Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);sim.simulate(1/60,0);sim.fetch_results();frames=[]
    if request.get('batch_camera'):
        import omni.physics.tensors as tensor_api
        from rgbd_batch_sensor import BatchRGBDCamera
        sensor_view=tensor_api.create_simulation_view('numpy',ident);moving={primnames[n]for n in linknames if n in primnames}|{paths['workpiece_'+p]for p in ['ring1','ring2','pin']};batch=BatchRGBDCamera(stage,sensor_view,moving_paths=moving)
        def render(stage,camera_path,width,height):
            c=UsdGeom.Camera(stage.GetPrimAtPath(camera_path));T=np.array(UsdGeom.Xformable(c).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T;eye=T[:3,3];target=eye-T[:3,2];span=float(c.GetHorizontalApertureAttr().Get()/c.GetFocalLengthAttr().Get());return batch.capture(eye,target,span,width,height,far_clip=5.)['rgb']
    sources=([(pickup,request.get('pickup_frames',8),'pickup')]if pickup is not None else[])+([(seq,request.get('sequence_frames',28),'sequence')]if seq is not None else[])
    sequence=result.get('sequence')or(json.load(open(source/'physx-sequence-results.json'))if(source/'physx-sequence-results.json').exists()else{});records=sequence.get('records',[]);firstpart=next(iter(result.get('parts',{'ring1':None})))
    for archive,count,kind in sources:
        end=len(archive['time_s'])-1
        if kind=='pickup' and seq is not None:end=max(0,int(np.searchsorted(archive['time_s'],seq['time_s'][0]))-1)
        selected=np.unique(np.linspace(0,end,count).astype(int))
        if kind=='sequence' and request.get('sequence_windows'):
            selected=np.unique(np.concatenate([np.linspace(np.searchsorted(archive['time_s'],w['start_s']),min(end,np.searchsorted(archive['time_s'],w['end_s'])),w['frames']).astype(int)for w in request['sequence_windows']]))
        shard=re.search(r'-s(\d+)of(\d+)$',OUT.name)
        if shard:selected=np.array_split(selected,int(shard.group(2)))[int(shard.group(1))]
        for j in selected:
            t=float(archive['time_s'][j]);active=firstpart;event='pickup'
            if kind=='sequence':
                event=next((r['event']for r in records if r['time_s']>=t),records[-1]['event']if records else'sequence');active=next((p for p in ['ring1','ring2','pin']if event.startswith(p)),firstpart)
                if not records and result.get('final_checks'):
                    goals=json.load(open(source/'arm-plan.json'))['goals'];names=archive['part_names'].tolist()
                    def released_at_target(part):
                        value=archive['parts'][j,names.index(part)];roots=[archive['links'][j,linknames.index(side+'_NativeHand'),:3]for side in ['left','right']]
                        axis_error=np.arccos(np.clip(1-2*(value[3]**2+value[4]**2),-1,1))
                        return np.linalg.norm(value[:3]-goals[part]['P'])<.003 and axis_error<.05 and min(np.linalg.norm(root-value[:3])for root in roots)>.16
                    if released_at_target('ring1'):active='ring2'
                    if released_at_target('ring1')and released_at_target('ring2'):active='pin'
                    event=active+'_continuous_task'
            for name,value in zip(linknames,archive['links'][j]):
                if name in primnames:set_pose(primnames[name],value)
            names=archive['part_names'].tolist()if kind=='sequence'else list(result['parts'])
            for p,value in zip(names,archive['parts'][j]):set_pose(paths['workpiece_'+p],value)
            index=names.index(active)if active in names else 0;target=archive['parts'][j,index,:3].astype(float)
            if request.get('inspection_views'):
                if target[1]>.05:camera('DemoCloseup',[-.22,-.35,1.40],[-.22,.25,1.535],50)
                elif target[1]<-.50:camera('DemoCloseup',(target+[-.18,-.20,.65]).tolist(),target.tolist(),70 if active!='pin'else 100)
                else:camera('DemoCloseup',(target+[.22,-.35,.30]).tolist(),target.tolist(),65)
            else:
                eye_offset=[.35,-.50,.08]if target[1]>.05 else[-.42,-.50,.35];camera('DemoCloseup',(target+eye_offset).tolist(),target.tolist(),38)
            sim.flush_changes();sim.simulate(1/60,len(frames)/60);sim.fetch_results();left=render(stage,globalcam,width,height);right=render(stage,closecam,width,height);rgb=np.concatenate([left,right],axis=1);file=OUT/f'frame_{len(frames):04d}.ppm'
            with file.open('wb')as fp:fp.write(f'P6\n{width*2} {height}\n255\n'.encode()+rgb.tobytes())
            frames.append(dict(file=file.name,sim_time_s=t,event=event,active_part=active));print('ISAAC_TRACE_FRAME',len(frames),round(t,3),event,flush=True)
    report=dict(source_run=request['source_run'],source_physical_assembly_pass=result.get('sequence',{}).get('final_assembly_pass',result.get('full_task_pass',False)),source_full_task_pass=result.get('full_task_pass',False),manual_restock_count=len(result.get('sequence',{}).get('trial_resets',[])),renderer='Isaac USD/PhysX CPU batch ray camera'if request.get('batch_camera')else'Isaac PhysX CPU collision-surface cameras',RTX=False,replay_only=True,kinematic_replay=True,synthetic_grasp_or_assembly_success=False,extra_proxy_clamps_omitted_from_drawing=True,video_time_compressed=request.get('playback_speed',10)!=1,colors=dict(ring1='steel',ring2='steel',pin='steel'),frames=frames)
    (OUT/'video-frames.json').write_text(json.dumps(report,indent=2));sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:traceback.print_exc();omni.kit.app.get_app().post_quit(1)
