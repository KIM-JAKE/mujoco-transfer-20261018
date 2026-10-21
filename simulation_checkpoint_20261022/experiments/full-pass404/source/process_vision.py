"""Nonblocking sensor RPC; inference and image formation run outside the GIL."""
from pathlib import Path
import subprocess,socket,time,numpy as np,json,os,builtins,hashlib
from concurrent.futures import Future
from multiprocessing.connection import Connection
class CachedGeometryCamera:
    """Reuse fixed CAD triangles, never images, masks or live object transforms."""
    def __init__(self,view,model,stage=None,include_visible_planes=False):
        start=time.perf_counter();self.model=Path(model);self.visual_planes=[]
        if include_visible_planes:
            if stage is None:raise ValueError('Authored visual Plane export needs the current Isaac stage')
            from pxr import UsdGeom,Usd,UsdPhysics
            from rgbd_batch_sensor import visible_plane,triangles
            from quality_physx_camera import material
            cache=UsdGeom.XformCache(Usd.TimeCode.Default())
            for prim in stage.Traverse():
                if not visible_plane(prim):continue
                enabled=prim.HasAPI(UsdPhysics.CollisionAPI)and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
                if enabled:continue # Original collider camera already includes it.
                v,f=triangles(prim);M=np.array(cache.GetLocalToWorldTransform(prim)).T;color=material(stage,str(prim.GetPath()))[0]
                plane=UsdGeom.Plane(prim);meta=dict(path=str(prim.GetPath()),axis=plane.GetAxisAttr().Get(),width_m=float(plane.GetWidthAttr().Get()),length_m=float(plane.GetLengthAttr().Get()),world_transform=M.tolist(),display_color=color.tolist(),collision_enabled=False,physics_changed=False)
                self.visual_planes.append((v@M[:3,:3].T+M[:3,3],f,color,meta))
        with np.load(model)as z:self.paths=z['paths'].tolist()
        self.all_bodies=view.create_rigid_body_view(self.paths);self.pose_indices={str(p):i for i,p in enumerate(self.all_bodies.prim_paths)}
        assert set(self.paths)<=set(self.pose_indices);self.preparation_s=time.perf_counter()-start
    def snapshot(self):
        poses=self.all_bodies.get_transforms().copy();return {path:poses[self.pose_indices[path]]for path in self.paths}
    def export(self,path):
        path=Path(path)
        if path.exists():raise FileExistsError('Camera geometry output already exists')
        if not self.visual_planes:path.symlink_to(os.path.relpath(self.model,path.parent));return
        with np.load(self.model,allow_pickle=False)as z:payload={name:z[name].copy()for name in z.files}
        vertices=[payload['static_vertices']];faces=[payload['static_faces']];colors=[payload['static_colors']];count=len(vertices[0])
        for v,f,color,meta in self.visual_planes:
            vertices.append(v.astype(np.float32));faces.append((f+count).astype(np.int32));colors.append(np.repeat(np.asarray(color,np.float32)[None,:],len(f),axis=0));count+=len(v)
        payload.update(static_vertices=np.concatenate(vertices),static_faces=np.concatenate(faces),static_colors=np.concatenate(colors),visible_plane_metadata_json=np.array(json.dumps([row[3]for row in self.visual_planes],sort_keys=True)))
        np.savez(path,**payload)

class Vision:
    def __init__(self,camera,calibration,out):
        self.camera=camera;self.calibration=calibration;self.jobs=[];self.start=None;self.out=Path(out);model=self.out/'camera-local-models.npz';camera.export(model)
        backend=calibration.get('ray_image_backend','embree_cpu')
        if backend not in ['embree_cpu','warp_cuda']:raise ValueError('Explicit supported ray image backend required')
        source_files=['realtime_camera_worker.py','rgbd_pin_axes.py','rgbd_fast_tracker.py','rgbd_initial_fast.py','rgbd_scattered_rings.py','rgbd_annular_cap_gate.py','rgbd_ring_translation.py','ray_camera_core.py']+(['warp_ray_camera.py']if backend=='warp_cuda'else[])
        version=hashlib.sha256(backend.encode()+hashlib.sha256(model.read_bytes()).digest()+b''.join((Path(__file__).parent/n).read_bytes()for n in source_files)).hexdigest();saved=getattr(builtins,'_m80_private_rgbd_worker',None)
        if saved and saved[0].poll()is None and (len(saved)!=4 or saved[3]!=version):
            saved[1].send(None);saved[0].wait(timeout=5);saved[1].close();saved[2].close();saved=None
        if saved and saved[0].poll()is None:
            self.process,self.conn,self.log=saved[:3];self.worker_preparation_s=0.;print('CAMERA_PROCESS_REUSED_READY',flush=True);return
        left,right=socket.socketpair();self.conn=Connection(left.detach());fd=right.fileno();log=open(self.out/'camera-worker.log','wb');script=Path(__file__).resolve().parent/'realtime_camera_worker.py';self.process=subprocess.Popen(['/isaac-sim/kit/python/bin/python3','-u',str(script),str(fd),str(model)],pass_fds=[fd],stdout=log,stderr=log,env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',M80_RAY_IMAGE_BACKEND=backend,CUDA_VISIBLE_DEVICES='0'if backend=='warp_cuda'else''));right.close();self.log=log
        if not self.conn.poll(180):
            self.process.terminate();self.process.wait(timeout=5);raise RuntimeError('Camera process preparation timed out')
        result=self.conn.recv();assert result['ready'];builtins._m80_private_rgbd_worker=(self.process,self.conn,self.log,version);self.worker_preparation_s=result['preparation_s'];print('CAMERA_PROCESS_READY',self.worker_preparation_s,flush=True)
    def request(self,event,H):
        mode=event['mode'];part=event['part'];H=H.copy();snapshot_started=time.perf_counter();poses=self.camera.snapshot()
        predicted_grasp=H@np.array(event['tracked_grasp'])if event.get('tracked_grasp')is not None else None
        if predicted_grasp is not None:target=predicted_grasp[:3,3];eye=target+np.array(event.get('camera_eye_offset',[.18,-.18,.08]));span=.10
        elif mode in ['initial','floor']:target=np.array([.51,-.70,1.1]);eye=[.75,-1.15,1.6];span=.28
        elif mode=='held_left':predicted=H@event['reference_grasp'];target=predicted[:3,3];eye=target+[-.25,.12,.2];span=.16
        elif mode=='pin_initial':target=np.array([.19,-.79,.916]);eye=[.19,-.79,1.35];span=.08
        else:target=H[:3,3];eye=target+(np.array([-.25,.12,-.2])if part!='pin'else np.array([0,0,.30]));span=.16 if part!='pin'else .08
        if mode=='initial' and self.calibration.get('loose_bin_mode'):
            floor=np.array(self.calibration['floor_origin_world_m']);target=floor+[0,0,.025];eye=floor+np.array(self.calibration.get('initial_eye_offset_m',[.40,-.12,.65]));span=.44
        if predicted_grasp is not None:seed=predicted_grasp[:3,3];axis=predicted_grasp[:3,2]
        elif mode=='floor':seed=np.array(event['reference']['pose']['center_m'])+event.get('floor_offset',np.zeros(3));axis=np.array(self.calibration['basket_to_world_R'])[:,2]
        elif mode=='pin_initial':seed=np.array([.19,-.79,.916]);axis=np.array([0,0,-1.])
        elif mode=='held_left':axis=predicted[:3,2].copy();axis*=1 if axis[2]>=0 else -1;seed=target
        elif mode=='held':axis=-H[:3,2]if part!='pin'else H[:3,2];axis*=1 if part=='pin'or axis[2]>=0 else -1;seed=target-axis*.003 if part!='pin'else target
        else:seed=axis=None
        initial_width,initial_height=640,400
        initial_override_keys=('initial_camera_width_px','initial_camera_height_px','initial_camera_span_m')
        initial_geometry={key:self.calibration[key] for key in initial_override_keys if key in self.calibration} if mode=='initial' and self.calibration.get('loose_bin_mode') else {}
        initial_geometry.update({key:event[key] for key in initial_override_keys if key in event})
        if initial_geometry:
            if mode!='initial' or not self.calibration.get('loose_bin_mode'):
                raise ValueError('Initial camera geometry override requires calibrated loose-bin initial capture')
            initial_width=initial_geometry.get('initial_camera_width_px',640);initial_height=initial_geometry.get('initial_camera_height_px',400)
            requested_span=initial_geometry.get('initial_camera_span_m',span)
            if (type(initial_width)is not int or type(initial_height)is not int or
                not 256<=initial_width<=1280 or not 256<=initial_height<=1280 or
                type(requested_span)not in (int,float)or not np.isfinite(requested_span)or not .20<=requested_span<=1.20):
                raise ValueError('Initial RGB-D size must be256--1280pixels and span0.20--1.20m')
            span=float(requested_span)
        precision_width=event.get('pin_precision_image_width_px')
        if precision_width is not None:
            if part!='pin' or mode!='held' or type(precision_width) is not int or not 384<=precision_width<=1280:
                raise ValueError('Pin precision camera width must be384–1280 pixels for held pin only')
        ring_precision_width=event.get('ring_precision_image_width_px')
        if ring_precision_width is not None:
            if precision_width is not None or part not in ['ring1','ring2'] or mode not in ['held','held_left'] or type(ring_precision_width)is not int or not 512<=ring_precision_width<=1280:
                raise ValueError('Ring precision camera width requires held ring only,512--1280 pixels')
            precision_width=ring_precision_width
        far_clip=event.get('rgbd_far_clip_m',self.calibration.get('rgbd_far_clip_m'))
        if far_clip is not None and (not isinstance(far_clip,(int,float))or not np.isfinite(far_clip)or not .1<=float(far_clip)<=5.):raise ValueError('Finite explicitly configured RGB-D clipping range0.1--5m required')
        future=Future();number=len(self.jobs);self.jobs.append((event,future));self.conn.send(dict(rgbd_far_clip_m=far_clip,id=number,mode=mode,part=part,H=H,eye=eye,target=target,span=span,width=initial_width if mode=='initial' and self.calibration.get('loose_bin_mode') else precision_width if precision_width is not None else int(self.calibration.get('tracking_image_width_px',384)) if mode in ['held','held_left','pregrasp_ring'] else 256,height=initial_height if mode=='initial' and self.calibration.get('loose_bin_mode') else precision_width if precision_width is not None else int(self.calibration.get('tracking_image_width_px',384)) if mode in ['held','held_left','pregrasp_ring'] else 256,seed=seed,axis=axis,robust_pin_cap_circle=bool(event.get('robust_pin_cap_circle')),require_annular_face=bool(event.get('require_annular_face')),allow_partial_preclose_tracking=bool(event.get('allow_partial_preclose_tracking')) and mode=='pregrasp_ring',calibration=self.calibration,render_snapshot=poses,snapshot_monotonic=snapshot_started,cycle_started=self.start));return future
    def poll(self):
        while self.conn.poll(0):
            value=self.conn.recv();self.jobs[value.pop('id')][1].set_result(value)
    def close(self):
        if os.environ.get('M80_REALTIME_KEEPALIVE'):return
        if self.process.poll()is None:
            try:self.conn.send(None);self.process.wait(timeout=5)
            except Exception:self.process.terminate();self.process.wait(timeout=5)
        self.conn.close();self.log.close()
    def save(self,out):
        deadline=time.perf_counter()+10
        while any(not f.done()for e,f in self.jobs)and time.perf_counter()<deadline:self.poll();time.sleep(.005)
        root=Path(out)/'rgb-perception';root.mkdir(exist_ok=True);records=[]
        for i,(event,future)in enumerate(self.jobs):
            if not future.done():continue
            value=future.result();work=root/f'observation-{i+1:03d}-{event["mode"]}';work.mkdir(exist_ok=True)
            if 'packet'not in value:records.append(dict(mode=event['mode'],part=event['part'],error=value['result'].get('error')));continue
            np.savez_compressed(work/'rgbd.npz',**value['packet']);overlay=value['packet']['rgb'].copy();overlay[value['mask']]=(overlay[value['mask']]*.5+[25,120,60]).clip(0,255).astype(np.uint8);name='detection-overlay.ppm'if event['mode']=='initial'else'tracking-overlay.ppm';(work/name).write_bytes(f'P6\n{overlay.shape[1]} {overlay.shape[0]}\n255\n'.encode()+overlay.tobytes());small={k:v for k,v in value.items()if k not in ['packet','mask','H']};small.update(delivered_wall_s=event.get('delivered_wall_s'),mode=event['mode'],part=event['part'],work=str(work),scene_sim_time_s=value['captured_wall_s'],hand_TCP_world=np.asarray(value['H']).tolist() if 'H'in value else None,timestamp_basis='monotonic wall time since cycle start; legacy scene_sim_time_s is an alias');records.append(small)
        (Path(out)/'rgb-observations.json').write_text(json.dumps(records,indent=2));self.close()
