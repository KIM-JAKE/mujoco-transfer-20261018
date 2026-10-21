from datetime import datetime,timezone
import hashlib,json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from field_demo.random_perception import load_random_config,detect_packet,_snapshot,_previous_image
from field_demo.stabilized.rgbd_geometry import axis_frame

ROOT=Path(__file__).resolve().parents[1]


def synthetic(seed=0):
    """Render only RGB-D plane/annulus pixels; no label/object-pose input to API."""
    rng=np.random.default_rng(seed);h,w=400,640;K=np.array([[1000.,0,(w-1)/2],[0,1000.,(h-1)/2],[0,0,1]])
    T=np.eye(4);T[:3,:3]=np.diag([1.,-1.,-1.]);T[:3,3]=[0,0,1]
    v,u=np.indices((h,w));rays=np.stack([(u-K[0,2])/K[0,0],(v-K[1,2])/K[1,1],np.ones((h,w))],axis=2)
    world=rays@T[:3,:3].T;depth=np.ones((h,w));rgb=np.full((h,w,3),230,np.uint8);truth={}
    for i,(part,radius,inner,half) in enumerate((('ring1',.0375,.032,.00775),('ring2',.031,.0255,.007))):
        n=np.r_[rng.uniform(-.30,.30,2),1.];n/=np.linalg.norm(n)
        center=np.array([(-.08 if i==0 else .08)+rng.uniform(-.025,.025),rng.uniform(-.055,.055),.035])
        face=center+n*half;t=(n@(face-T[:3,3]))/(world@n);points=world*t[:,:,None]+T[:3,3]
        basis=axis_frame(n)[:,:2];radial=np.linalg.norm((points-face)@basis,axis=2)
        mask=(radial>inner)&(radial<radius)&(t>0)&(t<depth);depth[mask]=t[mask];rgb[mask]=100
        truth[part]=dict(center_m=center,axis=-n)
    camera=rays*depth[:,:,None]
    packet=dict(rgb=rgb,depth_m=depth.astype(np.float32),K=K,points_camera_image_m=camera.astype(np.float32))
    cfg=dict(schema_version=2,units='m',_replay_only=True,calibration_kind='simulation_replay',
             calibration_id='SYNTHETIC_REPLAY_ONLY',_calibration_sha256='SYNTHETIC',camera_frame='test_optical',
             camera_serial='NOT_A_DEVICE',cell_frame='test_cell',T_cell_camera=T.tolist(),
             robots={s:dict(base_frame=s+'_base',T_cell_base=np.eye(4).tolist()) for s in ('left','right')},
             part_robot=dict(ring1='right',ring2='right',pin='right'),
             basket=dict(floor_origin_cell_m=[0,0,0],basket_to_cell_R=np.eye(3).tolist(),outer_bounds_cell_m=[[-.3,-.3,-.01],[.3,.3,.2]]),
             held_workspace_bounds_cell_m=[[-.3,-.3,0],[.3,.3,.3]],pin_holder=dict(enabled=False))
    meta=dict(capture_id='synthetic-image-'+str(seed),captured_at_utc=datetime.now(timezone.utc).isoformat(),
              units='m',camera_frame=cfg['camera_frame'],point_cloud_frame=cfg['camera_frame'],
              calibration_id=cfg['calibration_id'],calibration_sha256=cfg['_calibration_sha256'],
              source='synthetic_rgbd_replay',replay=True,rectified_registered_depth=True)
    return packet,cfg,meta,truth


def render_part_packet(part,T_cell_part,config,width=640,height=640,focal=960.):
    """SYNTHETIC REPLAY ONLY: render a part pose into RGB-D camera pixels.

    Returns camera arrays only, never labels, scene seeds or pose-oracle keys.
    This helper generates contract fixtures; it is not an inference algorithm
    or evidence of physical robot/sensor readiness.
    """
    from isaac_port.fieldkit.perception import ASSETS
    T=np.asarray(config['T_cell_camera'],float);part_pose=np.asarray(T_cell_part,float)
    K=np.array([[focal,0,(width-1)/2],[0,focal,(height-1)/2],[0,0,1.]])
    v,u=np.indices((height,width));rays=np.stack([(u-K[0,2])/focal,(v-K[1,2])/focal,np.ones((height,width))],axis=2)
    world_rays=rays@T[:3,:3].T;floor=np.asarray(config['basket']['floor_origin_cell_m']);normal=np.asarray(config['basket']['basket_to_cell_R'])[:,2]
    denominator=world_rays@normal
    depth=np.divide(normal@(floor-T[:3,3]),denominator,out=np.full((height,width),np.nan),where=abs(denominator)>1e-10)
    depth[depth<=.05]=np.nan;points=rays*depth[:,:,None];rgb=np.full((height,width,3),230,np.uint8)
    if part in ('ring1','ring2'):
        axis=part_pose[:3,2];center=part_pose[:3,3];visible=1 if axis@(T[:3,3]-center)>0 else -1
        radius,half=(.0375,.00775) if part=='ring1' else (.031,.007)
        inner=(.0355 if visible>0 else .032) if part=='ring1' else (.029 if visible>0 else .0255)
        face=center+axis*visible*half;den=world_rays@axis
        z=np.divide(axis@(face-T[:3,3]),den,out=np.full_like(depth,np.nan),where=abs(den)>1e-10)
        hit=world_rays*z[:,:,None]+T[:3,3];xy=(hit-face)@part_pose[:3,:2];radial=np.linalg.norm(xy,axis=2)
        mask=(radial>inner)&(radial<radius)&(z>.05)&((z<depth)|~np.isfinite(depth))
        depth[mask]=z[mask];points[mask]=rays[mask]*z[mask,None];rgb[mask]=100
    elif part=='pin':
        with np.load(ASSETS/'part-models.npz') as models:world=models['pin_points']@part_pose[:3,:3].T+part_pose[:3,3]
        camera=(world-T[:3,3])@T[:3,:3];good=camera[:,2]>.05;camera=camera[good]
        uv=np.rint(camera[:,:2]/camera[:,2,None]*focal+K[:2,2]).astype(int)
        keep=(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height);uv,camera=uv[keep],camera[keep]
        ids=uv[:,1]*width+uv[:,0];order=np.lexsort((camera[:,2],ids));_,first=np.unique(ids[order],return_index=True);chosen=order[first]
        x,y=uv[chosen,0],uv[chosen,1];z=camera[chosen,2];near=(z<depth[y,x])|~np.isfinite(depth[y,x]);x,y,z,chosen=x[near],y[near],z[near],chosen[near]
        points[y,x]=camera[chosen];depth[y,x]=z;rgb[y,x]=100
    elif part is not None:raise ValueError('Synthetic renderer supports ring1/ring2/pin/None')
    return dict(rgb=rgb,depth_m=depth.astype(np.float32),K=K,points_camera_image_m=points.astype(np.float32))


class RandomPerceptionTests(unittest.TestCase):
    def test_frozen_detector_matches_manifest_except_package_import(self):
        for name in ('rgbd_scattered_rings.py','rgbd_geometry.py'):
            frozen=(ROOT/'stabilization/v119'/name).read_bytes()
            packed=(ROOT/'src/field_demo/stabilized'/name).read_bytes()
            self.assertEqual(packed.replace(b'from .rgbd_geometry import',b'from rgbd_geometry import'),frozen)
            manifest=json.loads((ROOT/'stabilization/v119/IMPORT.json').read_text())
            self.assertEqual(hashlib.sha256(frozen).hexdigest(),manifest['copied_verified_source'][name])

    def test_authentic_v119_initial_frames_find_both_classes_without_scene_truth(self):
        config=load_random_config(ROOT/'src/field_demo/stabilized/replay-camera.json')
        for path in sorted((ROOT/'stabilization/replay-frames').glob('*initial.npz')):
            with self.subTest(frame=path.name):
                packet,metadata=_snapshot(path,config,replay=True)
                result=detect_packet(packet,config,metadata,replay=True)
                self.assertTrue(result['both_ring_sizes_visible'])
                self.assertGreaterEqual(len(result['instances']),2)
                self.assertFalse(result['initial_object_pose_prior_used'])
                self.assertFalse(result['execution_allowed'])
                self.assertTrue(all(p['front_back_from_RGBD'] for p in result['instances']))
                requested=detect_packet(packet,config,metadata,replay=True,part='ring1')
                self.assertTrue(requested['both_ring_sizes_visible'])

    def test_randomized_xy_tilt_pixels_need_no_object_seed_or_labels(self):
        for seed in (5,17,81):
            with self.subTest(seed=seed):
                packet,cfg,meta,truth=synthetic(seed)
                result=detect_packet(packet,cfg,meta,replay=True)
                self.assertTrue(result['both_ring_sizes_visible'])
                self.assertFalse(result['ring_pose_seed_used'])
                for part in ('ring1','ring2'):
                    p=result['parts'][part]
                    self.assertLess(np.linalg.norm(np.asarray(p['center_m'])-truth[part]['center_m']),.003)
                    self.assertGreater(np.asarray(p['axis'])@truth[part]['axis'],.99)

    def test_forbidden_truth_masks_flat_only_and_replay_live_are_rejected(self):
        packet,cfg,meta,_=synthetic(4)
        for key in ('labels','object_poses','scene_seed','simulator_body_poses'):
            with self.subTest(key=key):
                with self.assertRaisesRegex(ValueError,'Forbidden'):
                    detect_packet(dict(packet,**{key:np.zeros((400,640))}),cfg,meta,replay=True)
        flat=dict(packet);del flat['points_camera_image_m'];flat['points_camera_m']=np.zeros((100,3))
        live_cfg=dict(cfg,calibration_kind='real_camera_recording')
        with self.assertRaisesRegex(ValueError,'Organized SDK'):
            detect_packet(flat,live_cfg,meta,replay=True)
        with self.assertRaisesRegex(RuntimeError,'replay calibration'):
            detect_packet(packet,cfg,meta,replay=False)
        bad=dict(packet,rgb=np.zeros((20,3),np.uint8))
        with self.assertRaisesRegex(ValueError,'Organized'):
            detect_packet(bad,cfg,meta,replay=True)
        uniform=dict(packet,rgb=np.full_like(packet['rgb'],230))
        with self.assertRaises(RuntimeError):detect_packet(uniform,cfg,meta,replay=True)

    def test_held_scope_uses_current_image_near_encoder_tcp_and_rejects_ambiguity(self):
        packet,cfg,meta,truth=synthetic(19)
        tcp=np.eye(4);tcp[:3,3]=truth['ring2']['center_m']+[0,0,.01]
        result=detect_packet(packet,cfg,meta,replay=True,scope='held',part='ring2',held_tcp_pose=tcp)
        self.assertEqual(len(result['instances']),1)
        self.assertLess(np.linalg.norm(np.array(result['instances'][0]['center_m'])-truth['ring2']['center_m']),.003)
        # Held registration uses a viewing slab, not the basket support plane.
        sideways=dict(cfg,basket=dict(cfg['basket'],basket_to_cell_R=[[0,0,1],[0,1,0],[-1,0,0]]))
        second=detect_packet(packet,sideways,meta,replay=True,scope='held',part='ring2',held_tcp_pose=tcp)
        self.assertEqual(len(second['instances']),1)
        tcp[:3,3]=[.25,.25,.25]
        with self.assertRaises(RuntimeError):detect_packet(packet,cfg,meta,replay=True,scope='held',part='ring2',held_tcp_pose=tcp)

    def test_template_unmeasured_cannot_load_as_live(self):
        with self.assertRaises(ValueError):load_random_config(ROOT/'config/templates/random-camera.json')

    def test_unknown_basket_is_occupied_scene_only_never_a_pickup_target(self):
        packet,cfg,meta,_=synthetic(26);packet=dict(packet,rgb=np.full_like(packet['rgb'],230))
        with self.assertRaises(RuntimeError):detect_packet(packet,cfg,meta,replay=True,allow_unknown_basket=True)
        pose=np.eye(4);pose[:3,3]=[0,0,.15]
        cfg['basket']['unrecognized_scene_enclosure']=dict(name='unknown_basket_clutter',T_cell_box=pose.tolist(),
              half_extents_m=[.31,.31,.2],includes_possible_protrusions=True,
              measurement=dict(verified=False,method='Synthetic replay fixture only'))
        result=detect_packet(packet,cfg,meta,replay=True,allow_unknown_basket=True)
        self.assertEqual(result['basket_scene_status'],'unknown_treated_fully_occupied')
        self.assertTrue(result['scene_only']);self.assertFalse(result['instances']);self.assertFalse(result['proposals'])
        self.assertEqual(len(result['conservative_scene_obstacles']),1)
        with self.assertRaises(RuntimeError):detect_packet(packet,cfg,meta,replay=True)

    def test_cross_camera_prior_requires_exact_old_calibration_and_shared_cell(self):
        _,cfg,meta,_=synthetic(22)
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            saved={k:v for k,v in cfg.items() if not k.startswith('_')}
            saved.update(replay_only=True,replay_source='Synthetic test RGB-D only')
            old=root/'old.json';old.write_text(json.dumps(saved));oldc=load_random_config(old)
            newer=dict(saved,camera_frame='camera_two_optical',camera_serial='NOT_A_DEVICE_2')
            path=root/'new.json';path.write_text(json.dumps(newer));newc=load_random_config(path)
            prior=dict(state='random_ring_observation',replay=True,_camera_config_path=str(old),
                       calibration_sha256=oldc['_calibration_sha256'],source_metadata=dict(meta,calibration_sha256=oldc['_calibration_sha256']),
                       instances=[dict(part='pin',T_cell_part=np.eye(4).tolist())])
            self.assertEqual(_previous_image(prior,newc,True,'pin')['part'],'pin')
            bad=dict(newc,cell_frame='another_cell')
            with self.assertRaisesRegex(ValueError,'cell frame'):_previous_image(prior,bad,True,'pin')
            old.write_text(old.read_text()+' ')
            with self.assertRaisesRegex(ValueError,'changed'):_previous_image(prior,newc,True,'pin')

    def test_held_pin_fits_current_pixels_using_previous_camera_encoder_grasp_prior(self):
        from isaac_port.fieldkit.perception import ASSETS
        packet,cfg,meta,_=synthetic(27)
        h,w=packet['depth_m'].shape;K=packet['K'].copy();K[0,0]=K[1,1]=2000
        v,u=np.indices((h,w));rays=np.stack([(u-K[0,2])/2000,(v-K[1,2])/2000,np.ones((h,w))],axis=2)
        rgb=np.full((h,w,3),230,np.uint8);cloud=rays.copy();depth=np.ones((h,w))
        T=np.asarray(cfg['T_cell_camera']);target=np.eye(4);target[:3,:3]=axis_frame([.6,.2,.8]);target[:3,3]=[0,0,.08]
        with np.load(ASSETS/'part-models.npz') as z:world=z['pin_points']@target[:3,:3].T+target[:3,3]
        cam=(world-T[:3,3])@T[:3,:3]
        uv=np.rint(cam[:,:2]/cam[:,2,None]*[2000,2000]+K[:2,2]).astype(int)
        keep=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h);uv,cam=uv[keep],cam[keep]
        ids=uv[:,1]*w+uv[:,0];order=np.lexsort((cam[:,2],ids));_,first=np.unique(ids[order],return_index=True);chosen=order[first]
        x,y=uv[chosen,0],uv[chosen,1];cloud[y,x]=cam[chosen];depth[y,x]=cam[chosen,2];rgb[y,x]=100
        packet=dict(rgb=rgb,depth_m=depth.astype(np.float32),K=K,points_camera_image_m=cloud.astype(np.float32))
        prior=target.copy();prior[:3,3]+=[.0015,.001,0]
        previous=dict(state='random_ring_observation',replay=True,calibration_sha256=cfg['_calibration_sha256'],source_metadata=meta,
                      instances=[dict(part='pin',T_cell_part=prior.tolist(),center_m=prior[:3,3].tolist())])
        tcp=np.eye(4);tcp[:3,3]=[0,0,.09];G=np.linalg.inv(tcp)@prior
        result=detect_packet(packet,cfg,meta,replay=True,scope='held',part='pin',held_tcp_pose=tcp,
                             held_grasp_transform=G,previous_observation=previous,held_robot='left')
        measured=np.asarray(result['parts']['pin']['T_cell_part'])
        self.assertLess(np.linalg.norm(measured[:3,3]-target[:3,3]),.0005)
        self.assertGreater(np.linalg.norm(measured[:3,3]-prior[:3,3]),.001)
        self.assertEqual(result['proposals']['pin']['robot'],'left')
        np.testing.assert_allclose(np.asarray(result['T_tcp_part']),np.linalg.inv(tcp)@measured)


if __name__=='__main__':unittest.main()
