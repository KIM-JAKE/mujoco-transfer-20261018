"""Synthetic SDK/image tests; no real robot/hardware-readiness claims."""
from pathlib import Path
import importlib.util,sys,unittest,copy
from datetime import datetime,timezone,timedelta
import numpy as np
from field_demo.native_hand_geometry import actual_hand_points
from field_demo.residual_occupancy import build_occupancy
from field_demo import known_hand_surface as mod

class HandSurfaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.now=datetime.now(timezone.utc);T=np.eye(4);T[2,3]=.5;hand=actual_hand_points(T,np.eye(4),np.eye(4),.064);h=w=128;k=np.array([[250.,0,64],[0,250.,64],[0,0,1]]);uv=np.rint(hand[:,:2]/hand[:,2:]*250+64).astype(int);inside=(uv>=0).all(1)&(uv<[w,h]).all(1);d=np.full(h*w,.8);np.minimum.at(d,uv[inside,1]*w+uv[inside,0],hand[inside,2]);cls.packet=dict(depth_m=d.reshape(h,w),K=k,T_cell_camera=np.eye(4));cls.meta=dict(capture_id='synthetic-capture',scene_epoch='static-fixture',cell_frame='cell',source='synthetic_unit_test',replay=True,rectified_registered_depth=True,projection_kind='rectified_pinhole',captured_at_utc=cls.now.isoformat(),calibration_sha256='a'*64,source_packet_sha256='b'*64,depth_uncertainty_m=.0005);cls.state=dict(side='right',capture_id=cls.meta['capture_id'],T_cell_tcp=T,F_T_EE=np.eye(4),T_flange_hand=np.eye(4),width_m=.064,captured_at_utc=cls.now.isoformat(),robot_time_s=123.,source='actual_sdk_fk',pose_uncertainty_m=.0005,stopped_under_capture_lock=True);cls.association=mod.associate_hand_pixels(cls.packet,cls.meta,[cls.state]);cls.snapshot=build_occupancy(cls.packet,cls.meta,[[-.12,-.12,.47],[.12,.12,.85]],voxel_size_m=.008,max_age_s=2,now_utc=cls.now);cls.rows,cls.proof=mod.planner_boxes_with_hand_association(cls.snapshot,cls.packet,cls.meta,cls.association)
    def test_real_surface_samples_not_background(self):
        matched=self.association.pixel_body>0;self.assertGreater(matched.sum(),100);self.assertTrue(np.all(self.packet['depth_m'][matched]<.7));self.assertFalse(self.association.pixel_body.flags.writeable)
    def test_unknown_and_grid_unchanged(self):
        original=self.snapshot.fingerprint;rows=self.snapshot.planner_boxes();self.assertEqual([r for r in rows if r['occupancy_kind']=='unknown'],[r for r in self.rows if r['occupancy_kind']=='unknown']);self.assertEqual(original,self.snapshot.fingerprint);self.assertEqual(self.proof['free_voxels_created'],0)
    def test_body_owner_only(self):
        row=next(r for r in self.rows if r.get('known_hand_surface',{}).get('hand_body')=='palm');c=dict(a=np.zeros((2,3)),robot_side='right',hand_body='palm',geometry_role='palm');kwargs=dict(capsule=c,association_sha256=self.association.fingerprint,active_robot=True)
        self.assertTrue(mod.known_hand_contact_mask(row,**kwargs).all())
        self.assertFalse(mod.known_hand_contact_mask(row,**dict(kwargs,is_payload=True)).any())
        for key,value in [('robot_side','left'),('hand_body','left_finger'),('geometry_role','arm')]:self.assertFalse(mod.known_hand_contact_mask(row,**dict(kwargs,capsule=dict(c,**{key:value}))).any())
        self.assertFalse(mod.known_hand_contact_mask(dict(row,occupancy_kind='unknown'),**kwargs).any());self.assertFalse(mod.known_hand_contact_mask(row,**dict(kwargs,association_sha256='c'*64)).any())
    def test_tampered_mask_rejected(self):
        mask=self.association.pixel_body.copy();mask[:]=1
        with self.assertRaises(ValueError):mod.planner_boxes_with_hand_association(self.snapshot,self.packet,self.meta,mod.HandSurfaceAssociation(mask,self.association.owners,self.association.proof))
    def test_capture_and_timing(self):
        with self.assertRaises(ValueError):mod.associate_hand_pixels(self.packet,self.meta,[dict(self.state,capture_id='other')])
        with self.assertRaises(ValueError):mod.associate_hand_pixels(self.packet,self.meta,[dict(self.state,captured_at_utc=(self.now-timedelta(seconds=.3)).isoformat())])
        changed=dict(self.meta,source_packet_sha256='c'*64)
        with self.assertRaises(ValueError):mod.planner_boxes_with_hand_association(self.snapshot,self.packet,changed,self.association)
    def test_mixed_hit_voxel_is_never_waived(self):
        mask=self.association.pixel_body.copy();flat=mask.ravel();ids=np.flatnonzero(flat);points,valid=mod._cloud(self.packet);bounds=np.asarray(self.snapshot.bounds_cell_m);spacing=(bounds[1]-bounds[0])/self.snapshot.shape;grid=np.floor((points.reshape(-1,3)[ids]-bounds[0])/spacing).astype(int);inside=((grid>=0)&(grid<self.snapshot.shape)).all(1);ids=ids[inside];grid=grid[inside];cells=np.ravel_multi_index(grid.T,self.snapshot.shape);unique,count=np.unique(cells,return_counts=True);cell=unique[np.argmax(count)];pixel=ids[np.flatnonzero(cells==cell)[0]];flat[pixel]=0;proof=copy.deepcopy(self.association.proof);proof['pixel_body_sha256']=mod.hashlib.sha256(mask.tobytes()).hexdigest();proof['association_sha256']=mod._sha({k:v for k,v in proof.items()if k!='association_sha256'});altered=mod.HandSurfaceAssociation(mask,self.association.owners,proof);rows,_=mod.planner_boxes_with_hand_association(self.snapshot,self.packet,self.meta,altered);cellindex=np.array(np.unravel_index(cell,self.snapshot.shape));center=bounds[0]+(cellindex+.5)*spacing
        covering=[r for r in rows if r['occupancy_kind']=='occupied'and np.all(abs(center-np.asarray(r['T_cell_box'])[:3,3])<=np.asarray(r['half_extents_m'])+1e-10)];self.assertTrue(covering);self.assertTrue(all('known_hand_surface'not in r for r in covering))

if __name__=='__main__':unittest.main()
