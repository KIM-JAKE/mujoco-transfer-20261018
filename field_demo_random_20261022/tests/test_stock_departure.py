import copy,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from field_demo.stock_departure import certify_issued_departure,departure_contact_mask,loose_stock_pair_mask
from field_demo.random_cycle import LiveRuntime
from test_stock_contact import setup

def fixture():
 obstacle,args=setup();old=args['phase'];phase=dict(_phase_name='stock_departure',_contact='depart',_active_part='ring1',_target_instance_id='fresh-held-image:ring',force_monitor=old['force_monitor'],bounded_loose_stock_departure=dict(schema_version=1,contact_policy=old['bounded_loose_stock_contact'],up_axis_cell=[0,0,1],maximum_lift_m=.07,up_axis_measurement=old['force_monitor']['measurement']))
 poses=np.tile(np.eye(4),(4,1,1));poses[:,2,3]=[0,.01,.04,.06]
 payload=dict(name='current_image_ring',owner='active',part='ring1',instance_id='fresh-held-image:ring',a=poses[:,:3,3],b=poses[:,:3,3],r=.005)
 phase['_departure_certificate']=certify_issued_departure(poses,phase,args['envelope'],selected_payload=payload)
 return obstacle,args,phase,poses,payload

class DepartureTests(unittest.TestCase):
 def test_explicit_selected_payload_and_fingers_can_depart_contact_never_palm(self):
  o,a,p,T,body=fixture();env=a['envelope']
  np.testing.assert_array_equal(departure_contact_mask(o,phase=p,capsule_index=0,capsule=body,envelope=env,is_payload=True),[True,True,False,False])
  self.assertFalse(departure_contact_mask(dict(o,source='measured_fixture'),phase=p,capsule_index=0,capsule=body,envelope=env,is_payload=True).any())
  self.assertFalse(departure_contact_mask(o,phase=p,capsule_index=0,capsule=body,envelope=env,active_robot=True).any())
  wrong=dict(body,instance_id='other-ring');self.assertFalse(departure_contact_mask(o,phase=p,capsule_index=0,capsule=wrong,envelope=env,is_payload=True).any())
 def test_downward_lateral_rotation_and_over70mm_are_rejected(self):
  _,a,p,T,body=fixture()
  trials=[]
  q=T.copy();q[2,2,3]=-.001;trials.append(q)
  q=T.copy();q[2,0,3]=.002;trials.append(q)
  q=T.copy();q[-1,2,3]=.071;trials.append(q)
  q=T.copy();ang=.01;q[-1,:3,:3]=[[np.cos(ang),-np.sin(ang),0],[np.sin(ang),np.cos(ang),0],[0,0,1]];trials.append(q)
  for trial in trials:
   with self.assertRaisesRegex(ValueError,'upward'):certify_issued_departure(trial,p,a['envelope'],selected_payload=body)
 def test_observed_loose_ring_contact_is_distinct_from_fixed_or_installed_geometry(self):
  _,a,p,T,body=fixture();stock=dict(source='current_RGBD_loose_ring',part='ring2')
  self.assertTrue(loose_stock_pair_mask(body,stock,phase=p,envelope=a['envelope'],is_payload=True).all())
  self.assertFalse(loose_stock_pair_mask(body,dict(stock,source='installed_fixture'),phase=p,envelope=a['envelope'],is_payload=True).any())
  self.assertFalse(loose_stock_pair_mask(body,stock,phase=dict(p,_phase_name='carry'),envelope=a['envelope'],is_payload=True).any())
 def test_runtime_splits_70mm_contact_lift_before_any_remaining_transport(self):
  _,a,p,_,_=fixture();settings=dict(phases=dict(stock_departure=dict(force_monitor=p['force_monitor'],bounded_loose_stock_departure=p['bounded_loose_stock_departure'])),robots=dict(right=a['envelope']))
  with tempfile.TemporaryDirectory()as d:
   root=Path(d);(root/'motion.json').write_text(json.dumps(settings));r=SimpleNamespace(root=root,recipe=dict(planner_config='motion.json'));entry=dict(part='ring1',pickup_side='right',lift_offsets_cell_m=[[0,0,.1],[0,0,.02]])
   phase,targets,remaining=LiveRuntime.initial_stock_lift(r,entry,np.eye(4));self.assertEqual(phase,'stock_departure');self.assertAlmostEqual(targets[-1][2][3],.07);self.assertAlmostEqual(remaining[0][2],.03);self.assertAlmostEqual(remaining[1][2],.02)

class DepartureCollisionConsumer(unittest.TestCase):
 def test_actual_payload_unknown_contact_masks_only_selected_support_skill(self):
  from field_demo import collision
  from test_residual_planner_integration import SelectedVoxelCollisionContract
  x=SelectedVoxelCollisionContract();x.setUp();_,args,phase,T,body=fixture()
  phase['bounded_loose_stock_departure']['contact_policy']['bin_interior']['T_cell_box'][2][3]=.4
  phase['bounded_loose_stock_departure']['contact_policy']['finger_capsules'][0]['name']='named_finger'
  phase['corridors']=x.phase['corridors'];phase['tcp_corridor_radius_m']=.001
  phase['_departure_certificate']=dict(kind='bounded_support_contact_not_free_space',selected_instance_id='fresh-held-image:ring')
  env=dict(x.envelope,tool_capsules_tcp_m=[dict(name='named_finger')],payload_robot_ignore_capsules=[8])
  x.box['occupancy_kind']='unknown';held=dict(x.active[-1],name='actual_held_ring',part='ring1',owner='active',instance_id='fresh-held-image:ring',robot_ignore_capsules=[])
  # Full actual collision consumer rejects this ordinary payload path.
  with self.assertRaises(collision.CollisionError):collision.check_envelopes(x.active,[],x.scene,x.phase,env,payloads=[held],margin=.001)
  collision.check_envelopes(x.active,[],x.scene,phase,env,payloads=[held],margin=.001)
  x.box['source']='measured_fixture'
  with self.assertRaises(collision.CollisionError):collision.check_envelopes(x.active,[],x.scene,phase,env,payloads=[held],margin=.001)
