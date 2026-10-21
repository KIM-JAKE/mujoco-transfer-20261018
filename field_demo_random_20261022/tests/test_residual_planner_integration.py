"""Actual planner/collision consumers of camera occupancy; no robot backend."""
import copy,unittest
import numpy as np
from field_demo import collision,random_planner as planner
from field_demo.residual_occupancy import build_occupancy
import test_random_planner as fixtures

class ResidualPlannerIntegration(unittest.TestCase):
 setUp=fixtures.PlannerTests.setUp
 write=fixtures.PlannerTests.write
 write_observation=fixtures.PlannerTests.write_observation
 plan=fixtures.PlannerTests.plan
 def snapshot(self):
  T=np.eye(4);T[:3,3]=[.4,0,-.5];packet=dict(depth_m=np.full((64,64),.35),K=np.array([[80,0,31.5],[0,80,31.5],[0,0,1.]]),T_cell_camera=T)
  meta=dict(capture_id=self.observation['capture_id'],calibration_sha256=self.observation['calibration_sha256'],source_packet_sha256=self.observation['source_packet_sha256'],captured_at_utc=self.now.isoformat(),source='realsense_live',replay=False,scene_epoch='unit-test',cell_frame='cell',rectified_registered_depth=True,projection_kind='rectified_pinhole',depth_uncertainty_m=.001)
  return build_occupancy(packet,meta,[[.3,-.1,-.05],[.5,.1,.05]],voxel_size_m=.02,depth_margin_m=.003,now_utc=self.now)
 def enable(self):self.config['scene']['require_residual_occupancy']=True;self.write('random-motion.json',self.config)
 def test_missing_residual_refuses_partial_instances_before_motion(self):
  self.enable()
  with self.assertRaisesRegex(ValueError,'residual occupancy'):self.plan()
 def test_actual_unknown_boxes_block_full_robot_in_planner(self):
  self.enable();self.observation['residual_occupancy']=self.snapshot().to_record();self.write_observation(self.observation)
  with self.assertRaisesRegex(collision.CollisionError,'residual_'):self.plan()
 def test_source_byte_binding_cannot_point_to_another_capture(self):
  self.enable();snap=self.snapshot();d=snap.to_record();self.observation['residual_occupancy']=d;self.observation['source_packet_sha256']='d'*64;self.write_observation(self.observation)
  with self.assertRaises(ValueError):self.plan()
 def test_inmemory_occupancy_cannot_replace_immutable_record(self):
  self.enable();self.observation['residual_occupancy']=self.snapshot().to_record()
  with self.assertRaisesRegex(ValueError,'Immutable residual'):self.plan()

class SelectedVoxelCollisionContract(unittest.TestCase):
 def setUp(self):
  self.active=[dict(name='arm'+str(i),a=np.array([[.2+i*.1,0,0]]),b=np.array([[.2+i*.1+.02,0,0]]),r=.001)for i in range(8)]
  self.active.append(dict(name='named_finger',a=np.array([[0,0,.4]]),b=np.array([[0,0,.4]]),r=.001))
  self.box=fixtures.obb(0,0,.4,half=(.01,.01,.01),name='observed_target_surface');self.box.update(source='conservative_registered_RGBD_rays',occupancy_kind='occupied',selected_contact_instance_id='capture:image:0')
  self.scene=dict(workspace=fixtures.obb(half=(3,3,3)),obstacles=[self.box]);self.phase=dict(corridors=[fixtures.obb(half=(3,3,3))],tcp_corridor_radius_m=.001,_target_instance_id='capture:image:0',_contact='grasp',target_contact_tool_capsules=[8]);self.envelope=dict(self_ignore_pairs=[[i,i+1]for i in range(8)])
 def run_check(self,**kw):return collision.check_envelopes(self.active,[],self.scene,self.phase,self.envelope,margin=.001,**kw)
 def test_observed_selected_surface_exempts_only_named_finger(self):self.run_check()
 def test_unknown_never_exempt_even_with_target_metadata(self):
  self.box['occupancy_kind']='unknown'
  with self.assertRaises(collision.CollisionError):self.run_check()
 def test_palm_or_forearm_cannot_use_selected_finger_exemption(self):
  self.active[0]=dict(self.active[-1],name='arm0');self.active[-1]=dict(self.active[-1],a=np.array([[1,1,1]]),b=np.array([[1,1,1]]))
  with self.assertRaises(collision.CollisionError):self.run_check()
 def test_held_payload_cannot_use_selected_or_fixture_exemption(self):
  self.phase.update(allow_payload_contact_fixture=self.box['name'],payload_contact_approved=True,payload_contact_region=fixtures.obb(half=(3,3,3)),_active_part='ring1')
  held=dict(self.active[-1],name='held_payload',part='ring1',owner='active',robot_ignore_capsules=[])
  with self.assertRaises(collision.CollisionError):self.run_check(payloads=[held])
class BoundedStockConsumer(SelectedVoxelCollisionContract):
 def enable(self):
  from test_stock_contact import setup
  _,kw=setup();self.phase.update(kw['phase']);policy=self.phase['bounded_loose_stock_contact']
  policy['bin_interior']['T_cell_box'][2][3]=.4
  policy['finger_capsules'][0]['name']='named_finger'
  self.envelope['tool_capsules_tcp_m']=[dict(name='named_finger')]
  self.box['occupancy_kind']='unknown'
 def test_actual_collision_consumer_allows_only_explicit_bounded_stock_finger(self):
  self.box['occupancy_kind']='unknown'
  with self.assertRaises(collision.CollisionError):self.run_check()
  self.enable();self.run_check();self.assertEqual(self.box['occupancy_kind'],'unknown')
 def test_same_physical_wall_or_palm_still_blocks(self):
  self.enable();self.box['source']='measured_fixture'
  with self.assertRaises(collision.CollisionError):self.run_check()
  self.box['source']='conservative_registered_RGBD_rays';self.active[0]=dict(self.active[-1],name='arm0')
  with self.assertRaises(collision.CollisionError):self.run_check()
 def test_disabled_real_force_guard_refuses_contact(self):
  self.enable();self.phase['force_monitor']['enabled']=False
  with self.assertRaises(collision.CollisionError):self.run_check()
if __name__=='__main__':unittest.main()
