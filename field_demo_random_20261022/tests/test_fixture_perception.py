"""Actual pixel inference in measured fixture ROI; no fabricated TCP input."""
import copy,unittest
import numpy as np
from field_demo.random_perception import detect_packet
from field_demo.stabilized.rgbd_geometry import axis_frame
from test_random_perception import synthetic,render_part_packet

class FixturePerceptionTests(unittest.TestCase):
 def test_ring_fixture_target_only_restricts_ROI_pixels_determine_pose(self):
  packet,cfg,meta,truth=synthetic(9)
  for part in ['ring1','ring2']:
   target=np.eye(4);target[:3,:3]=axis_frame(truth[part]['axis']);target[:3,3]=truth[part]['center_m']+[.001,0,0]
   result=detect_packet(packet,cfg,meta,replay=True,scope='fixture',part=part,fixture_T_cell_target=target)
   value=result['instances'][0];self.assertEqual(value['part'],part);self.assertLess(np.linalg.norm(np.array(value['center_m'])-truth[part]['center_m']),.0008)
   self.assertTrue(result['fixture_target_prior_used']);self.assertEqual(result['fixture_search_prior_source'],'measured_fixture_target')
   self.assertNotIn('measured_tcp_pose',result)
 def test_fixture_goal_cannot_make_missing_ring_appear(self):
  packet,cfg,meta,_=synthetic(9);packet['depth_m'][:]=1.;packet['rgb'][:]=230
  v,u=np.indices(packet['depth_m'].shape);K=packet['K'];packet['points_camera_image_m']=np.stack([(u-K[0,2])/K[0,0],(v-K[1,2])/K[1,1],np.ones(v.shape)],-1).astype(np.float32)
  target=np.eye(4);target[:3,3]=[0,0,.035]
  with self.assertRaises(RuntimeError):detect_packet(packet,cfg,meta,replay=True,scope='fixture',part='ring1',fixture_T_cell_target=target)
 def test_pin_fixture_uses_measured_target_and_current_CAD_pixels(self):
  _,cfg,meta,_=synthetic(3);target=np.eye(4);target[:3,:3]=axis_frame([.6,.2,.8]);target[:3,3]=[0,0,.08]
  packet=render_part_packet('pin',target,cfg,width=640,height=640,focal=1800.)
  result=detect_packet(packet,cfg,meta,replay=True,scope='fixture',part='pin',fixture_T_cell_target=target)
  self.assertEqual(result['instances'][0]['part'],'pin');self.assertLess(np.linalg.norm(np.array(result['instances'][0]['center_m'])-target[:3,3]),.001)
  self.assertIn('measured_fixture_target',result['instances'][0]['search_prior_scope'])
if __name__=='__main__':unittest.main()

class FixtureRuntimeConsumption(unittest.TestCase):
 def test_withdrawal_verifies_every_installed_part_from_new_fixture_images(self):
  from types import SimpleNamespace
  from unittest.mock import Mock
  from field_demo.random_cycle import LiveRuntime
  runtime=LiveRuntime.__new__(LiveRuntime);runtime.installed_parts=[dict(part=p)for p in ['ring1','ring2','pin']];runtime.wait=Mock();runtime.ready=Mock();runtime.tcp=lambda side:np.eye(4).tolist()
  recipe=dict(parts=[dict(part=p,min_tcp_clearance_m=.04)for p in ['ring1','ring2','pin']],insertion_targets={p:np.array([[1,0,0,1.],[0,1,0,1.],[0,0,1,1.],[0,0,0,1.]]).tolist()for p in ['ring1','ring2','pin']},feedback_timeout_s=2.,feedback_dwell_s=.1)
  runtime.verify_clearance(recipe)
  self.assertEqual([c.args[0]for c in runtime.wait.call_args_list],['ring1_retained','ring2_retained','pin_retained'])
