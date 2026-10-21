"""Synthetic SDK boundary regressions; no hardware calls or readiness claim."""
import copy,unittest
from datetime import datetime,timezone,timedelta
import numpy as np
from field_demo import known_hand_capture as capture,collision
import test_residual_planner_integration as fixtures

class CaptureBinding(unittest.TestCase):
 def setUp(self):
  now=datetime.now(timezone.utc);self.capture=now-timedelta(seconds=.1);self.meta=dict(capture_id='actual-capture-test',captured_at_utc=self.capture.isoformat(),exposure_s=dict(rgb=.001,depth=.001))
  self.opts=dict(surface_tolerance_m=.002,pose_uncertainty_m=.0005,max_sdk_capture_gap_s=.2);self.robots={s:dict(T_cell_base=np.eye(4).tolist())for s in ['left','right']};self.mounts={s:dict(T_flange_hand=np.eye(4).tolist())for s in ['left','right']}
  def sdk(offset):return dict(read_only=True,server_protocol_version=10,robot_mode_name='idle',q=[0.]*7,q_d=[0.]*7,dq=[0.]*7,dq_d=[0.]*7,F_T_EE_column_major=np.eye(4).ravel(order='F').tolist(),O_T_EE_column_major=np.eye(4).ravel(order='F').tolist(),robot_time_s=100.+offset)
  def row(offset):
   t=self.capture+timedelta(seconds=offset);return dict(read_started_utc=(t-timedelta(seconds=.002)).isoformat(),read_returned_utc=t.isoformat(),state=sdk(offset),hand=dict(completed=True,is_grasped=False,robot_time_s=10.+offset,width_m=.064,max_width_m=.08,captured_at_utc=(t-timedelta(seconds=.001)).isoformat()))
  self.before={s:row(-.025)for s in self.robots};self.after={s:row(.025)for s in self.robots}
 def bind(self):return capture.bind_states(self.before,self.after,self.meta,self.robots,self.mounts,self.opts)
 def test_bound_pose_is_sdk_fk_not_image_or_requested_target(self):
  r=self.bind();self.assertEqual(len(r['actual_states']),2);self.assertEqual(r['actual_states'][0]['width_m'],.064);self.assertFalse(r['physical_unsampled_motion_certified'])
 def test_buffered_exposure_or_stationary_clock_refused(self):
  self.meta['captured_at_utc']=(self.capture-timedelta(seconds=.1)).isoformat()
  with self.assertRaisesRegex(RuntimeError,'between'):self.bind()
  self.meta['captured_at_utc']=self.capture.isoformat();self.after['left']['state']['robot_time_s']=self.before['left']['state']['robot_time_s']
  with self.assertRaisesRegex(RuntimeError,'clock'):self.bind()
 def test_actual_motion_aperture_change_and_missing_uncertainty_refused(self):
  self.after['left']['state']['q'][2]=.0001
  with self.assertRaisesRegex(RuntimeError,'moved'):self.bind()
  self.after['left']['state']['q'][2]=0;self.after['left']['hand']['width_m']=.063
  with self.assertRaisesRegex(RuntimeError,'aperture'):self.bind()
  with self.assertRaises(ValueError):capture.settings(dict(self.opts,pose_uncertainty_m=0))

class ActualCollisionSurfaceConsumer(unittest.TestCase):
 setUp=fixtures.SelectedVoxelCollisionContract.setUp
 run_check=fixtures.SelectedVoxelCollisionContract.run_check
 def enable(self):
  self.phase.update(target_contact_tool_capsules=[],_known_hand_association_hashes=['a'*64]);self.active[-1].update(robot_side='right',hand_body='right_finger',geometry_role='finger')
  self.box.update(source_capture_ids=['capture'],known_hand_surface=dict(robot_side='right',hand_body='right_finger',capture_id='capture',association_sha256='a'*64))
 def test_real_consumer_requires_validated_hash_and_same_body(self):
  self.enable();self.run_check();self.phase['_known_hand_association_hashes']=[]
  with self.assertRaises(collision.CollisionError):self.run_check()
  self.phase['_known_hand_association_hashes']=['a'*64];self.active[-1]['hand_body']='left_finger'
  with self.assertRaises(collision.CollisionError):self.run_check()
 def test_envelope_prior_same_body_only_never_generic_unknown(self):
  self.enable();self.box['occupancy_kind']='known_robot_envelope';self.box['known_hand_surface']['material_source']='eroded_pinned_official_collision_envelope_assumption';self.run_check()
  self.box['known_hand_surface']['material_source']='invented_solid'
  with self.assertRaises(collision.CollisionError):self.run_check()
 def test_unknown_and_other_arm_and_payload_still_block(self):
  self.enable();self.box['occupancy_kind']='unknown'
  with self.assertRaises(collision.CollisionError):self.run_check()
  self.box['occupancy_kind']='occupied';self.box['known_hand_surface']['robot_side']='left'
  with self.assertRaises(collision.CollisionError):self.run_check()
  self.box['known_hand_surface']['robot_side']='right';held=dict(self.active[-1],name='held_payload',part='ring1',owner='active',robot_ignore_capsules=[])
  with self.assertRaises(collision.CollisionError):self.run_check(payloads=[held])

if __name__=='__main__':unittest.main()
