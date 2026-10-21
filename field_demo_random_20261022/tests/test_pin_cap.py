"""Recorded SIM pixels + synthetic quality rejection, no robot/SDK calls."""
import copy,json,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from field_demo.pin_cap import estimate
from field_demo.pin_cap_geometry import _robust_cap_circle
from field_demo.random_perception import detect_packet,load_random_config
from datetime import datetime,timezone

ROOT=Path(__file__).resolve().parents[1];DIR=ROOT/'replay/pin-cap308'
class PinCapTests(unittest.TestCase):
    def inputs(self):
        fixture=json.loads((DIR/'component-fixtures.json').read_text());row=fixture['rows'][0]
        return dict(np.load(DIR/row['source_packet'])),np.asarray(row['prior_body_from_previous_RGBD_and_SIM_encoder']),row
    def test_shifted_ROI_still_measures_current_XY_not_prior(self):
        packet,prior,row=self.inputs();prior[0,3]+=.001
        fitted=estimate(packet,prior)
        self.assertFalse(fitted['fallback_used']);self.assertFalse(fitted['lateral_from_previous_RGBD_grasp_and_encoders'])
        np.testing.assert_allclose(fitted['measured_physical_axis_center_m'],row['reference_SIM_RGBD_pose']['center_m'],atol=.000005)
    def test_partial_arc_and_wrong_class_radius_reject(self):
        a=np.linspace(0,np.pi,70);half=np.c_[np.cos(a),np.sin(a)]*.0025
        with self.assertRaises(RuntimeError):_robust_cap_circle(half,.0025)
        a=np.linspace(0,2*np.pi,80,endpoint=False);wrong=np.c_[np.cos(a),np.sin(a)]*.00525
        with self.assertRaises(RuntimeError):_robust_cap_circle(wrong,.0025)
    def test_missing_actual_surface_rejects_no_xy_prediction(self):
        packet,prior,_=self.inputs();packet['rgb']=np.full_like(packet['rgb'],255)
        with self.assertRaises(RuntimeError):estimate(packet,prior)
    def test_organized_SDK_XYZ_is_used_and_invalid_registration_rejects(self):
        packet,prior,row=self.inputs();depth=packet['depth_m'];K=packet['K'];v,u=np.indices(depth.shape)
        xyz=np.stack([(u-K[0,2])/K[0,0]*depth,(v-K[1,2])/K[1,1]*depth,depth],-1);xyz[:,:,0]+=.0001
        packet['points_camera_image_m']=xyz;fit=estimate(packet,prior)
        expected=np.array(row['reference_SIM_RGBD_pose']['center_m'])+packet['T_cell_camera'][:3,:3]@np.array([.0001,0,0])
        np.testing.assert_allclose(fit['measured_physical_axis_center_m'],expected,atol=.000005)
        xyz[:,:,2]+=.0011
        with self.assertRaises(ValueError):estimate(packet,prior)
    def test_public_optin_failure_never_falls_back_to_ICP(self):
        packet,prior,row=self.inputs();cfg=load_random_config(DIR/'observation-010-cap-camera.json')
        metadata=dict(capture_id='OFFLINE-mocked-cap-reject',captured_at_utc=datetime.now(timezone.utc).isoformat(),units='m',camera_frame=cfg['camera_frame'],point_cloud_frame=cfg['camera_frame'],calibration_id=cfg['calibration_id'],calibration_sha256=cfg['_calibration_sha256'],source='offline_mock_only',replay=True,rectified_registered_depth=True)
        with patch('field_demo.pin_cap.estimate',side_effect=RuntimeError('Current cap occluded')),patch('isaac_port.fieldkit.perception.CadPoseEstimator')as icp:
            with self.assertRaisesRegex(RuntimeError,'occluded'):detect_packet(packet,cfg,metadata,replay=True,scope='fixture',part='pin',fixture_T_cell_target=prior)
        icp.assert_not_called()
    def test_camera_packet_cannot_accept_actor_state(self):
        packet,prior,_=self.inputs();packet['actor_poses']={}
        with self.assertRaises(ValueError):estimate(packet,prior)

if __name__=='__main__':unittest.main()
