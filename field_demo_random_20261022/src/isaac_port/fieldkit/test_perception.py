"""Metric frame, stale-data and noisy CAD registration regressions."""
from datetime import datetime,timezone,timedelta
from pathlib import Path
from tempfile import TemporaryDirectory
import json,unittest
import numpy as np
from scipy.spatial.transform import Rotation
from .perception import ASSETS,CadPoseEstimator,deproject,load_snapshot,pose,transform


class PerceptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.estimator=CadPoseEstimator();cls.surfaces=np.load(ASSETS/'part-validation-surfaces.npz');cls.meta=json.loads((ASSETS/'part-models.json').read_text())['parts']

    def test_independent_partial_noisy_surfaces_recover_shifted_pickup_centers(self):
        rng=np.random.default_rng(923);eye=np.array([0.,-.8,2.])
        for sign in [-1,1]:
            for name in ['ring1','ring2','pin']:
                with self.subTest(sign=sign,part=name):
                    row=self.meta[name];prior=pose(row['nominal_position'],row['nominal_quaternion_wxyz']);truth=prior.copy();truth[:3,3]+=sign*np.array([.006,-.004,.001]);truth[:3,:3]=Rotation.from_euler('z',sign*4,degrees=True).as_matrix()@truth[:3,:3]
                    points=self.surfaces[name+'_points']@truth[:3,:3].T+truth[:3,3];normals=self.surfaces[name+'_normals']@truth[:3,:3].T;visible=np.sum(normals*(eye-points),axis=1)>0;observed=points[visible];observed=observed[rng.choice(len(observed),1400,replace=False)]+rng.normal(0,.00025,(1400,3));clutter=rng.uniform(points.min(axis=0)-.005,points.max(axis=0)+.005,(200,3));fit=self.estimator.fit(name,np.r_[observed,clutter],prior)
                    self.assertLess(np.linalg.norm(fit.T_cell_part[:3,3]-truth[:3,3]),.001)
                    if name.startswith('ring'):
                        error=np.arccos(np.clip(fit.T_cell_part[:3,2]@truth[:3,2],-1,1))
                    else:error=Rotation.from_matrix(fit.T_cell_part[:3,:3]@truth[:3,:3].T).magnitude()
                    self.assertLess(error,np.deg2rad(2))
                    self.assertGreater(fit.inlier_fraction,.75)

    def test_registered_metric_depth_and_camera_transform(self):
        points,labels=deproject(np.array([[1000,2000],[0,1000]],dtype=np.uint16),np.array([[100.,0,0],[0,100.,0],[0,0,1]]),.001,rectified=True,labels=np.array([[1,2],[0,3]]))
        np.testing.assert_allclose(points,[[0,0,1],[.02,0,2],[.01,.01,1]])
        np.testing.assert_equal(labels,[1,2,3])
        with self.assertRaises(ValueError):deproject(np.ones((2,2)),np.eye(3),.001,rectified=False)
        bad=np.eye(4);bad[0,0]=-1
        with self.assertRaises(ValueError):transform(bad)

    def test_stale_frame_and_wrong_calibration_cannot_be_used_live(self):
        with TemporaryDirectory()as directory:
            p=Path(directory)/'snapshot.npz';np.savez_compressed(p,points_camera_m=np.zeros((200,3)))
            metadata=dict(camera_frame='cam',calibration_id='site1',captured_at_utc=(datetime.now(timezone.utc)-timedelta(hours=1)).isoformat(),capture_id='old');p.with_suffix('.json').write_text(json.dumps(metadata));config=dict(camera_frame='cam',calibration_id='site1')
            with self.assertRaises(RuntimeError):load_snapshot(p,config)
            load_snapshot(p,config,replay=True)
            with self.assertRaises(RuntimeError):load_snapshot(p,config|dict(calibration_id='different'),replay=True)

    def test_insufficient_visible_surface_is_rejected(self):
        prior=pose(self.meta['pin']['nominal_position'],self.meta['pin']['nominal_quaternion_wxyz'])
        with self.assertRaises(RuntimeError):self.estimator.fit('pin',np.zeros((30,3)),prior)

    def test_dense_small_surface_patch_does_not_establish_ring_center(self):
        prior=pose(self.meta['ring1']['nominal_position'],self.meta['ring1']['nominal_quaternion_wxyz'])
        rng=np.random.default_rng(31)
        patch=np.c_[rng.uniform(.018,.021,500),rng.uniform(-.001,.001,500),np.full(500,-.00775)]
        observed=patch@prior[:3,:3].T+prior[:3,3]
        with self.assertRaises(RuntimeError):self.estimator.fit('ring1',observed,prior)


if __name__=='__main__':unittest.main()
