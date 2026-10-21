import unittest
from types import SimpleNamespace
import numpy as np
from field_demo.camera_rectification import RealSenseRectifier


class SDK:
    @staticmethod
    def rs2_project_point_to_pixel(intr, ray):
        x,y=ray[0]/ray[2],ray[1]/ray[2]
        k1,k2,p1,p2,k3=intr.coeffs;r=x*x+y*y;scale=1+k1*r+k2*r*r+k3*r*r*r
        return [intr.fx*(x*scale+2*p1*x*y+p2*(r+2*x*x))+intr.ppx,
                intr.fy*(y*scale+2*p2*x*y+p1*(r+2*y*y))+intr.ppy]


class RectificationTests(unittest.TestCase):
    def make(self, coeffs=(0,0,0,0,0)):
        intr=SimpleNamespace(width=24,height=18,fx=35.,fy=36.,ppx=11.5,ppy=8.5,model='brown_conrady',coeffs=coeffs)
        yy,xx=np.mgrid[:intr.height,:intr.width];z=np.full(xx.shape,.7)
        xyz=np.stack([(xx-intr.ppx)/intr.fx*z,(yy-intr.ppy)/intr.fy*z,z],axis=2)
        packet=dict(rgb=np.stack([xx,yy,xx+yy],axis=2).astype(np.uint8),depth_m=z,K=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]]),points_camera_image_m=xyz)
        meta=dict(camera_serial='test-real-profile',rectified_registered_depth=False,aligned_to='RGB optical frame',units='m',depth_scale_m=.001,capture_id='frame-17',calibration_sha256='actual-calibration')
        return intr,packet,meta

    def test_identity_retains_pixels_actual_xyz_and_missing_depth(self):
        intr,p,m=self.make();p['depth_m'][4,5]=np.nan;p['points_camera_image_m'][4,5]=np.nan
        rect=RealSenseRectifier(SDK,intr,camera_serial=m['camera_serial'],depth_uncertainty_m=.003)
        q,n=rect.apply(p,m)
        np.testing.assert_array_equal(q['rgb'],p['rgb']);np.testing.assert_allclose(q['points_camera_image_m'],p['points_camera_image_m'],atol=1e-7,equal_nan=True)
        self.assertTrue(np.isnan(q['depth_m'][4,5]));self.assertEqual(n['capture_id'],m['capture_id']);self.assertEqual(n['calibration_sha256'],m['calibration_sha256'])
        self.assertTrue(n['rectified_registered_depth']);self.assertEqual(n['projection_kind'],'rectified_pinhole');self.assertGreaterEqual(n['ray_footprint_padding_px'],1)

    def test_lens_map_uses_actual_projection_and_exposes_empty_edges(self):
        intr,p,m=self.make((.45,0,.005,-.003,0));rect=RealSenseRectifier(SDK,intr,camera_serial=m['camera_serial'],depth_uncertainty_m=.003)
        # Actual organized SDK cloud for this synthetic radial lens. Invert
        # projection numerically, independent of the production remapping.
        from scipy.optimize import root
        for y in range(intr.height):
            for x in range(intr.width):
                sol=root(lambda uv:np.asarray(SDK.rs2_project_point_to_pixel(intr,[uv[0],uv[1],1]))-[x,y],[(x-intr.ppx)/intr.fx,(y-intr.ppy)/intr.fy]);self.assertTrue(sol.success)
                p['points_camera_image_m'][y,x]=[sol.x[0]*.7,sol.x[1]*.7,.7]
        q,n=rect.apply(p,m)
        self.assertTrue(np.isnan(q['depth_m']).any());self.assertGreater(n['maximum_measured_ray_pixel_offset'],0)
        valid=np.isfinite(q['depth_m']);np.testing.assert_allclose(q['points_camera_image_m'][valid],p['points_camera_image_m'][rect.y,rect.x][valid],atol=1e-7)
        self.assertGreaterEqual(n['ray_footprint_padding_px'],np.ceil(n['maximum_measured_ray_pixel_offset']))

    def test_rejects_wrong_profile_or_unregistered_cloud(self):
        intr,p,m=self.make();rect=RealSenseRectifier(SDK,intr,camera_serial=m['camera_serial'],depth_uncertainty_m=.003)
        with self.assertRaises(ValueError):rect.apply(p,dict(m,camera_serial='other'))
        p['points_camera_image_m'][3,4,2]+=.01
        with self.assertRaises(ValueError):rect.apply(p,m)

    def test_no_depth_or_sdk_projection_fallback(self):
        intr,p,m=self.make();p['depth_m'][:]=np.nan
        rect=RealSenseRectifier(SDK,intr,camera_serial=m['camera_serial'],depth_uncertainty_m=.003)
        with self.assertRaises(ValueError):rect.apply(p,m)
        class Unsupported:
            @staticmethod
            def rs2_project_point_to_pixel(*a):raise RuntimeError('unsupported lens')
        with self.assertRaises(ValueError):RealSenseRectifier(Unsupported,intr,camera_serial=m['camera_serial'],depth_uncertainty_m=.003)


if __name__=='__main__':unittest.main()
