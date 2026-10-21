"""Exact closed material tests, no collision-hull or shadow waiver."""
import unittest,itertools,json
from datetime import datetime,timezone
import numpy as np
from field_demo.native_hand_solid import models,contains_eroded,material_owner
from field_demo.residual_occupancy import build_occupancy

class ClosedMaterialTests(unittest.TestCase):
 def cube(self,half):
  v=np.array(list(itertools.product([-half,half],repeat=3)));faces=[]
  for axis in range(3):
   for sign in [-1,1]:
    ids=np.flatnonzero(v[:,axis]==sign*half);center=v[ids].mean(0);other=[i for i in range(3)if i!=axis];angle=np.arctan2(v[ids,other[1]],v[ids,other[0]]);ids=ids[np.argsort(angle)]
    for tri in [[ids[0],ids[1],ids[2]],[ids[0],ids[2],ids[3]]]:
     a,b,c=v[tri]
     if np.cross(b-a,c-a)@center<0:tri=tri[::-1]
     faces.append(tri)
  return v,np.array(faces)
 def test_signed_cavity_and_eroded_boundary(self):
  outer,fo=self.cube(.01);inner,fi=self.cube(.003);v=np.concatenate([outer,inner]);f=np.concatenate([fo,fi[:,::-1]+len(outer)])
  inside=contains_eroded(np.array([[.006,0,0],[0,0,0],[.0099,0,0],[.02,0,0]]),v,f,.0005)
  np.testing.assert_array_equal(inside,[True,False,False,False])
 def test_open_main_palm_excluded_actual_source_and_fingers_closed(self):
  m,a=models();self.assertEqual(len(m['palm']),2);self.assertEqual(len(m['finger']),2)
  main=next(v for v in a if v['geometry']=='Part__Feature005_005-mesh');self.assertFalse(main['closed_oriented'])
 def test_actual_finger_material_leaves_all_other_unknown_untouched(self):
  now=datetime.now(timezone.utc);packet=dict(depth_m=np.full((8,8),np.nan),K=np.array([[100,0,4],[0,100,4],[0,0,1.]]),T_cell_camera=np.eye(4));meta=dict(capture_id='synthetic-closed-material',scene_epoch='test',cell_frame='cell',source='synthetic',replay=True,rectified_registered_depth=True,projection_kind='rectified_pinhole',captured_at_utc=now.isoformat(),calibration_sha256='a'*64,source_packet_sha256='b'*64,depth_uncertainty_m=.0002)
  snap=build_occupancy(packet,meta,[[-.04,-.11,.47],[.04,.11,.62]],voxel_size_m=.002,now_utc=now,max_voxels=500000);original=snap.fingerprint;T=np.eye(4);T[2,3]=.5
  state=dict(side='right',T_cell_tcp=T,F_T_EE=np.eye(4),T_flange_hand=np.eye(4),width_m=.064,pose_uncertainty_m=.0002)
  owner,owners,proof=material_owner(snap,[state],.0002);self.assertGreater(np.count_nonzero(owner),100);self.assertEqual(original,snap.fingerprint);self.assertEqual(proof['free_voxels_created'],0)
  # Opaque main palm interior is not claimed from its open visual mesh.
  idx=np.floor((np.array([0,0,.54])-np.asarray(snap.bounds_cell_m)[0])/.002).astype(int);self.assertEqual(owner[tuple(idx)],0)
  with self.assertRaises(ValueError):material_owner(snap,[state],0)
  prior,_,evidence=material_owner(snap,[state],.0002,collision_envelope=True)
  self.assertGreater(np.count_nonzero(prior),np.count_nonzero(owner));self.assertTrue(evidence['collision_envelope_assumption']);self.assertFalse(evidence['physical_solid_certified']);self.assertEqual(original,snap.fingerprint)
  # Current jaw gap and external shadow are not inside either finger.
  gap=np.floor((np.array([0,0,.6])-np.asarray(snap.bounds_cell_m)[0])/.002).astype(int);self.assertEqual(prior[tuple(gap)],0)

if __name__=='__main__':unittest.main()
