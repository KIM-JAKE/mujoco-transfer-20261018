import copy,unittest,numpy as np
from field_demo.floor_constraint import project_inferred_ring_above_floor,support_vertices
from field_demo.stock_departure import certify_issued_departure,departure_contact_mask,departure_support_floor_mask
from test_stock_departure import fixture
class FloorConstraintTests(unittest.TestCase):
 def test_only_small_inferred_penetration_moves_up_and_elevated_part_stays(self):
  v=support_vertices('ring2');T=np.eye(4);T[2,3]=-v[:,2].min()-.00001
  C,r=project_inferred_ring_above_floor(T,'ring2',[0,0,0],[0,0,1],maximum_correction_m=.001,reported_uncertainty_m=.001)
  self.assertAlmostEqual(r['upward_inference_correction_m'],.00001);self.assertGreaterEqual((v@C[:3,:3].T+C[:3,3])[:,2].min(),-1e-12)
  T[2,3]=.02;C,r=project_inferred_ring_above_floor(T,'ring2',[0,0,0],[0,0,1],maximum_correction_m=.001,reported_uncertainty_m=.001);np.testing.assert_array_equal(C,T)
 def test_pixels_exceeding_their_own_reported_bound_are_not_hoisted(self):
  T=np.eye(4)
  with self.assertRaisesRegex(ValueError,'reported'):project_inferred_ring_above_floor(T,'ring2',[0,0,0],[0,0,1],maximum_correction_m=.001,reported_uncertainty_m=.001)
 def test_selected_actual_CAD_can_depart_floor0_without_allowing_finger_floor_contact(self):
  o,a,p,T,body=fixture();spec=p['bounded_loose_stock_departure'];reg=spec['contact_policy']['bin_interior'];reg['T_cell_box'][2][3]=.001
  floor=-.1;T[:,2,3]=floor-support_vertices('ring1')[:,2].min()+np.array([0,.01,.04,.06]);body.update(a=T[:,:3,3],b=T[:,:3,3],r=.01,T_tcp_part=np.eye(4).tolist())
  spec['support_floor']=dict(origin_cell_m=[0,0,floor],normal_cell=[0,0,1],obstacle_name='actual_bin_floor',plane_alignment_tolerance_m=.0001,maximum_inference_correction_m=.001,reported_fit_depth_bound_m=.001,measurement=spec['up_axis_measurement'])
  p['_departure_certificate']=certify_issued_departure(T,p,a['envelope'],selected_payload=body)
  F=np.eye(4);F[2,3]=-.105;box=dict(name='actual_bin_floor',T_cell_box=F.tolist(),half_extents_m=[.2,.2,.005],source='measured_CAD')
  self.assertTrue(departure_support_floor_mask(box,phase=p,capsule=body,envelope=a['envelope'],is_payload=True).all())
  # Exact same support permission cannot reach fingers/palm or another fixed wall.
  self.assertFalse(departure_support_floor_mask(box,phase=p,capsule=body,envelope=a['envelope'],is_payload=False).any())
  self.assertFalse(departure_support_floor_mask(dict(box,name='actual_bin_wall'),phase=p,capsule=body,envelope=a['envelope'],is_payload=True).any())
