import copy
import unittest
import numpy as np
from field_demo.stock_contact import allow_stock_contact, stock_contact_mask, validate_stock_contact_policy


def measurement():
    return dict(verified=True, by='measured_test_fixture', at_utc='2026-10-21T00:00:00+00:00', evidence_file='actual-stop-or-fixture-evidence.json')


def setup():
    region=dict(T_cell_box=np.eye(4).tolist(), half_extents_m=[.15, .2, .1], measurement=measurement())
    policy=dict(schema_version=1, inventory_contract=dict(parts=['ring1', 'ring2'], maximum_count=10, measurement=measurement()),
                bin_interior=region, finger_capsules=[dict(index=8, name='actual_native_finger_left', measurement=measurement())],
                monitoring=dict(wrench_source='actual_fr3_sdk_aggregate_wrench', force_limit_n=5, moment_limit_nm=1,
                                heartbeat_timeout_s=.1, native_driver_sha256='a'*64, stop_test_sha256='b'*64, measurement=measurement()))
    phase=dict(_phase_name='pickup', _contact='grasp', _active_part='ring1', bounded_loose_stock_contact=policy,
               force_monitor=dict(enabled=True, force_limit_n=5, moment_limit_nm=1, measurement=measurement()))
    envelope=dict(tool_capsules_tcp_m=[dict(name='actual_native_finger_left'), dict(name='measured_palm')])
    obstacle=dict(name='unrecognized_ring_surface', T_cell_box=np.eye(4).tolist(), half_extents_m=[.01, .01, .01],
                  source='conservative_registered_RGBD_rays', occupancy_kind='unknown')
    capsule=dict(name='actual_native_finger_left', a=[0, 0, -.02], b=[0, 0, .02], r=.005)
    return obstacle,dict(phase=phase,capsule_index=8,capsule=capsule,envelope=envelope,active_robot=True)


class StockContactTests(unittest.TestCase):
    def test_only_measured_active_finger_stock_contact_is_permitted(self):
        obstacle,args=setup(); self.assertTrue(allow_stock_contact(obstacle,**args))
        obstacle['occupancy_kind']='occupied'; self.assertTrue(allow_stock_contact(obstacle,**args))
        self.assertEqual(obstacle['occupancy_kind'],'occupied')  # No FREE rewrite.

    def test_walls_palm_other_arm_payload_and_wrong_phase_never_exempt(self):
        obstacle,args=setup()
        for source in ['measured_fixture','measured_unknown_scene_enclosure','static_CAD']:
            row=dict(obstacle,source=source); self.assertFalse(allow_stock_contact(row,**args))
        for changed in [dict(capsule_index=0),dict(capsule_index=9,capsule=dict(args['capsule'],name='measured_palm')),
                        dict(active_robot=False),dict(is_payload=True)]:
            self.assertFalse(allow_stock_contact(obstacle,**dict(args,**changed)))
        for key,value in [('_phase_name','carry'),('_contact','insert'),('_active_part','pin')]:
            changed=copy.deepcopy(args); changed['phase'][key]=value
            self.assertFalse(allow_stock_contact(obstacle,**changed))

    def test_outside_part_of_trajectory_is_still_checked(self):
        obstacle,args=setup();args['capsule']=dict(args['capsule'],a=[[0,0,0],[.149,0,0]],b=[[0,0,.02],[.149,0,.02]])
        np.testing.assert_array_equal(stock_contact_mask(obstacle,**args),[True,False])
        self.assertFalse(allow_stock_contact(obstacle,**args))
        outside=copy.deepcopy(obstacle);outside['T_cell_box'][0][3]=.15
        self.assertFalse(stock_contact_mask(outside,**args).any())

    def test_merged_box_boundary_does_not_deny_contained_actual_contact(self):
        obstacle,args=setup()
        # UNKNOWN box extends below the reviewed interior; the complete
        # actual capsule and its contact are still wholly inside it.
        obstacle['T_cell_box'][2][3]=-.08;obstacle['half_extents_m'][2]=.04
        args['capsule']=dict(args['capsule'],a=[0,0,-.07],b=[0,0,-.05])
        self.assertTrue(allow_stock_contact(obstacle,**args))
        self.assertEqual(obstacle['occupancy_kind'],'unknown')
        outside=dict(args['capsule'],a=[0,0,-.099],b=[0,0,-.05])
        self.assertFalse(allow_stock_contact(obstacle,**dict(args,capsule=outside)))
        obstacle['source']='measured_fixture'
        self.assertFalse(allow_stock_contact(obstacle,**args))

    def test_missing_force_heartbeat_inventory_or_identity_fails_closed(self):
        obstacle,args=setup()
        mutations=[lambda p:p['force_monitor'].update(enabled=False),
                   lambda p:p['bounded_loose_stock_contact']['monitoring'].update(heartbeat_timeout_s=.5),
                   lambda p:p['bounded_loose_stock_contact']['monitoring'].update(wrench_source='simulated_contact'),
                   lambda p:p['bounded_loose_stock_contact']['monitoring'].pop('stop_test_sha256'),
                   lambda p:p['bounded_loose_stock_contact']['inventory_contract'].update(parts=['ring1','ring2','pin']),
                   lambda p:p['bounded_loose_stock_contact']['finger_capsules'][0].update(name='measured_palm')]
        for mutate in mutations:
            changed=copy.deepcopy(args);mutate(changed['phase'])
            self.assertFalse(allow_stock_contact(obstacle,**changed))
            with self.assertRaises(ValueError):validate_stock_contact_policy(changed['phase'],changed['envelope'])


if __name__=='__main__':unittest.main()
