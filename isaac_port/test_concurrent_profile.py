"""Contact on one arm must not stop or discontinuously restart the other."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
import numpy as np
from concurrent_profile import ConcurrentProfile
from fr3_reference_profile import reference_fingerprint


class ConcurrentClockTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        path = Path(self.temp.name)/'clock.npz'
        self.names = [f'{s}_arm_fr3v2_joint{j}' for s in ['left','right'] for j in range(1,8)]
        self.names += [s+'_grip_left_driver_joint' for s in ['left','right']]
        times = np.linspace(0,2,21)
        q = np.tile(np.r_[[0,-.2,0,-1.5,0,1.5,0]*2,0,0],(len(times),1))
        q[:,0] += .02*times
        q[:,9] += .03*times
        schedule = {}
        data = dict(fingerprint=np.array(reference_fingerprint(times,q,self.names,schedule)))
        for side in ['left','right']:
            prefix=side+'_'
            data[prefix+'bounds']=np.array([0.,1.,2.])
            data[prefix+'durations']=np.array([1.,1.])
            for i in range(2):
                data[f'{prefix}clock_{i}_c']=np.array([[6.],[-15.],[10.],[0.],[0.],[float(i)]])
                data[f'{prefix}clock_{i}_x']=np.array([0.,1.])
                data[f'{prefix}clock_{i}_scale']=np.array(1.)
        np.savez_compressed(path,**data)
        path.with_suffix('.json').write_text(json.dumps(dict(planning_fraction=.95)))
        self.profile=ConcurrentProfile(times,q,self.names,schedule,path)

    def tearDown(self):
        self.temp.cleanup()

    def test_one_arm_contact_hold_leaves_other_clock_running(self):
        p=self.profile
        p.gate({'left':0.,'right':0.},{'left':None,'right':None},True)
        for _ in range(100):p.advance(.01)
        p.gate({'left':1.,'right':1.},{'left':'clamp','right':None},True)
        pos,vel,acc=p.advance(.1)
        self.assertEqual(p.last_reference['left'],1.)
        self.assertGreater(p.last_reference['right'],1.)
        np.testing.assert_allclose(vel[:7],0.,atol=1e-12)
        np.testing.assert_allclose(acc[:7],0.,atol=1e-12)
        self.assertGreater(abs(vel[9]),0.)
        p.gate(dict(p.last_reference),{'left':None,'right':None},True)
        p.advance(.1)
        self.assertGreater(p.last_reference['left'],1.)

    def test_unplanned_stop_inside_a_moving_segment_is_rejected(self):
        with self.assertRaises(RuntimeError):
            self.profile.gate({'left':.5,'right':0.},{'left':'force','right':None},True)

    def test_full_gripper_and_arm_targets_use_their_own_reference(self):
        q=self.profile.full_path({'left':.5,'right':1.5})
        self.assertAlmostEqual(q[0],.01)
        self.assertAlmostEqual(q[9],.045)

    def test_crossing_zone_requires_other_arm_actual_reference(self):
        p = self.profile
        p.metadata['coordination_rules'] = [dict(side='right', reference=1., wait_for_side='left', minimum_reference=1.5)]
        previous = {'left':1.4, 'right':.99}
        proposed, holds = p.coordinate(previous, {'left':1.6, 'right':1.01}, {'left':None, 'right':None})
        # A prediction that the other arm will clear is insufficient: it may
        # still stop on a clamp/contact interlock during this control tick.
        self.assertLess(proposed['right'],1.)
        self.assertIsNotNone(holds['right'])
        proposed, holds = p.coordinate({'left':1.5,'right':1.-1e-8}, {'left':1.6,'right':1.01}, {'left':None,'right':None})
        self.assertEqual(proposed['right'],1.01)
        self.assertIsNone(holds['right'])

    def test_measured_handoff_waits_for_stationary_actual_arm(self):
        p=self.profile
        p.metadata.update(conservative_ring_handoff=True,
            settled_handoff_checkpoints={'left':[1.]},
            coordination_rules=[dict(side='right',reference=1.,wait_for_side='left',minimum_reference=1.)])
        previous={'left':1.-1e-8,'right':1.-1e-8};proposed={'left':1.01,'right':1.01}
        with self.assertRaisesRegex(ValueError,'actual joint state'):
            p.coordinate(previous,proposed,{'left':None,'right':None})
        self.assertFalse(p.settled_handoffs)
        q=p.full_path({'left':1.,'right':1.});v=np.zeros_like(q);q[0]+=.01
        with self.assertRaisesRegex(ValueError,'Non-finite'):
            p.coordinate(previous,proposed,{'left':None,'right':None},q*np.nan,v,1.9)
        refs,holds=p.coordinate(previous,proposed,{'left':None,'right':None},q,v,2.)
        self.assertIsNotNone(holds['right']);self.assertFalse(p.settled_handoffs)
        q[0]-=.01;v[0]=.04
        refs,holds=p.coordinate(previous,proposed,{'left':None,'right':None},q,v,2.1)
        self.assertIsNotNone(holds['right']);self.assertFalse(p.settled_handoffs)
        v[0]=0.
        refs,holds=p.coordinate(previous,proposed,{'left':None,'right':None},q,v,2.2)
        self.assertIsNone(holds['right']);self.assertEqual(refs['right'],1.01)
        self.assertTrue(p.handoff_events[-1]['measured'])
        self.assertEqual(p.handoff_events[-1]['time_s'],2.2)


if __name__=='__main__':
    unittest.main()
