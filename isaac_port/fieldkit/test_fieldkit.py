import unittest
from dataclasses import replace
from .policy import DryRunCell, assemble
from .__main__ import preflight


class FieldContractTests(unittest.TestCase):
    def test_normal_contract_orders_clamp_before_release_and_button_last(self):
        cell = DryRunCell()
        assemble(cell)
        for part in ['ring1', 'ring2', 'pin']:
            actions = [x['action'] for x in cell.events if x.get('part') == part]
            self.assertEqual(actions, ['grasp', 'insert', 'clamp', 'release'])
        self.assertEqual(cell.events[-1]['action'], 'press_button')
        self.assertFalse(cell.stopped)

    def test_missing_clamp_ack_stops_without_release_or_button(self):
        class FailedClamp(DryRunCell):
            def observe(self):
                return replace(self.state, clamped=frozenset())
        cell = FailedClamp()
        with self.assertRaises(RuntimeError):
            assemble(cell)
        self.assertTrue(cell.stopped)
        self.assertFalse(any(x['action'] in ['release', 'press_button'] for x in cell.events))

    def test_observed_fault_stops_the_policy(self):
        class Fault(DryRunCell):
            def observe(self):
                return replace(self.state, fault='PLC timeout')
        cell = Fault()
        with self.assertRaises(RuntimeError):
            assemble(cell)
        self.assertEqual(cell.events[-1]['action'], 'stop')

    def test_preflight_does_not_certify_simulation(self):
        report = preflight()
        self.assertFalse(report['hardware_ready'])
        self.assertFalse(report['live_motion_enabled'])
        self.assertTrue(report['blockers'])


if __name__ == '__main__':
    unittest.main()
