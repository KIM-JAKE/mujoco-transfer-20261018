import unittest
from threading import Event
from dataclasses import replace
from .policy import DryRunCell, assemble, assemble_concurrent
from .__main__ import preflight


class FieldContractTests(unittest.TestCase):
    def test_left_pin_can_start_while_right_ring_is_still_being_inserted(self):
        class OverlapCell(DryRunCell):
            def __init__(self):
                super().__init__();self.right_inserting=Event();self.left_pin_started=Event()
            def execute_parallel(self,actions):
                if ('right','insert','ring1')in actions:
                    self.right_inserting.set()
                    if not self.left_pin_started.wait(2):raise RuntimeError('Unwanted global ring barrier')
                if ('left','grasp','pin')in actions:
                    if not self.right_inserting.wait(2):raise RuntimeError('Right lane did not run')
                    self.left_pin_started.set()
                super().execute_parallel(actions)
        cell=OverlapCell();assemble_concurrent(cell)
        self.assertTrue(cell.left_pin_started.is_set())
        self.assertTrue(cell.observe().button_verified)
        self.assertEqual(cell.events[-1]['action'],'press_button')

    def test_button_command_return_without_measured_confirmation_stops(self):
        class NoButtonAck(DryRunCell):
            def press_button(self,**parameters):
                self.events.append(dict(action='button_command_only'))
        cell=NoButtonAck()
        with self.assertRaisesRegex(RuntimeError,'button travel'):
            assemble_concurrent(cell)
        self.assertTrue(cell.stopped)
        self.assertFalse(cell.observe().button_verified)

    def test_concurrent_clamp_fault_stops_both_lanes_without_press(self):
        class FailedClamp(DryRunCell):
            def observe(self):return replace(super().observe(),clamped=frozenset())
        cell=FailedClamp()
        with self.assertRaises(RuntimeError):assemble_concurrent(cell)
        self.assertTrue(cell.stopped)
        self.assertFalse(any(x['action']=='press_button'for x in cell.events))

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
