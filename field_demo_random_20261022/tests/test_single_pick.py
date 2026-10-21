import unittest,tempfile
from pathlib import Path
from field_demo.random_cycle import run
from test_random_cycle import FakeRuntime,recipe
class SinglePickTests(unittest.TestCase):
 def test_real_same_state_machine_stops_holding_one_requested_part(self):
  with tempfile.TemporaryDirectory()as d:
   c=recipe();r=FakeRuntime(Path(d),c);result=run(r,c,single_pick='ring2')
   self.assertEqual(result['state'],'single_pick_held_for_commissioning');self.assertEqual(r.hands['right']['part'],'ring2')
   self.assertFalse(any(e[0]=='clamp'or e[:2]in [('move','insert'),('move','receiver_approach')]for e in r.events))
   self.assertFalse(result['hardware_execution_verified'])
