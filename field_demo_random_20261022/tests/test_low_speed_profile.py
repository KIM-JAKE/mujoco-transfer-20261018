"""Mock SDK/profile boundaries only. No robot connections."""
import csv
import json
from pathlib import Path
import tempfile
import unittest
from field_demo.low_speed_profile import prepare,load_tracking_profile,activate
from field_demo.commissioning_test import telemetry_tracking


class LowSpeedProfileTests(unittest.TestCase):
    def test_new_profile_keeps_main_and_uses_one_stricter_threshold(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);main=root/'main.json';main.write_text(json.dumps(dict(schema_version=2,units='m',joint_tracking_error_limit_rad=.02)))
            original=main.read_bytes();low=root/'low.json';result=prepare(main,low)
            self.assertEqual(main.read_bytes(),original)
            self.assertEqual(load_tracking_profile(low)['joint_tracking_error_limit_rad'],.005)
            self.assertFalse(result['hardware_tested'])
            self.assertIsNone(json.loads(low.read_text())['tracking_guard_measurement'])
            with self.assertRaises(ValueError):activate(low,{},root/'ready.json')
            with self.assertRaises(ValueError):prepare(main,main)

    def test_actual_complete_tracking_columns_are_required_for_profile_PASS(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'telemetry.csv';keys=[k+str(i)for k in ['q','q_d']for i in range(1,8)]
            with path.open('w')as f:
                w=csv.DictWriter(f,keys);w.writeheader()
                for value in [0.,.0049]:
                    row={k:0. for k in keys};row['q3']=value;w.writerow(row)
            self.assertEqual(telemetry_tracking(path),.0049)
            path.write_text('t_s,O_T_EE12\n0,0\n.4,.00003\n')
            self.assertIsNone(telemetry_tracking(path))
