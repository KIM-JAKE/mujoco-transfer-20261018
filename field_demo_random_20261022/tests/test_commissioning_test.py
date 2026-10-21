"""Synthetic subprocess protocol checks. These never connect to a robot."""
from contextlib import nullcontext
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from field_demo import commissioning_test as c


def state():
    return dict(server_protocol_version=10,robot_mode_name='idle',q=[0.]*7,q_d=[0.]*7,
                dq=[0.]*7,dq_d=[0.]*7,m_ee_kg=.73,m_load_kg=0.,
                O_T_EE_column_major=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.],
                F_T_EE_column_major=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.])


class CommissioningTest(unittest.TestCase):
    def test_explicit_stop_needs_motion_and_stop_return_and_idle(self):
        values=dict(observed=dict(observed_displacement_m=.00003,observed_motion_interval_s=.4),
                    markers=['EXPLICIT_STOP_ACCEPTED','SDK_STOP_RETURNED'],stop_sent_s=10.,finished_s=10.2,returncode=1,after=state())
        self.assertTrue(c.stop_evidence(**values))
        for override in (dict(observed=None),dict(markers=['SDK_STOP_RETURNED']),dict(finished_s=11.01),
                         dict(returncode=0),dict(markers=['SDK_STOP_RETURNED','EXPLICIT_STOP_ACCEPTED']),dict(after=dict(state(),robot_mode_name='reflex')),
                         dict(observed=dict(observed_displacement_m=.000001,observed_motion_interval_s=.4))):
            self.assertFalse(c.stop_evidence(**dict(values,**override)))

    def test_stationary_mass_and_mode_are_actual_sdk_requirements(self):
        row=dict(tool_mass_kg=.73,load_mass_kg=0.)
        c.validate_state(state(),row)
        for key,value in [('m_load_kg',.1),('robot_mode_name','moving'),('dq',[.1]*7),('server_protocol_version',9)]:
            with self.assertRaises(ValueError):c.validate_state(dict(state(),**{key:value}),row)

    def test_partial_csv_rows_cannot_establish_motion(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'telemetry.csv'
            p.write_text('t_s,O_T_EE12,O_T_EE13,O_T_EE14\n0,0,0,0\n.4,0,0,.00003')
            self.assertIsNone(c.telemetry_progress(p))
            p.write_text(p.read_text()+'\n')
            self.assertAlmostEqual(c.telemetry_progress(p)['observed_displacement_m'],.00003)

    def test_bootstrap_native_protocol_sends_STOP_only_after_actual_telemetry(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);binary=root/'native-fixture'
            binary.write_text('''#!/usr/bin/env python3
import pathlib,sys
out=pathlib.Path(sys.argv[7])
print('HEARTBEAT_REQUIRED',flush=True)
for line in sys.stdin:
 if line.strip()=='READY':
  out.write_text('t_s,O_T_EE12,O_T_EE13,O_T_EE14\\n0,0,0,0\\n.4,0,0,.00003\\n')
 if line.strip()=='STOP':
  print('EXPLICIT_STOP_ACCEPTED',flush=True)
  print('SDK_STOP_RETURNED',flush=True)
  sys.exit(1)
sys.exit(2)
''');binary.chmod(0o755)
            cal=root/'calibration';cal.write_text('synthetic measured-calibration fixture')
            cfg=root/'site.json';cfg.write_text('{}')
            class Site:
                filename=cfg;process=None
                config=dict(motion_binary=str(binary),state_binary=str(binary),calibration_record=str(cal))
                grippers={'right':type('Hand',(),{'status':lambda _:dict(is_grasped=False)})()}
                def path(self,p):return Path(p)
                def control_lock(self):return nullcontext()
                def require_ready(self):pass
                def event(self,*args,**kwargs):pass
                def stop(self):
                    self.process.stdin.write('STOP\n');self.process.stdin.flush()
            site=Site();row=dict(ip='192.0.2.1',tool_mass_kg=.73,load_mass_kg=0.)
            with patch.object(c,'validate',return_value=row),patch.object(c,'read_state',return_value=state()):
                report,record=c.execute(site,'right',root/'trial')
            self.assertEqual(report['state'],'PASS')
            self.assertEqual(json.loads(record.read_text())['native_returncode'],1)
            self.assertIsNone(site.process)


class LowTrackingCommissioningTest(unittest.TestCase):
    def test_first_test_dispatch_uses_selected_profile_and_requires_its_telemetry(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);binary=root/'native-fixture'
            binary.write_text("#!/usr/bin/env python3\nimport pathlib,sys,json\nr=pathlib.Path(__file__).parent\nif sys.argv[-2:]!=['--tracking-error-rad','0.005']:raise RuntimeError('Profile threshold not passed to native executor')\n(r/'actual-dispatch-args.json').write_text(json.dumps(sys.argv))\nout=pathlib.Path(sys.argv[7]);print('HEARTBEAT_REQUIRED',flush=True)\nfor line in sys.stdin:\n if line.strip()=='READY':\n  keys=['t_s','O_T_EE12','O_T_EE13','O_T_EE14']+[k+str(i)for k in ['q','q_d']for i in range(1,8)]\n  out.write_text(','.join(keys)+'\\n'+','.join(map(str,[0,0,0,0]+[0]*14))+'\\n'+','.join(map(str,[.4,0,0,.00003]+[.001]*7+[0]*7))+'\\n')\n if line.strip()=='STOP':\n  print('EXPLICIT_STOP_ACCEPTED',flush=True);print('SDK_STOP_RETURNED',flush=True);sys.exit(1)\nsys.exit(2)\n");binary.chmod(0o755)
            cal=root/'calibration';cal.write_text('MOCK fixture only');cfg=root/'site.json';cfg.write_text('{}');profile=root/'low.json';profile.write_text(json.dumps(dict(schema_version=2,units='m',joint_tracking_error_limit_rad=.005)))
            class Site:
                filename=cfg;process=None
                config=dict(motion_binary=str(binary),state_binary=str(binary),calibration_record=str(cal))
                grippers={'right':type('Hand',(),{'status':lambda _:dict(is_grasped=False)})()}
                def path(self,p):return Path(p)
                def control_lock(self):return nullcontext()
                def require_ready(self):pass
                def event(self,*args,**kwargs):pass
                def stop(self):self.process.stdin.write('STOP\n');self.process.stdin.flush()
            row=dict(ip='192.0.2.1',tool_mass_kg=.73,load_mass_kg=0.)
            with patch.object(c,'validate',return_value=row),patch.object(c,'read_state',return_value=state()):
                report,_=c.execute(Site(),'right',root/'trial',tracking_profile=profile)
            self.assertTrue(report['stop_tested']);self.assertEqual(report['joint_tracking_error_limit_rad'],.005)
            self.assertEqual(report['observed_max_tracking_error_rad'],.001)
            self.assertEqual(report['tracking_profile']['path'],str(profile))

if __name__=='__main__':unittest.main()
