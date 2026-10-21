"""Native adapter tests with local fake SDK processes, without hardware."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from .hand import FrankaHand, main
from .teach import capture, compile_stage


class HandTests(unittest.TestCase):
    def fake(self,folder):
        binary=folder/'hand'
        binary.write_text('#!'+sys.executable+'''\nimport json,sys
from pathlib import Path
root=Path(__file__).parent
mode=sys.argv[1]
s=dict(completed=True,width_m=.04,max_width_m=.08,is_grasped=False)
if mode=='status':print(json.dumps(s));sys.exit(0)
print('HEARTBEAT_REQUIRED',flush=True)
for line in sys.stdin:
 if line.strip()=='STOP':(root/'stopped').write_text('yes');sys.exit(3)
 if line.strip()=='READY':
  if (root/'hang').exists():continue
  if mode=='--execute-grasp':s['is_grasped']=not (root/'empty').exists()
  if mode in ('--execute-grasp','--execute-move'):s['width_m']=float(sys.argv[3])
  print(json.dumps(s),flush=True);sys.exit(0)
''');binary.chmod(0o700)
        return FrankaHand(dict(ip='192.0.2.1',binary=binary))

    def test_status_move_grasp_and_explicit_homing(self):
        with tempfile.TemporaryDirectory()as d:
            g=self.fake(Path(d));self.assertFalse(g.status()['is_grasped'])
            self.assertAlmostEqual(g.move(.07,.005)['width_m'],.07)
            self.assertTrue(g.move(.04,.005,grasp=True,force_n=5,epsilon_inner_m=.0005,epsilon_outer_m=.0005)['is_grasped'])
            self.assertTrue(g.activate()['completed'])

    def test_empty_grasp_never_acknowledged(self):
        with tempfile.TemporaryDirectory()as d:
            root=Path(d);g=self.fake(root);(root/'empty').touch()
            with self.assertRaisesRegex(RuntimeError,'confirm object'):
                g.move(.04,.005,grasp=True,force_n=5,epsilon_inner_m=.0005,epsilon_outer_m=.0005)

    def test_timeout_and_interlock_loss_send_stop(self):
        with tempfile.TemporaryDirectory()as d:
            root=Path(d);g=self.fake(root);(root/'hang').touch()
            with self.assertRaises(TimeoutError):g.move(.04,.005,timeout_s=.12)
            self.assertTrue((root/'stopped').exists());(root/'stopped').unlink()
            began=time.monotonic()
            with self.assertRaisesRegex(RuntimeError,'cancelled'):g.move(.04,.005,cancelled=lambda:time.monotonic()-began>.1)
            self.assertTrue((root/'stopped').exists())

    def test_invalid_or_cancelled_commands_do_not_spawn(self):
        g=FrankaHand(dict(ip='192.0.2.1',binary='/no-binary'))
        with patch('subprocess.Popen',side_effect=AssertionError('must not actuate')):
            for params in [dict(width_m=.09,speed_m_s=.005),dict(width_m=.04,speed_m_s=.03),dict(width_m=.04,speed_m_s=.005,grasp=True,force_n=30,epsilon_inner_m=.0005,epsilon_outer_m=.0005)]:
                with self.assertRaises(ValueError):g.move(**params)
            with self.assertRaisesRegex(RuntimeError,'cancelled'):g.move(.04,.005,cancelled=lambda:True)
            with self.assertRaises(SystemExit):main(['homing','--ip','192.0.2.1','--execute'])

    def test_capture_rejects_moving_or_wrong_protocol(self):
        state=dict(read_only=True,server_protocol_version=10,q=[0]*7,q_d=[0]*7,dq=[0]*7,dq_d=[0]*7)
        with patch('subprocess.run')as run:
            run.return_value.stdout=json.dumps(state);self.assertEqual(capture('/fake','192.0.2.1')['robot_ip'],'192.0.2.1')
            state['dq'][0]=.02;run.return_value.stdout=json.dumps(state)
            with self.assertRaisesRegex(RuntimeError,'Stop the robot'):capture('/fake','192.0.2.1')

    def test_compile_registers_hash_and_invalidates_assembly_approval(self):
        with tempfile.TemporaryDirectory()as d:
            root=Path(d);c=json.loads(Path(__file__).with_name('site.franka.example.json').read_text())
            c['robots']['left'].update(ip='192.0.2.1',tool_mass_kg=.7,load_mass_kg=0);c['assembly_commissioned']=True
            site=root/'site.json';site.write_text(json.dumps(c));poses=[]
            for i in range(2):
                s=dict(server_protocol_version=10,q_d=[0,-.4,0,-1.8,0,1.4,.7+i*.01],dq=[0]*7,dq_d=[0]*7,F_T_EE_column_major=[0]*16,m_ee_kg=.7,m_load_kg=0)
                p=root/('pose'+str(i)+'.json');p.write_text(json.dumps(dict(source='m80_read_state',robot_ip='192.0.2.1',state=s)));poses.append(p)
            with patch('subprocess.run')as run:
                with self.assertRaisesRegex(ValueError,'path-reviewed'):compile_stage(site,'left.grasp.ring2',poses,10,False)
                result=compile_stage(site,'left.grasp.ring2',poses,10,True);self.assertEqual(run.call_count,1)
            c=json.loads(site.read_text());self.assertFalse(c['assembly_commissioned'])
            import hashlib
            self.assertEqual(c['stages']['left.grasp.ring2']['sha256'],hashlib.sha256(Path(result['csv']).read_bytes()).hexdigest())
            c['robots']['left']['tool_mass_kg']=.8;site.write_text(json.dumps(c))
            with self.assertRaisesRegex(ValueError,'payload'):compile_stage(site,'left.grasp.ring2',poses,10,True)


if __name__=='__main__':unittest.main()
