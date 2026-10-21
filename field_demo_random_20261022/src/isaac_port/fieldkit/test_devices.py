"""Loopback TCP device tests: no real robot, gripper or PLC connections."""
from contextlib import contextmanager
import json
from pathlib import Path
import socketserver
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from .modbus import ModbusTCP
from .devices import PLC, Robotiq
from .site import Site, ConfiguredCell, rotation, main
from .policy import assemble
from .hand import FrankaHand


class Registers:
    def __init__(self):
        self.bits={1:True,2:False,3:True};self.registers={2000:0x3100,2001:0,2002:0}
        self.writes=[];self.bad_tid=False;self.bad_echo=False;self.drop=False
        self.hold_echo=False;self.grip_object=3;self.coil_feedback={}

    def handle(self, function, payload):
        a,b=struct.unpack('>HH',payload[:4])
        if function in (1,2):
            out=bytearray((b+7)//8)
            for i in range(b):
                if self.bits.get(a+i,False):out[i//8]|=1<<(i%8)
            return bytes([len(out)])+out
        if function in (3,4):
            return bytes([b*2])+struct.pack('>'+str(b)+'H',*[self.registers.get(a+i,0)for i in range(b)])
        if function==5:
            self.bits[a]=b==0xff00;self.writes.append((a,b))
            if a in self.coil_feedback:self.bits[self.coil_feedback[a]]=b==0xff00
            return payload if not self.bad_echo else b'\0'*4
        if function==16:
            values=struct.unpack('>'+str(b)+'H',payload[5:]);self.writes.append((a,values))
            if a==1000:
                data=struct.pack('>3H',*values);action=data[0]
                if action==0:self.registers.update({2000:0,2001:0})
                elif action==1:self.registers[2000]=0x3100
                elif not self.hold_echo:
                    self.registers[2000]=(0x39|(self.grip_object<<6))<<8
                    self.registers[2001]=data[3];self.registers[2002]=data[3]<<8
            return payload[:4]
        return b''


@contextmanager
def server(registers):
    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            def receive(n):
                out=b''
                while len(out)<n:
                    block=self.request.recv(n-len(out))
                    if not block:return out
                    out+=block
                return out
            h=receive(7)
            if len(h)!=7:return
            tid,proto,n,unit=struct.unpack('>HHHB',h);body=receive(n-1)
            if registers.drop:return
            response=bytes([body[0]])+registers.handle(body[0],body[1:])
            frame=struct.pack('>HHHB',tid+int(registers.bad_tid),0,len(response)+1,unit)+response
            # Deliberately fragment the TCP header and payload.
            self.request.sendall(frame[:3]);self.request.sendall(frame[3:])
    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address=True;daemon_threads=True
    s=Server(('127.0.0.1',0),Handler);t=threading.Thread(target=s.serve_forever,daemon=True);t.start()
    try:yield {'host':'127.0.0.1','port':s.server_address[1],'unit':9,'timeout_s':.1}
    finally:s.shutdown();s.server_close();t.join()


def gripper(endpoint):
    return {'endpoint':endpoint,'read_kind':'input','status_address':2000,'command_address':1000}


def plc(endpoint):
    return {'endpoint':endpoint,'signals':{'cell_ready':{'kind':'discrete','address':1,'active':True},
             'cell_fault':{'kind':'discrete','address':2,'active':True},
             'seat':{'kind':'discrete','address':3,'active':True},
             'cell_stop':{'kind':'coil','address':4,'active':True}}}


class DeviceTests(unittest.TestCase):
    def test_fragmented_read_write_and_wrong_transaction(self):
        r=Registers()
        with server(r)as e:
            b=ModbusTCP(**e);self.assertEqual(b.read('discrete',1,3),[True,False,True])
            b.write_coil(5,True);self.assertTrue(r.bits[5])
            r.bad_tid=True
            with self.assertRaisesRegex(RuntimeError,'header'):b.read('input',2000)

    def test_truncation_and_bad_write_ack_reject(self):
        r=Registers()
        with server(r)as e:
            b=ModbusTCP(**e);r.bad_echo=True
            with self.assertRaisesRegex(RuntimeError,'acknowledgement'):b.write_coil(5,True)
            r.drop=True
            with self.assertRaises(ConnectionError):b.read('discrete',1)

    def test_gripper_activation_move_and_stop_do_not_auto_release(self):
        r=Registers()
        with server(r)as e:
            g=Robotiq(gripper(e));self.assertEqual(g.activate()['activation'],3)
            self.assertEqual(g.move(10,16,16)['position'],10)
            g.stop();self.assertEqual(r.writes[-1][1][0],0x0100)
            self.assertNotIn(0x1100,[x[1][0] for x in r.writes if isinstance(x[1],tuple)])

    def test_grasp_requires_object_not_full_closure_and_fault_rejects(self):
        r=Registers()
        with server(r)as e:
            g=Robotiq(gripper(e))
            with self.assertRaisesRegex(RuntimeError,'without confirmed object'):g.move(230,16,16,grasp=True)
            r.grip_object=2;self.assertEqual(g.move(231,16,16,grasp=True)['object'],2)
            r.registers[2001]=9<<8
            with self.assertRaisesRegex(RuntimeError,'free of faults'):g.move(0,16,16)

    def test_stale_echo_cannot_complete_new_gripper_command(self):
        r=Registers();r.hold_echo=True
        with server(r)as e:
            with self.assertRaises(TimeoutError):Robotiq(gripper(e)).move(100,16,16,timeout_s=.06)

    def test_plc_dwell_fault_timeout_and_polarity(self):
        r=Registers()
        with server(r)as e:
            c=plc(e);c['signals']['active_low']={'kind':'discrete','address':2,'active':False}
            p=PLC(c);self.assertTrue(p.read('active_low'))
            began=time.monotonic();p.wait('seat',dwell_s=.04);self.assertGreaterEqual(time.monotonic()-began,.04)
            r.bits[3]=False
            with self.assertRaises(TimeoutError):p.wait('seat',timeout_s=.04)
            r.bits[2]=True
            with self.assertRaisesRegex(RuntimeError,'interlock'):p.wait('seat')

    def test_register_masks_and_invalid_address_types(self):
        r=Registers();r.registers[8]=0xa0
        with server(r)as e:
            c=plc(e);c['signals']['masked']={'kind':'holding','address':8,'mask':0xf0,'active':0xa0}
            self.assertTrue(PLC(c).read('masked'))
            with self.assertRaises(ValueError):ModbusTCP(**e).read('coil',True)
            with self.assertRaises(ValueError):ModbusTCP(**e).write_registers(65535,[1,2])

    def test_cancelled_gripper_wait_never_credits_completion(self):
        r=Registers()
        with server(r)as e:
            with self.assertRaisesRegex(RuntimeError,'cancelled'):
                Robotiq(gripper(e)).move(0,16,16,cancelled=lambda:True)


class SiteTests(unittest.TestCase):
    def config(self, folder, e):
        c=json.loads((Path(__file__).parent/'site.example.json').read_text())
        c['plc']=plc(e);c['grippers']={s:gripper(e)for s in ['left','right']}
        for i,side in enumerate(['left','right']):
            c['robots'][side].update(ip='192.0.2.'+str(10+i),T_cell_base=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],tool_mass_kg=.9,load_mass_kg=0)
        c['commissioning']={k:True for k in c['commissioning']}
        for name in ['calibration.md','commissioning.md']:(folder/name).write_text('Mock record for offline test only')
        c['motion_binary']=str(folder/'motion.py')
        (folder/'motion.py').write_text('#!'+sys.executable+'\nimport sys,time,json\nprint("HEARTBEAT_REQUIRED",flush=True)\nfor line in sys.stdin:\n if line.strip()=="STOP":sys.exit(3)\n if line.strip()=="READY":\n  print(json.dumps({"stage_completed":True,"hardware_ready":False}),flush=True);sys.exit(0)\n')
        (folder/'motion.py').chmod(0o700)
        f=folder/'site.json';f.write_text(json.dumps(c));return f,c

    def test_explicit_execute_required_and_readonly_validation(self):
        r=Registers()
        with server(r)as e,tempfile.TemporaryDirectory()as d:
            f,c=self.config(Path(d),e)
            with patch.object(Site,'motion',side_effect=AssertionError('must not actuate')):
                with self.assertRaises(SystemExit):main(['jog','--config',str(f),'--log-dir',d])
                self.assertEqual(main(['validate','--config',str(f),'--log-dir',d]),0)
            self.assertEqual(r.writes,[])

    def test_subprocess_handshake_success_and_telemetry_path(self):
        r=Registers()
        with server(r)as e,tempfile.TemporaryDirectory()as d:
            f,c=self.config(Path(d),e);s=Site(f,d)
            result=s.motion('left','jog',[7,.01,3])
            self.assertTrue(result['stage_completed']);self.assertTrue(list(Path(d).glob('*.stdout.log')))

    def test_fault_prevents_motion_and_invalid_transform_rejected(self):
        r=Registers();r.bits[2]=True
        with server(r)as e,tempfile.TemporaryDirectory()as d:
            f,c=self.config(Path(d),e);s=Site(f,d)
            with patch('subprocess.Popen',side_effect=AssertionError('must not spawn')):
                with self.assertRaisesRegex(RuntimeError,'interlock'):s.motion('left','jog',[7,.01,3])
            c['robots']['left']['T_cell_base'][0][0]=-1
            with self.assertRaisesRegex(ValueError,'right handed'):rotation(c,'left')

    def test_plc_fault_during_live_subprocess_sends_stop(self):
        r=Registers()
        with server(r)as e,tempfile.TemporaryDirectory()as d:
            f,c=self.config(Path(d),e)
            Path(c['motion_binary']).write_text('#!'+sys.executable+'\nimport sys\nprint("HEARTBEAT_REQUIRED",flush=True)\nfor line in sys.stdin:\n if line.strip()=="STOP":sys.exit(3)\n')
            s=Site(f,d);timer=threading.Timer(.12,lambda:r.bits.update({2:True}));timer.start()
            try:
                with self.assertRaisesRegex(RuntimeError,'interlock'):s.motion('left','jog',[7,.01,3])
            finally:timer.join()
            self.assertTrue(r.bits[4]);self.assertIsNone(s.process)

    def test_serial_cell_requires_fresh_seat_clamp_retention_and_button(self):
        self.serial_cell(False)

    def test_native_serial_cell_with_measured_grasp_and_stationary_release(self):
        self.serial_cell(True)

    def serial_cell(self,native):
        r=Registers()
        with server(r)as e,tempfile.TemporaryDirectory()as d:
            folder=Path(d);f,c=self.config(folder,e);c['button_verified_travel_m']=.0018
            for i,part in enumerate(['ring1','ring2','pin']):
                for k,suffix in enumerate(['grasped','seated','clamped','retained']):
                    c['plc']['signals'][part+'_'+suffix]={'kind':'discrete','address':10+10*i+k,'active':True}
                out=100+i;c['plc']['signals'][part+'_clamp_command']={'kind':'coil','address':out,'active':True}
                r.coil_feedback[out]=12+10*i
                c['gripper_actions'][part]={'grasp':dict(position=200,speed=16,force=16),'release':dict(position=0,speed=16,force=16)}
                side='right'if part=='ring1'else'left'
                for action in ['grasp','insert','release']:
                    key='.'.join([side,action,part]);p=folder/(key+'.csv');p.write_text('time_s,'+','.join('q'+str(i)for i in range(1,8))+'\n0,'+','.join(['0']*7)+'\n2,'+','.join(['0']*7)+'\n' if native else 'offline fixture only')
                    import hashlib
                    c['stages'][key]={'csv':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
            c['plc']['signals']['button_pressed']={'kind':'discrete','address':90,'active':True}
            for name in ['button','button_press','button_release']:
                key='right.approach.'+name;p=folder/(key+'.csv');p.write_text('offline fixture only')
                c['stages'][key]={'csv':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
            if native:
                for side in ('left','right'):c['grippers'][side]=dict(type='franka_hand',binary='/unused')
                for part in ('ring1','ring2','pin'):
                    c['gripper_actions'][part]=dict(open=dict(width_m=.078,speed_m_s=.005),grasp=dict(width_m=.04,speed_m_s=.005,force_n=5,epsilon_inner_m=.0005,epsilon_outer_m=.0005),release=dict(width_m=.078,speed_m_s=.005))
                for key,row in c['stages'].items():row.update(tool_type='franka_hand',field_path_reviewed=True,robot_ip=c['robots'][key.split('.')[0]]['ip'],F_T_EE_column_major=[0]*16)
            f.write_text(json.dumps(c));s=Site(f,d);current={};events=[]
            def motion(side,mode,target):
                key=Path(target[0]).name.removesuffix('.csv');_,action,part=key.split('.')
                events.append(key)
                if action=='grasp':current[side]=part
                if action=='insert':r.bits[c['plc']['signals'][part+'_seated']['address']]=True
                if part=='button_press':r.bits[90]=True
                if part=='button_release':r.bits[90]=False
                return {'stage_completed':True}
            class Grip(FrankaHand if native else object):
                def __init__(self,side):self.side=side
                def status(self):
                    part=current.get(self.side)
                    return dict(is_grasped=bool(part and r.bits.get(c['plc']['signals'][part+'_grasped']['address'],False)))
                def move(self,*,grasp=False,cancelled,**kw):
                    cancelled();part=current.get(self.side)
                    if part is None:return dict(is_grasped=False)
                    r.bits[c['plc']['signals'][part+'_grasped']['address']]=grasp
                    if not grasp:r.bits[c['plc']['signals'][part+'_retained']['address']]=True
                def stop(self):pass
            s.grippers={side:Grip(side)for side in ['left','right']}
            cell=ConfiguredCell(s)
            with patch('subprocess.run')as offline_check:
                cell.validate();self.assertEqual(offline_check.call_count,12)
            with patch.object(s,'motion',side_effect=motion):assemble(cell)
            self.assertTrue(cell.observe().button_verified);self.assertFalse(r.bits[90])
            self.assertEqual(events[-2:],['right.approach.button_press','right.approach.button_release'])
            with self.assertRaisesRegex(RuntimeError,'cleared part feedback'):
                cell.validate()


if __name__=='__main__':unittest.main()
