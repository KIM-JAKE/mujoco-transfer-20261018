import contextlib
import hashlib
import io
import json
import multiprocessing
import os
import signal
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from field_demo.execution import CAPABILITIES, StopListener, main, request_stop, run_parts
from isaac_port.fieldkit.hand import FrankaHand
from isaac_port.fieldkit.policy import PART_SIDES
from isaac_port.fieldkit.site import ConfiguredCell, Site


class FakePLC:
    def __init__(self, parts):
        self.signals = {part+'_'+suffix: dict(address=index, kind='discrete', active=True)
                        for index, (part, suffix) in enumerate((p, s) for p in parts for s in
                        ('grasped', 'seated', 'clamped', 'retained', 'clamp_command'))}
        self.bits = dict.fromkeys(self.signals, False)
        self.missing_clamp = False
        self.missing_seat = False
        self.reads = []

    def read(self, name):
        self.reads.append(name)
        return self.bits[name]

    def write(self, name, value):
        self.bits[name] = value
        if name.endswith('_clamp_command') and not self.missing_clamp:
            self.bits[name.replace('_clamp_command', '_clamped')] = value

    def wait(self, name, **kwargs):
        if not self.read(name):
            raise TimeoutError('Missing measured clamp')


class FakeHand(FrankaHand):
    def __init__(self, site, side):
        self.site, self.side, self.held = site, side, None

    def move(self, *, grasp=False, **kwargs):
        part = self.site.last_part
        self.site.events.append(('hand', self.side, 'grasp' if grasp else 'open'))
        if grasp:
            self.held = part
            self.site.plc.bits[part+'_grasped'] = True
        elif self.held:
            self.site.plc.bits[self.held+'_retained'] = self.site.plc.bits[self.held+'_clamped']
            self.site.plc.bits[self.held+'_grasped'] = False
            self.held = None
        return dict(is_grasped=bool(self.held))

    def status(self):
        return dict(is_grasped=bool(self.held))

    def stop(self):
        self.site.events.append(('hand_stop', self.side))


class FakeSite:
    def __init__(self, root, parts):
        self.root, self.events, self.stopped = root, [], False
        self.expected_held, self.last_part = {}, None
        self.plc = FakePLC(parts)
        self.config = dict(assembly_commissioned=True, stages={}, robots={}, gripper_actions={})
        self.grippers = {}
        for part in parts:
            side = PART_SIDES[part]
            self.config['robots'][side] = dict(ip='192.0.2.1' if side=='left' else '192.0.2.2')
            self.grippers.setdefault(side, FakeHand(self, side))
            self.config['gripper_actions'][part] = dict(
                open=dict(width_m=.07, speed_m_s=.005),
                grasp=dict(width_m=.04, speed_m_s=.005, force_n=5,
                           epsilon_inner_m=.0005, epsilon_outer_m=.0005),
                release=dict(width_m=.07, speed_m_s=.005))
            for action in ('grasp', 'insert', 'release'):
                key = '.'.join((side, action, part))
                path = root / (key+'.csv')
                path.write_text('time_s,'+','.join('q'+str(j) for j in range(1,8))+'\n0,0,0,0,0,0,0,0\n3,0,0,0,0,0,0,0\n')
                self.config['stages'][key] = dict(csv=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    tool_type='franka_hand',field_path_reviewed=True, robot_ip=self.config['robots'][side]['ip'],
                    F_T_EE_column_major=[0]*16)
        self.config['motion_binary'] = '/offline-validator'

    def path(self, name):
        return Path(name)

    def validate_motion_config(self, side):
        assert side in self.config['robots']

    def require_ready(self):
        if self.stopped:raise RuntimeError('stopped')

    def guarded(self, work):
        return work(lambda:False)

    def event(self, *args, **kwargs):
        pass

    def motion(self, side, mode, target):
        key = Path(target[0]).name.removesuffix('.csv')
        _, action, part = key.split('.')
        self.last_part = part
        self.events.append(('motion', part, action))
        if action == 'insert' and not self.plc.missing_seat:
            self.plc.bits[part+'_seated'] = True

    def stop(self):
        self.stopped = True
        self.events.append(('stop',))


def send_external_stop(config, output):
    try:output.put(request_stop(config))
    except Exception as error:output.put(dict(error=str(error)))


def interrupted_part(root, started, output):
    site = FakeSite(Path(root), ('pin',))
    def motion(*args):
        started.set()
        time.sleep(10)
    site.motion = motion
    signal.signal(signal.SIGINT, signal.default_int_handler)
    with patch('isaac_port.fieldkit.site.subprocess.run'):
        try:run_parts(site, parts=('pin',))
        except KeyboardInterrupt:output.put(dict(stopped=site.stopped, released=('motion','pin','release') in site.events))



class ExecutionTests(unittest.TestCase):
    def test_three_parts_complete_without_button_config_or_actions(self):
        with tempfile.TemporaryDirectory() as folder, patch('isaac_port.fieldkit.site.subprocess.run'):
            site = FakeSite(Path(folder), PART_SIDES)
            result = run_parts(site, full_assembly=True)
            self.assertEqual(result['completed_parts'], list(PART_SIDES))
            self.assertFalse(result['button_verified'])
            self.assertFalse(site.stopped)
            self.assertEqual(sum(e[0]=='motion' for e in site.events), 9)
            for part in PART_SIDES:
                self.assertTrue(site.plc.bits[part+'_retained'])
                self.assertIn(part+'_retained', site.plc.reads)
            self.assertFalse(any('button' in name for name in site.plc.reads))

    def test_missing_clamp_stops_and_never_releases(self):
        with tempfile.TemporaryDirectory() as folder, patch('isaac_port.fieldkit.site.subprocess.run'):
            site = FakeSite(Path(folder), ('ring2',))
            site.plc.missing_clamp = True
            with self.assertRaisesRegex(TimeoutError, 'clamp'):run_parts(site, parts=('ring2',))
            self.assertTrue(site.stopped)
            self.assertTrue(site.grippers['left'].held)
            self.assertNotIn(('motion', 'ring2', 'release'), site.events)

    def test_missing_seat_and_stale_feedback_stop_before_progress(self):
        with tempfile.TemporaryDirectory() as folder, patch('isaac_port.fieldkit.site.subprocess.run'):
            site = FakeSite(Path(folder), ('ring2',))
            site.plc.missing_seat = True
            with self.assertRaisesRegex(RuntimeError, 'inserted'):run_parts(site, parts=('ring2',))
            self.assertNotIn(('motion', 'ring2', 'release'), site.events)
            site = FakeSite(Path(folder), ('ring2',))
            site.plc.bits['ring2_clamped'] = True
            with self.assertRaisesRegex(RuntimeError, 'cleared'):run_parts(site, parts=('ring2',))
            self.assertFalse(any(event[0]=='motion' for event in site.events))

    def test_part_validation_and_execution_ignore_unrelated_config(self):
        with tempfile.TemporaryDirectory() as folder, patch('isaac_port.fieldkit.site.subprocess.run'):
            site = FakeSite(Path(folder), ('pin',))
            run_parts(site, parts=('pin',))
            self.assertEqual(set(site.config['robots']), {'left'})
            self.assertNotIn('button_verified_travel_m', site.config)
            self.assertFalse(any(name.startswith('ring') for name in site.plc.reads))

    def test_offline_validation_does_not_read_plc(self):
        with tempfile.TemporaryDirectory() as folder, patch('isaac_port.fieldkit.site.subprocess.run'):
            site = FakeSite(Path(folder), ('pin',))
            ConfiguredCell(site).validate(profile='three-part', parts=('pin',), check_feedback=False)
            self.assertEqual(site.plc.reads, [])

    def test_live_commands_require_execute_before_site_construction(self):
        with patch('field_demo.execution.Site', side_effect=AssertionError('no device setup')):
            for command in ('part', 'assemble', 'trajectory', 'stop', 'jog', 'cartesian-jog', 'gripper-move'):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main([command, '--config', '/nonexistent'])

    def test_dry_run_is_three_parts_and_no_connections(self):
        output = io.StringIO()
        with patch('socket.socket', side_effect=AssertionError('no socket')), contextlib.redirect_stdout(output):
            self.assertEqual(main(['dry-run']), 0)
        result = json.loads(output.getvalue())
        self.assertEqual(result['completed_parts'], list(PART_SIDES))
        self.assertFalse(any('button' in e.get('part','') for e in result['events']))
        self.assertFalse(CAPABILITIES['camera_guided_motion'])

    def test_optional_button_legacy_contract(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):main(['dry-run','--profile','button'])
        self.assertTrue(json.loads(output.getvalue())['button_verified'])

    def test_actual_site_constructs_only_selected_arm(self):
        with tempfile.TemporaryDirectory() as folder, patch('isaac_port.fieldkit.site.subprocess.run'):
            root=Path(folder)
            staged=FakeSite(root, ('pin',))
            template=Path(__file__).parents[1]/'src/isaac_port/fieldkit/site.franka.example.json'
            config=json.loads(template.read_text())
            config['robots']['left'].update(ip='192.0.2.1',tool_mass_kg=.7,load_mass_kg=0,
                T_cell_base=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            config['commissioning']={key:True for key in config['commissioning']}
            config['plc']['endpoint']['host']='192.0.2.100'
            config['plc']['signals']['cell_stop']['address']=2
            config['plc']['signals'].update(staged.plc.signals)
            config['stages']=staged.config['stages']
            config['gripper_actions']={'pin':staged.config['gripper_actions']['pin']}
            for key in ('calibration_record','commissioning_record'):
                record=root/(key+'.md');record.write_text('measured test record')
                config[key]=str(record)
            path=root/'site.json';path.write_text(json.dumps(config))
            site=Site(path,root/'logs',sides=('left',))
            ConfiguredCell(site).validate(profile='three-part',parts=('pin',),check_feedback=False)
            with site.control_lock():self.assertEqual(tuple(site.grippers),('left',))

    def test_camera_binding_checked_with_stage_context_before_probe_and_after_lock(self):
        from unittest.mock import MagicMock
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);staged=FakeSite(root, ('ring2',))
            site=Site.__new__(Site)
            site.filename=root/'site.json';site.log_dir=root
            site.config=staged.config
            site.config.update(state_binary='/state',motion_binary='/motion',
                               calibration_record='calibration.md',commissioning_record='commissioning.md')
            site.config['robots']['left'].update(tool_mass_kg=.7,load_mass_kg=0)
            site.grippers=staged.grippers
            site.validate_motion_config=MagicMock();site.require_ready=MagicMock();site.event=MagicMock()
            site._control_mutex=threading.RLock();site._control_depth=0
            row=site.config['stages']['left.grasp.ring2']
            row['observation_binding']={'observation':'fresh.json'}
            probe=MagicMock(stdout=json.dumps({'F_T_EE_column_major':[0]*16}))
            with patch('field_demo.motion.validate_binding',side_effect=[None,RuntimeError('stale after lock')]) as binding, \
                 patch('isaac_port.fieldkit.site.subprocess.run',return_value=probe) as read, \
                 patch('isaac_port.fieldkit.site.subprocess.Popen',side_effect=AssertionError('must not start native motion')):
                with self.assertRaisesRegex(RuntimeError,'stale after lock'):
                    site.motion('left','csv',[Path(row['csv'])])
            self.assertEqual(binding.call_count,2)
            self.assertEqual(read.call_count,1)
            for call in binding.call_args_list:
                self.assertIs(call.kwargs['row'],row)
                self.assertEqual(call.kwargs['side'],'left')
                self.assertIs(call.kwargs['site_config'],site.config)
            with patch('field_demo.motion.validate_binding',side_effect=RuntimeError('stale before probe')), \
                 patch('isaac_port.fieldkit.site.subprocess.run',side_effect=AssertionError('must not read state')):
                with self.assertRaisesRegex(RuntimeError,'stale before probe'):
                    site.motion('left','csv',[Path(row['csv'])])

    def test_single_arm_and_two_arm_configurations_share_motion_exclusion(self):
        def locking_site(robots):
            site=Site.__new__(Site)
            site.config={'robots':robots}
            site._control_mutex=threading.RLock();site._control_depth=0
            return site
        single=locking_site({'left':{'ip':'192.0.2.241'}})
        pair=locking_site({'left':{'ip':'192.0.2.241'},'right':{'ip':'192.0.2.242'}})
        with single.control_lock():
            with self.assertRaises(BlockingIOError):
                with pair.control_lock():self.fail('Overlapping arm lock accepted')
        with pair.control_lock():pass

    def test_first_motion_cli_forwards_native_caps_and_selected_side(self):
        from unittest.mock import MagicMock
        for command,arguments in [('jog',['--joint','7','--delta','.01']),
                                  ('cartesian-jog',['--axis','z','--distance-mm','1']),
                                  ('gripper-move',['--width-mm','40','--speed-mm-s','5'])]:
            site=MagicMock()
            with patch('field_demo.execution.Site',return_value=site) as constructor, \
                 patch('field_demo.execution.StopListener'), \
                 patch('field_demo.execution.legacy_execute',return_value=0) as execute:
                self.assertEqual(main([command,'--config','/unused','--side','right','--execute',*arguments]),0)
                self.assertEqual(constructor.call_args.kwargs['sides'],('right',))
                self.assertEqual(execute.call_args.args[0].side,'right')

    def test_ctrl_c_stops_current_part_without_release(self):
        with tempfile.TemporaryDirectory() as folder:
            context=multiprocessing.get_context('spawn')
            started=context.Event();output=context.Queue()
            process=context.Process(target=interrupted_part,args=(folder,started,output))
            process.start()
            try:
                self.assertTrue(started.wait(2))
                os.kill(process.pid,signal.SIGINT)
                result=output.get(timeout=2)
                self.assertTrue(result['stopped']);self.assertFalse(result['released'])
                process.join(timeout=2);self.assertEqual(process.exitcode,0)
            finally:
                if process.is_alive():process.terminate();process.join()

    def test_separate_process_stop_bypasses_active_operation(self):
        class ActiveSite:
            def __init__(self):
                self.config = {'robots':{'left':{'ip':'192.0.2.247'},'right':{'ip':'192.0.2.248'}}}
                self.cancelled = threading.Event()
                self.stopped = threading.Event()
            def stop(self):
                self.cancelled.set();self.stopped.set()
        site = ActiveSite()
        context=multiprocessing.get_context('spawn')
        output = context.Queue()
        with StopListener(site):
            process = context.Process(target=send_external_stop, args=(site.config,output))
            began = time.monotonic();process.start()
            result = output.get(timeout=2)
            process.join(timeout=2)
            self.assertEqual(process.exitcode, 0)
            self.assertTrue(result['stop_requested'])
            self.assertFalse(result['physical_stop_confirmed'])
            self.assertTrue(site.stopped.wait(1))
            self.assertLess(time.monotonic()-began, 2)


if __name__ == '__main__':unittest.main()
