"""Field commissioning CLI: read-only inspection by default, explicit low-speed tests.

No Isaac or GPU dependency. Modbus TCP gateways must expose the documented
register mapping; Ethernet/IP, Profinet and serial-only devices need adapters.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager, ExitStack
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import ipaddress
import math
import os
from pathlib import Path
import select
import subprocess
import threading
import time
from .devices import PLC, Robotiq
from .hand import FrankaHand
from .policy import TaskState, assemble


def finite(value):
    if type(value) not in (int,float) or not math.isfinite(value):
        raise ValueError('Expected finite number')
    return float(value)


def rotation(config, side):
    m = config['robots'][side]['T_cell_base']
    if len(m)!=4 or any(len(row)!=4 for row in m):
        raise ValueError('T_cell_base must be a 4x4 matrix')
    a = [[finite(v) for v in row] for row in m]
    if any(abs(a[3][i]-v)>1e-9 for i,v in enumerate([0,0,0,1])):
        raise ValueError('Invalid transform last row')
    r = [row[:3] for row in a[:3]]
    for i in range(3):
        for j in range(3):
            if abs(sum(r[k][i]*r[k][j] for k in range(3))-(i==j))>1e-6:
                raise ValueError('Transform rotation is not orthonormal')
    det=sum(r[0][i]*(r[1][(i+1)%3]*r[2][(i+2)%3]-r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
    if abs(det-1)>1e-6:
        raise ValueError('Transform rotation must be right handed')
    return r


class Site:
    def __init__(self, filename, log_dir, *, sides=None):
        self.filename = Path(filename).resolve()
        self.config = json.loads(self.filename.read_text())
        self.log_dir = Path(log_dir).resolve()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.cancelled = threading.Event()
        self.expected_held = {}
        cell_mode=self.config.get('cell_io_mode','actual_PLC')
        if cell_mode not in ('actual_PLC','SDK_stop'):raise ValueError('Explicit actual_PLC or SDK_stop cell I/O required')
        self.plc = PLC(self.config['plc']) if cell_mode=='actual_PLC' else None
        self.grippers = {}
        self.active_sides = tuple(sides) if sides is not None else tuple(self.config['grippers'])
        for side in self.active_sides:
            row = self.config['grippers'][side]
            if row.get('type','robotiq')=='franka_hand':
                self.grippers[side]=FrankaHand(dict(ip=self.config['robots'][side]['ip'],binary=self.path(row['binary'])))
            elif row.get('type','robotiq')=='robotiq':self.grippers[side]=Robotiq(row)
            else:raise ValueError('Unknown gripper type')
        self.process = None
        self._control_mutex = threading.RLock()
        self._control_depth = 0
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        self.audit_path=self.log_dir/('session-'+stamp+'.jsonl')
        self.event('opened',configuration_sha256=hashlib.sha256(self.filename.read_bytes()).hexdigest())

    def event(self, event, **values):
        row=dict(event=event,utc=datetime.now(timezone.utc).isoformat(),monotonic_s=time.monotonic(),**values)
        with self.audit_path.open('a') as log:log.write(json.dumps(row)+'\n')

    @contextmanager
    def control_lock(self):
        with self._control_mutex:
            if self._control_depth:
                self._control_depth += 1
                try:yield
                finally:self._control_depth -= 1
                return
            addresses=sorted(r['ip'] for r in self.config['robots'].values() if r.get('ip'))
            key='|'.join(addresses)
            # Keep the legacy pair lock and lock each address as well: a
            # single-arm configuration must not overlap a two-arm session.
            names=['m80-cell-'+hashlib.sha256(key.encode()).hexdigest()[:24]]
            names.extend('m80-arm-'+hashlib.sha256(ip.encode()).hexdigest()[:24] for ip in addresses)
            with ExitStack() as stack:
                for name in names:
                    lock=stack.enter_context((Path('/tmp')/(name+'.lock')).open('a'))
                    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                self._control_depth=1
                try:yield
                finally:self._control_depth=0

    def path(self, name):
        p = Path(name)
        return p.resolve() if p.is_absolute() else (self.filename.parent/p).resolve()

    def validate_motion_config(self, side):
        c = self.config
        if c.get('schema_version')!=1:
            raise ValueError('Unsupported site configuration schema')
        for key in ['workspace_checked','stop_tested','tool_and_payload_checked','single_arm_test_area_clear']:
            if c['commissioning'].get(key) is not True:
                raise RuntimeError('Field check required: '+key)
        for key in ['calibration_record','commissioning_record']:
            p = self.path(c[key])
            if not p.is_file() or not p.read_bytes().strip():
                raise RuntimeError('Nonempty field record required: '+key)
        if c.get('cell_io_mode','actual_PLC')=='actual_PLC':
            stop=c['plc']['signals']['cell_stop']
            from .modbus import integer
            if stop['kind']!='coil' or type(stop['active']) is not bool:raise ValueError('Dedicated PLC stop coil mapping required')
            integer(stop['address'],0,65535)
        elif c.get('cell_io_mode')!='SDK_stop':raise ValueError('Explicit actual PLC or tested native SDK stop path required')
        r = c['robots'][side]
        rotation(c, side)
        for key in ['tool_mass_kg','load_mass_kg']:
            if finite(r[key])<0:
                raise ValueError('Invalid payload mass')
        if not r['ip'] or not isinstance(r['ip'],str):
            raise ValueError('Robot IP required')
        ipaddress.ip_address(r['ip'])
        if c['robots'].get('left',{}).get('ip') and c['robots'].get('left',{}).get('ip')==c['robots'].get('right',{}).get('ip'):
            raise ValueError('Two arm IPs must be distinct')

    def require_ready(self):
        ready=not self.cancelled.is_set() and (self.plc.ready() if self.plc is not None else self.config.get('cell_io_mode')=='SDK_stop')
        if ready:
            for side,part in tuple(self.expected_held.items()):
                if self.config.get('part_evidence_source')=='RGBD_SDK':
                    if self.grippers[side].status().get('is_grasped')is not True:ready=False;break
                elif self.plc is None or not self.plc.read(part+'_grasped'):ready=False;break
        self.event('plc_ready_observation',ready=ready)
        if not ready:
            raise RuntimeError('Cell is stopped or PLC readiness/fault interlock failed')

    def stop(self):
        self.cancelled.set()
        # No automatic opening or unclamping: retain a supported workpiece.
        errors=[]
        process=self.process
        if process and process.poll() is None:
            try:process.stdin.write('STOP\n'); process.stdin.flush()
            except (BrokenPipeError,OSError,ValueError):pass
        def stop_device(name, work):
            try:work()
            except Exception as e:errors.append({'device':name,'error':str(e)})
        with ThreadPoolExecutor(max_workers=3) as pool:
            preserve=self.config.get('abort_hand_policy')=='preserve_completed_grasp'
            tasks=[] if preserve else [pool.submit(stop_device,side,g.stop) for side,g in self.grippers.items()]
            if preserve:
                # Stop the active arm via heartbeat above; do not cancel a completed native grasp.
                # This is a command policy, not a guarantee of friction/power/retention.
                for side,g in self.grippers.items():
                    try:self.event('abort_hand_command_preserved',side=side,SDK_state=g.status(),retention_guaranteed=False,resume_allowed=False)
                    except Exception as e:errors.append({'device':side,'error':str(e)})
            if self.plc is not None and 'cell_stop' in self.plc.signals:
                tasks.append(pool.submit(stop_device,'PLC',lambda:self.plc.write('cell_stop', True)))
            for f in tasks:f.result()
        try:self.event('stop_requested',device_errors=errors)
        except OSError:pass
        return errors

    def guarded(self, work):
        def cancelled():
            self.require_ready()
            return False
        self.require_ready()
        try:return work(cancelled)
        except BaseException:
            self.stop()
            raise

    def motion(self, side, mode, target):
        self.validate_motion_config(side)
        self.require_ready()
        r = self.config['robots'][side]
        binding=None
        dynamic_row=None
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        output = self.log_dir/(side+'-'+stamp)
        if mode=='csv' and isinstance(self.grippers[side],FrankaHand):
            row=next((row for row in self.config['stages'].values() if self.path(row['csv'])==Path(target[0]).resolve()),None)
            if row and hashlib.sha256(Path(target[0]).read_bytes()).hexdigest()!=row['sha256']:
                raise RuntimeError('Field trajectory hash mismatch')
            if row and row.get('planning_source')=='dynamic_field_envelope':
                if row.get('field_path_reviewed') is not False:
                    raise RuntimeError('Dynamic phases must not impersonate a taught path review')
                from capx_autoring.field_demo.random_planner import validate_runtime_stage
                validate_runtime_stage(row,side,site=self)
                dynamic_row=row
            if not row or row.get('tool_type')!='franka_hand' or (dynamic_row is None and row.get('field_path_reviewed') is not True) or row.get('robot_ip')!=r['ip']:
                raise RuntimeError('A reviewed, taught native Hand trajectory is required')
            binding=row.get('observation_binding')
            if binding is not None:
                from capx_autoring.field_demo.motion import validate_binding
                validate_binding(binding,self.filename.parent,row=row,side=side,site_config=self.config)
            probe=subprocess.run([str(self.path(self.config['state_binary'])),r['ip']],check=True,capture_output=True,text=True,timeout=10)
            frame=json.loads(probe.stdout)['F_T_EE_column_major']
            expected=row['F_T_EE_column_major']
            if len(frame)!=16 or len(expected)!=16 or any(abs(finite(a)-finite(b))>1e-8 for a,b in zip(frame,expected)):
                raise RuntimeError('Measured TCP differs from taught trajectory')
        args = [str(self.path(self.config['motion_binary'])), '--execute-'+mode, r['ip'],
                str(r['tool_mass_kg']),str(r['load_mass_kg']),
                str(self.path(self.config['calibration_record'])),str(self.path(self.config['commissioning_record'])),
                str(output.with_suffix('.csv')), *map(str,target)]
        if dynamic_row is not None:
            tracking=finite(dynamic_row.get('joint_tracking_error_limit_rad',.02))
            if not .0001<=tracking<=.02:raise ValueError('Invalid dynamic tracking guard')
            args += ['--tracking-error-rad',str(tracking)]
        self.event('motion_requested',side=side,mode=mode,target=list(map(str,target)),telemetry=str(output.with_suffix('.csv')))
        # One shared cell lock, held across subprocess and all I/O monitoring.
        # Never launch simultaneous independently planned stages on the two arms.
        with self.control_lock():
            if dynamic_row is not None:
                from capx_autoring.field_demo.random_planner import validate_runtime_stage
                validate_runtime_stage(dynamic_row,side,site=self)
            if binding is not None:
                from capx_autoring.field_demo.motion import validate_binding
                validate_binding(binding,self.filename.parent,row=row,side=side,site_config=self.config)
            with output.with_suffix('.stderr.log').open('w') as err:
                self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                                stderr=err, text=True, bufsize=1)
                began = time.monotonic(); armed=False; lines=[]
                try:
                    while self.process.poll() is None:
                        if time.monotonic()-began>90:
                            raise TimeoutError('Commissioning process timeout')
                        if select.select([self.process.stdout], [], [], .01)[0]:
                            line = self.process.stdout.readline()
                            if line:
                                lines.append(line)
                                if line.strip()=='HEARTBEAT_REQUIRED':armed=True
                                elif line.startswith('{'):armed=False
                        self.require_ready()
                        if armed:
                            self.process.stdin.write('READY\n'); self.process.stdin.flush()
                        time.sleep(.02)
                    lines.extend(self.process.stdout.readlines())
                    output.with_suffix('.stdout.log').write_text(''.join(lines))
                    if self.process.returncode:
                        raise RuntimeError('Robot stage failed; see '+str(output.with_suffix('.stderr.log')))
                    results=[json.loads(s) for s in lines if s.startswith('{')]
                    if not results or results[-1].get('stage_completed') is not True:
                        raise RuntimeError('Missing completed-stage acknowledgement')
                    self.event('motion_completed',side=side,result=results[-1])
                    result=dict(results[-1],actual_driver_file=str(self.path(self.config['motion_binary'])),actual_driver_sha256=hashlib.sha256(self.path(self.config['motion_binary']).read_bytes()).hexdigest(),actual_completed_at_utc=datetime.now(timezone.utc).isoformat())
                    if output.with_suffix('.csv').is_file():result.update(actual_telemetry_file=str(output.with_suffix('.csv')),actual_telemetry_sha256=hashlib.sha256(output.with_suffix('.csv').read_bytes()).hexdigest())
                    return result
                except BaseException:
                    self.stop()
                    try:self.process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        # SIGTERM is handled by native monitor; closed stdin also
                        # drops the native gate. Hardware stop remains independent.
                        self.process.stdin.close();self.process.terminate()
                        self.process.wait(timeout=5)
                    raise
                finally:
                    self.process.stdin.close();self.process.stdout.close();self.process=None

    def status(self):
        # All operations here are reads; gripper activation is a separate command.
        robots={}
        probe=str(self.path(self.config['state_binary']))
        for side in self.active_sides:
            r=self.config['robots'][side]
            p=subprocess.run([probe,r['ip']],capture_output=True,text=True,timeout=10)
            if p.returncode:raise RuntimeError(side+' read failed: '+p.stderr)
            robots[side]=json.loads(p.stdout)
        return dict(robots=robots,grippers={s:g.status() for s,g in self.grippers.items()},cell_ready=self.plc.ready(),
                    plc={name:self.plc.read(name) for name in self.plc.signals if name!='cell_stop' and self.plc.signals[name]['kind']!='coil'},
                    hardware_ready=False)


class ConfiguredCell:
    """Sequential CaP adapter for site-validated CSV stages and real I/O.

    Optimized concurrent replay and automatic contact search are not enabled.
    The conservative stage recipe must contain the measured approach, seating,
    withdrawal and button motion; signals are freshly read after each action.
    """
    def __init__(self, site):
        self.site=site;self.state=TaskState();self.released=set();self.held={}
        self.parts=('ring1','ring2','pin')

    def validate(self, profile='button', parts=None, *, check_feedback=True):
        from .policy import PART_SIDES
        if profile not in ('three-part','button'):raise ValueError('Unknown field profile')
        self.parts=tuple(parts) if parts is not None else tuple(PART_SIDES)
        if not self.parts or len(set(self.parts))!=len(self.parts) or any(p not in PART_SIDES for p in self.parts):
            raise ValueError('Select unique known parts')
        if profile=='button' and set(self.parts)!=set(PART_SIDES):raise ValueError('Button profile requires all three parts')
        required=['right.approach.button','right.approach.button_press','right.approach.button_release'] if profile=='button' else []
        for part in self.parts:
            side=PART_SIDES[part]
            required.extend('.'.join([side,action,part])for action in ('grasp','insert','release'))
            for suffix in ['grasped','seated','clamped','retained','clamp_command']:
                row=self.site.plc.signals[part+'_'+suffix]
                if type(row.get('address'))is not int:raise ValueError('PLC mapping incomplete: '+part+'_'+suffix)
            native=isinstance(self.site.grippers[side],FrankaHand)
            for action in (('open','grasp','release') if native else ('grasp','release')):
                row=self.site.config['gripper_actions'][part][action]
                if native:FrankaHand.validate_action(row,grasp=action=='grasp')
                else:
                    from .modbus import integer
                    for key in ('position','speed','force'):integer(row[key],0,255)
            if native:
                actions=self.site.config['gripper_actions'][part]
                for action in ('open','release'):
                    if actions[action]['width_m']<=actions['grasp']['width_m']+actions['grasp']['epsilon_outer_m']:
                        raise ValueError('Open/release width must clear the grasped object')
            for suffix in ['grasped','seated','clamped','retained']:
                if check_feedback and self.site.plc.read(part+'_'+suffix):
                    raise RuntimeError('Cycle must start with cleared part feedback: '+part+'_'+suffix)
        if profile=='button':
            travel=finite(self.site.config['button_verified_travel_m'])
            if travel<.0018:raise ValueError('Button feedback travel calibration is required')
            self.site.plc.signals['button_pressed']
            if check_feedback and self.site.plc.read('button_pressed'):
                raise RuntimeError('Button already pressed before new cycle')
        for key in required:
            row=self.site.config['stages'][key];p=self.site.path(row['csv'])
            side=key.split('.')[0]
            if isinstance(self.site.grippers[side],FrankaHand):
                if row.get('tool_type')!='franka_hand' or row.get('field_path_reviewed') is not True or row.get('robot_ip')!=self.site.config['robots'][side]['ip']:
                    raise RuntimeError('Native Hand field trajectory approval missing: '+key)
                frame=row.get('F_T_EE_column_major',[])
                if len(frame)!=16 or any(not math.isfinite(finite(x)) for x in frame):
                    raise ValueError('Native Hand trajectory needs the measured TCP frame')
                if '.release.' in key:
                    import csv
                    with p.open() as stream:points=list(csv.DictReader(stream))
                    if not points or any(abs(finite(float(point['q'+str(j)]))-finite(float(points[0]['q'+str(j)])))>1e-6 for point in points for j in range(1,8)):
                        raise RuntimeError('Release stage must hold position while the part is clamped')
            if hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:
                raise RuntimeError('Field trajectory hash mismatch: '+key)
            subprocess.run([str(self.site.path(self.site.config['motion_binary'])),'--validate-csv',str(p)],check=True,capture_output=True,text=True,timeout=15)
        for side in set(PART_SIDES[p] for p in self.parts):self.site.validate_motion_config(side)

    def execute_parallel(self, actions):
        # The sequential policy may request two arms together; serialize them
        # until a field validated shared-zone scheduler is available.
        for side,action,part in actions:
            self.site.require_ready()
            if action=='clamp':
                self.site.guarded(lambda cancelled:self._clamp(part,cancelled))
            elif action in ('grasp','insert','release','approach'):
                key='.'.join([side,action,part]);row=self.site.config['stages'][key]
                p=self.site.path(row['csv'])
                if hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:
                    raise RuntimeError('Field trajectory changed: '+key)
                if action in ('insert','release') and isinstance(self.site.grippers[side],FrankaHand):
                    if self.held.get(side)!=part or not self.site.grippers[side].status()['is_grasped']:
                        raise RuntimeError('Native Hand lost the current workpiece before '+action)
                if action=='release':
                    if not self.site.plc.read(part+'_seated') or not self.site.plc.read(part+'_clamped'):
                        raise RuntimeError('Release requires measured seat and closed clamp')
                if action=='grasp' and isinstance(self.site.grippers[side],FrankaHand):
                    g=self.site.grippers[side];opening=self.site.config['gripper_actions'][part]['open']
                    self.site.guarded(lambda cancelled:g.move(**opening,cancelled=cancelled))
                self.site.motion(side,'csv',[p])
                if action in ('grasp','release'):
                    g=self.site.grippers[side];c=self.site.config['gripper_actions'][part][action]
                    if action=='release':self.site.expected_held.pop(side,None)
                    feedback=self.site.guarded(lambda cancelled:g.move(**c,grasp=action=='grasp',cancelled=cancelled))
                    self.site.event('gripper_completed',side=side,part=part,action=action,feedback=feedback)
                    if action=='release':
                        self.released.add(part);self.held.pop(side,None)
                    elif isinstance(g,FrankaHand):
                        self.held[side]=part;self.site.expected_held[side]=part
            else:raise ValueError('Unknown field action')

    def _clamp(self,part,cancelled):
        self.site.plc.write(part+'_clamp_command',True)
        self.site.plc.wait(part+'_clamped',cancelled=cancelled)

    def observe(self):
        self.site.require_ready();p=self.site.plc
        grasped=set();inserted=set();clamped=set();released=set()
        from .policy import PART_SIDES
        for part in self.parts:
            side=PART_SIDES[part]
            measured=p.read(part+'_grasped')
            if isinstance(self.site.grippers[side],FrankaHand):
                measured=measured and self.held.get(side)==part and self.site.grippers[side].status()['is_grasped']
            if measured:grasped.add(part)
            if p.read(part+'_seated'):inserted.add(part)
            if p.read(part+'_clamped'):clamped.add(part)
            if part in self.released and part in clamped and p.read(part+'_retained'):released.add(part)
        return TaskState(frozenset(grasped),frozenset(inserted),frozenset(clamped),frozenset(released),
                         button_verified=self.state.button_verified)

    def press_button(self,*,minimum_travel_m,dwell_s):
        if set(self.observe().released)!={'ring1','ring2','pin'}:
            raise RuntimeError('Parts not released and retained')
        if self.site.config['button_verified_travel_m']<minimum_travel_m:
            raise RuntimeError('Configured button signal travel calibration is insufficient')
        if self.site.plc.read('button_pressed'):
            raise RuntimeError('Button was pressed before commanded press stage')
        self.execute_parallel([('right','approach','button_press')])
        self.site.guarded(lambda cancelled:self.site.plc.wait('button_pressed',dwell_s=dwell_s,cancelled=cancelled))
        self.execute_parallel([('right','approach','button_release')])
        self.site.guarded(lambda cancelled:self.site.plc.wait('button_pressed',expected=False,cancelled=cancelled))
        self.state=TaskState(button_verified=True)

    def stop(self):
        return self.site.stop()


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['validate','status','jog','cartesian-jog','gripper-activate','gripper-move','plc-test','trajectory','assemble'])
    p.add_argument('--config',type=Path,required=True);p.add_argument('--log-dir',type=Path,required=True)
    p.add_argument('--execute',action='store_true');p.add_argument('--side',choices=['left','right'],default='left')
    p.add_argument('--joint',type=int,default=7);p.add_argument('--delta',type=float,default=.01)
    p.add_argument('--duration',type=float,default=3.);p.add_argument('--axis',choices=['x','y','z'],default='z')
    p.add_argument('--distance-mm',type=float,default=1.);p.add_argument('--position',type=int,default=0)
    p.add_argument('--speed',type=int,default=16);p.add_argument('--force',type=int,default=16)
    p.add_argument('--width-mm',type=float);p.add_argument('--speed-mm-s',type=float,default=5)
    p.add_argument('--stage');p.add_argument('--output-signal');p.add_argument('--feedback-signal')
    a=p.parse_args(argv);site=Site(a.config,a.log_dir)
    if a.command=='validate':
        for side in ['left','right']:site.validate_motion_config(side)
        result=dict(state='configuration_checked',hardware_ready=False,live_motion_enabled=False)
    elif a.command=='status':result=site.status()
    else:
        if not a.execute:p.error('Actuation requires --execute; status and validate are read-only')
        with site.control_lock():
            return execute(a,site)
    print(json.dumps(result,indent=2))
    return 0


def execute(a,site):
    result=None
    try:
        if a.command=='trajectory':
            if not a.stage or a.stage.split('.')[0]!=a.side:raise ValueError('--stage must belong to --side')
            row=site.config['stages'][a.stage];path=site.path(row['csv'])
            if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:raise RuntimeError('Trajectory hash mismatch')
            if '.release.' in a.stage:
                import csv
                with path.open() as stream:points=list(csv.DictReader(stream))
                if not points or any(abs(finite(float(point['q'+str(j)]))-finite(float(points[0]['q'+str(j)])))>1e-6 for point in points for j in range(1,8)):
                    raise RuntimeError('Release path must hold position while clamped')
            result=site.motion(a.side,'csv',[path])
        elif a.command=='jog':result=site.motion(a.side,'jog',[a.joint,a.delta,a.duration])
        elif a.command=='cartesian-jog':
            r=rotation(site.config,a.side);axis=['x','y','z'].index(a.axis);distance=finite(a.distance_mm)/1000
            if not 0<abs(distance)<=.005:raise ValueError('Cartesian jog must be nonzero and <=5mm')
            result=site.motion(a.side,'cartesian',[r[axis][k]*distance for k in range(3)]+[a.duration])
        elif a.command=='gripper-activate':result=site.guarded(lambda cancel:site.grippers[a.side].activate(cancelled=cancel))
        elif a.command=='gripper-move':
            g=site.grippers[a.side]
            if isinstance(g,FrankaHand):
                if a.width_mm is None:raise ValueError('Native Hand requires --width-mm')
                result=site.guarded(lambda cancel:g.move(a.width_mm/1000,a.speed_mm_s/1000,cancelled=cancel))
            else:result=site.guarded(lambda cancel:g.move(a.position,a.speed,a.force,cancelled=cancel))
        elif a.command=='plc-test':
            if not a.output_signal or not a.feedback_signal:raise ValueError('Specify output and feedback signal names')
            def test(cancel):
                site.plc.write(a.output_signal,True)
                site.plc.wait(a.feedback_signal,cancelled=cancel)
                return dict(output_asserted=a.output_signal,feedback_confirmed=a.feedback_signal)
            result=site.guarded(test)
        else:
            if site.config.get('assembly_commissioned') is not True:
                raise RuntimeError('Complete per-stage field commissioning before assembly')
            cell=ConfiguredCell(site);cell.validate();assemble(cell);result=dict(sequential_assembly_completed=True,hardware_ready=False)
    except BaseException:
        site.stop();raise
    print(json.dumps(result,indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
