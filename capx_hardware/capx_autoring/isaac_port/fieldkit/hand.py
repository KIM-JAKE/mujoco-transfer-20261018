"""Native Franka Hand adapter and standalone first connection CLI (no arm motion)."""
import argparse
import fcntl
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import select
import subprocess
import time


def bounded(value, low, high, name, *, positive=False):
    if type(value) not in (int,float) or not math.isfinite(value) or not low<=value<=high or (positive and value==0):
        raise ValueError('Invalid '+name)
    return float(value)


class FrankaHand:
    def __init__(self, config):
        self.ip=str(ipaddress.ip_address(config['ip']))
        self.binary=str(Path(config['binary']).resolve())

    @staticmethod
    def validate_action(config, *, grasp=False):
        required={'width_m','speed_m_s','force_n','epsilon_inner_m','epsilon_outer_m'} if grasp else {'width_m','speed_m_s'}
        if set(config)!=required:raise ValueError('Native Hand action fields must be '+str(sorted(required)))
        bounded(config['width_m'],0,.08,'width_m')
        bounded(config['speed_m_s'],0,.02,'speed_m_s',positive=True)
        if grasp:
            bounded(config['force_n'],5,20,'force_n')
            for k in ('epsilon_inner_m','epsilon_outer_m'):bounded(config[k],0,.002,k,positive=True)

    def status(self):
        p=subprocess.run([self.binary,'status',self.ip],capture_output=True,text=True,timeout=10,check=True)
        return self._result(p.stdout.splitlines())

    @staticmethod
    def _result(lines):
        rows=[json.loads(line) for line in lines if line.startswith('{')]
        if not rows or rows[-1].get('completed') is not True:raise RuntimeError('Missing successful native Hand acknowledgement')
        s=rows[-1]
        for k in ('width_m','max_width_m'):bounded(s[k],0,.1,k)
        if type(s.get('is_grasped')) is not bool:raise RuntimeError('Invalid Hand object feedback')
        return s

    def _run(self, mode, values, *, timeout_s=20, cancelled=lambda:False):
        key=hashlib.sha256(self.ip.encode()).hexdigest()[:24]
        with (Path('/tmp')/('m80-hand-'+key+'.lock')).open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            return self._run_locked(mode,values,timeout_s=timeout_s,cancelled=cancelled)

    def _run_locked(self, mode, values, *, timeout_s=20, cancelled=lambda:False):
        bounded(timeout_s,.1,60,'timeout_s')
        if cancelled():raise RuntimeError('Hand action cancelled before start')  # Site callback can also raise.
        p=subprocess.Popen([self.binary,'--execute-'+mode,self.ip,*map(str,values)],
                           stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
        lines=[];armed=False;began=time.monotonic()
        try:
            while p.poll() is None:
                if cancelled():raise RuntimeError('Hand action cancelled')
                if time.monotonic()-began>timeout_s:raise TimeoutError('Hand action timeout')
                if select.select([p.stdout],[],[],.01)[0]:
                    line=p.stdout.readline();lines.append(line)
                    if line.strip()=='HEARTBEAT_REQUIRED':armed=True
                    elif not line or line.startswith('{'):armed=False
                if armed:p.stdin.write('READY\n');p.stdin.flush()
                time.sleep(.02)
            lines.extend(p.stdout.readlines());error=p.stderr.read()
            if p.returncode:raise RuntimeError('Native Hand failed: '+error.strip())
            if cancelled():raise RuntimeError('Hand cancelled before acknowledgement')
            return self._result(lines)
        except BaseException:
            if p.poll() is None:
                try:p.stdin.write('STOP\n');p.stdin.flush()
                except (OSError,ValueError):pass
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.terminate()
                    try:p.wait(timeout=3)
                    except subprocess.TimeoutExpired:p.kill();p.wait()
            raise
        finally:
            for stream in (p.stdin,p.stdout,p.stderr):stream.close()

    def activate(self, *, timeout_s=30, cancelled=lambda:False):
        # Explicit homing only; never auto-home during a task.
        return self._run('homing',[],timeout_s=timeout_s,cancelled=cancelled)

    def move(self, width_m, speed_m_s, *, grasp=False, force_n=None, epsilon_inner_m=None,
             epsilon_outer_m=None, timeout_s=20, cancelled=lambda:False):
        config=dict(width_m=width_m,speed_m_s=speed_m_s)
        if grasp:config.update(force_n=force_n,epsilon_inner_m=epsilon_inner_m,epsilon_outer_m=epsilon_outer_m)
        elif any(v is not None for v in (force_n,epsilon_inner_m,epsilon_outer_m)):
            raise ValueError('Move uses width and speed only')
        self.validate_action(config,grasp=grasp)
        values=[width_m,speed_m_s]+([force_n,epsilon_inner_m,epsilon_outer_m] if grasp else [])
        result=self._run('grasp' if grasp else 'move',values,timeout_s=timeout_s,cancelled=cancelled)
        if grasp and not result['is_grasped']:raise RuntimeError('Native Hand did not confirm object')
        if not grasp and (result['is_grasped'] or abs(result['width_m']-width_m)>.001):
            raise RuntimeError('Native Hand opening not confirmed')
        return result

    def stop(self):
        subprocess.run([self.binary,'stop',self.ip],capture_output=True,text=True,timeout=5,check=True)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['status','homing','move','grasp','stop'])
    p.add_argument('--ip',required=True);p.add_argument('--binary',default='/tmp/m80-field-build/m80_franka_hand')
    p.add_argument('--execute',action='store_true');p.add_argument('--clear-hand-area',action='store_true')
    p.add_argument('--width-mm',type=float);p.add_argument('--speed-mm-s',type=float,default=5)
    p.add_argument('--force-n',type=float,default=5);p.add_argument('--epsilon-mm',type=float,default=.5)
    a=p.parse_args(argv);g=FrankaHand(dict(ip=a.ip,binary=a.binary))
    if a.command=='status':result=g.status()
    elif a.command=='stop':g.stop();result=dict(stop_requested=True)
    else:
        if not a.execute or not a.clear_hand_area:p.error('Finger motion requires --execute --clear-hand-area; homing opens and closes the fingers')
        if a.command=='homing':result=g.activate()
        else:
            if a.width_mm is None:p.error('--width-mm is required')
            kwargs=dict(width_m=a.width_mm/1000,speed_m_s=a.speed_mm_s/1000)
            if a.command=='grasp':kwargs.update(grasp=True,force_n=a.force_n,epsilon_inner_m=a.epsilon_mm/1000,epsilon_outer_m=a.epsilon_mm/1000)
            result=g.move(**kwargs)
    print(json.dumps(result,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
