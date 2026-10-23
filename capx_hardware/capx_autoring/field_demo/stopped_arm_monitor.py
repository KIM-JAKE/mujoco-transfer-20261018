"""Fresh actual SDK aggregate wrench/pose guard during stopped Hand commands.

No inferred pad forces, sensor booleans or synthetic motion. This is an
application monitoring condition, not a certified stopping-distance bound.
"""
import copy,math,threading,time
import numpy as np


def validate(value,reference_q_d,tracking_error_rad):
    if value.get('read_only')is not True or value.get('server_protocol_version')!=10 or value.get('robot_mode_name')!='idle':raise RuntimeError('Actual idle SDK arm state required during stopped Hand sweep')
    t=value.get('robot_time_s')
    if type(t)not in(int,float)or not math.isfinite(t)or t<0:raise RuntimeError('Actual SDK arm clock required')
    def vector(key,size):
        row=np.asarray(value[key],float)
        if row.shape!=(size,)or not np.isfinite(row).all():raise RuntimeError('Invalid actual SDK '+key)
        return row
    q,desired=vector('q',7),vector('q_d',7)
    if np.max(np.abs(desired-np.asarray(reference_q_d)))>1e-6:raise RuntimeError('SDK desired joint pose changed during stopped Hand sweep')
    if np.max(np.abs(q-desired))>min(.005,tracking_error_rad)or max(np.max(np.abs(vector('dq',7))),np.max(np.abs(vector('dq_d',7))))>.01:raise RuntimeError('Actual arm left stopped SDK pose envelope')
    wrench=vector('O_F_ext_hat_K',6)
    if np.linalg.norm(wrench[:3])>5 or np.linalg.norm(wrench[3:])>1:raise RuntimeError('Actual SDK aggregate wrench exceeds unchanged5N/1Nm Hand-sweep stop threshold')
    return value

class StoppedArmMonitor:
    def __init__(self,read,reference_q_d,tracking_error_rad,*,clock=time.monotonic,max_age_s=.2):
        if not 0<max_age_s<=.2:raise ValueError('Actual stopped arm SDK sample expires within200ms')
        self.read,self.reference,self.tracking,self.clock,self.max_age=read,reference_q_d,tracking_error_rad,clock,max_age_s
        self.lock=threading.Lock();self.done=threading.Event();self.value=None;self.stamp=None;self.error=None
        self.publish(read())  # actual synchronous valid state before any Hand command
        self.thread=threading.Thread(target=self._poll,daemon=True);self.thread.start()
    def publish(self,value):
        try:validate(value,self.reference,self.tracking)
        except BaseException as error:
            with self.lock:self.error=str(error)
            raise
        with self.lock:
            if self.value is not None and value['robot_time_s']<=self.value['robot_time_s']:return
            self.value=copy.deepcopy(value);self.stamp=self.clock();self.error=None
    def _poll(self):
        while not self.done.is_set():
            try:self.publish(self.read())
            except BaseException as error:
                with self.lock:self.error=str(error)
            self.done.wait(.03)
    def status(self):
        with self.lock:
            if self.error or self.stamp is None or self.clock()-self.stamp>self.max_age:raise RuntimeError('Actual stopped arm wrench/pose SDK sample unavailable/stale: '+str(self.error))
            return copy.deepcopy(self.value)
    def close(self):self.done.set();self.thread.join(timeout=.3)
