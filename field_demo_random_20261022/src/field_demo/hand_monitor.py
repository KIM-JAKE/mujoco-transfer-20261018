"""Actual SDK polling outside the native arm-heartbeat thread.

A retained sample is usable for at most200ms; failed/stalled reads revoke it.
No interpolation or commanded-grasp substitution is permitted.
"""
import copy,threading,time,math
from datetime import datetime,timezone

class HandMonitor:
    def __init__(self,read,*,max_age_s=.2,clock=time.monotonic):
        if not 0<max_age_s<=.2:raise ValueError('Hand monitoring must expire within200ms')
        self.read,self.max_age,self.clock=read,max_age_s,clock
        self.lock=threading.Lock();self.done=threading.Event();self.value=None;self.stamp=None;self.error=None
        self.thread=threading.Thread(target=self._poll,daemon=True);self.thread.start()
    def publish(self,value):
        if value.get('completed')is not True or type(value.get('robot_time_s'))not in (int,float)or not math.isfinite(value['robot_time_s'])or value['robot_time_s']<0:raise RuntimeError('Actual completed timestamped SDK Hand sample required')
        with self.lock:
            if self.value is not None and value['robot_time_s']<=self.value['robot_time_s']:return
            self.value=dict(value,captured_at_utc=datetime.now(timezone.utc).isoformat());self.stamp=self.clock();self.error=None
    def _poll(self):
        while not self.done.is_set():
            try:self.publish(self.read())
            except BaseException as e:
                with self.lock:self.error=str(e)
            self.done.wait(.03)
    def status(self):
        with self.lock:
            if self.error or self.stamp is None or self.clock()-self.stamp>self.max_age:raise RuntimeError('Actual Hand SDK sample unavailable/stale: '+str(self.error))
            return copy.deepcopy(self.value)
    def close(self):self.done.set();self.thread.join(timeout=.3)
