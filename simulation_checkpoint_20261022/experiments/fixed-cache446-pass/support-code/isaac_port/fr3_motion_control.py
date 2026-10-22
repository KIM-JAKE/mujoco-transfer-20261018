"""Jerk-limited arm motor commands, including all live pose corrections.

Official FR3 FCI limits: https://frankarobotics.github.io/docs/robot_specifications.html
No robot or workpiece state is overwritten by this controller.
"""
import numpy as np

LOWER = np.array([-2.9007,-1.8361,-2.9007,-3.0770,-2.8763,.4398,-3.0508])
UPPER = np.array([2.9007,1.8361,2.9007,-.1169,2.8763,4.6216,3.0508])
VMAX = np.array([2.62,2.62,2.62,2.62,5.26,4.18,5.26])
OFFSET = np.array([.6599,.2517,.2000,.3533,.5757,.4878,.4628])
DECEL = np.array([6.,2.585,3.5,4.,17.,5.5,17.])

def position_velocity_bounds(q):
    return (np.maximum(-VMAX,np.minimum(0.,OFFSET-np.sqrt(np.maximum(0.,2*DECEL*(q-LOWER))))),
            np.minimum(VMAX,np.maximum(0.,-OFFSET+np.sqrt(np.maximum(0.,2*DECEL*(UPPER-q))))))

class MotionLimiter:
    def __init__(self,names,initial,dt=1/480):
        from ruckig import Ruckig, InputParameter, OutputParameter, ControlInterface, Synchronization
        self.indices=np.array([names.index(f'{s}_arm_fr3v2_joint{j}') for s in ['left','right'] for j in range(1,8)])
        self.dt=dt; self.otg=Ruckig(14,dt); self.input=InputParameter(14); self.output=OutputParameter(14)
        self.input.control_interface=ControlInterface.Velocity
        self.input.synchronization=Synchronization.No
        self.p=np.asarray(initial,dtype=float)[self.indices].copy(); self.v=np.zeros(14); self.a=np.zeros(14)
        self.input.current_position=self.p.tolist(); self.input.current_velocity=self.v.tolist(); self.input.current_acceleration=self.a.tolist()
        # Deliberate reserve below official 10 rad/s² / 5000 rad/s³ limits.
        self.input.max_acceleration=np.tile([3.5,2.,3.5,3.5,3.5,3.5,3.5],2).tolist()
        self.input.max_jerk=[500.]*14
        self.target=self.p.copy(); self.feedforward=np.zeros(14)
    def set_target(self,q,v):
        self.target=np.asarray(q,dtype=float)[self.indices].copy()
        self.feedforward=np.asarray(v,dtype=float)[self.indices].copy()
        if np.any(self.target<np.tile(LOWER,2)+.005) or np.any(self.target>np.tile(UPPER,2)-.005):
            raise RuntimeError('FR3 target outside reserved joint range')
    def following_error(self,actual):
        return float(np.max(abs(np.asarray(actual)[self.indices]-self.target)))
    def progress_scale(self,actual):
        error=self.following_error(actual)
        return float(np.clip((.030-error)/.026,0.,1.))
    def step(self):
        lo,hi=position_velocity_bounds(self.p.reshape(2,7)); lo=lo.reshape(-1); hi=hi.reshape(-1)
        # Reserve for actuator tracking and changes in the position-dependent envelope.
        self.input.min_velocity=(.75*lo).tolist(); self.input.max_velocity=(.75*hi).tolist()
        desired=self.feedforward+12*(self.target-self.p)
        self.input.target_velocity=np.clip(desired,.90*.75*lo,.90*.75*hi).tolist()
        self.input.target_acceleration=[0.]*14
        result=self.otg.update(self.input,self.output)
        if int(result)<0: raise RuntimeError('FR3 motion generation failed: '+str(result))
        before=self.a.copy(); self.p=np.array(self.output.new_position); self.v=np.array(self.output.new_velocity); self.a=np.array(self.output.new_acceleration)
        lower,upper=position_velocity_bounds(self.p.reshape(2,7))
        if np.any(self.p<np.tile(LOWER,2)) or np.any(self.p>np.tile(UPPER,2)) or np.any(self.v<lower.reshape(-1)-1e-7) or np.any(self.v>upper.reshape(-1)+1e-7):
            raise RuntimeError('FR3 position-dependent command velocity envelope exceeded')
        if np.max(abs(self.a))>10+1e-7 or np.max(abs(self.a-before))/self.dt>5000+1e-5:
            raise RuntimeError('FR3 command acceleration or jerk exceeded')
        self.output.pass_to_input(self.input)
        return self.p,self.v,self.a

class IsolatedMotionLimiter:
    """Keep Ruckig's C++ runtime out of Isaac Kit's incompatible symbol scope."""
    def __init__(self,names,initial,worker_path,dt=1/480,worker_mode='absolute'):
        import subprocess,sys,json
        self.indices=np.array([names.index(f'{s}_arm_fr3v2_joint{j}')for s in ['left','right']for j in range(1,8)])
        self.p=np.asarray(initial)[self.indices].astype(float).copy();self.v=np.zeros(14);self.a=np.zeros(14)
        self.target=self.p.copy();self.feedforward=np.zeros(14)
        self.log=open(worker_path.parent.parent/'motion-worker.log','wb')
        self.process=subprocess.Popen([sys.executable,str(worker_path)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log)
        header=dict(names=[names[i]for i in self.indices],initial=self.p.tolist(),dt=dt,mode=worker_mode)
        self.process.stdin.write((json.dumps(header)+'\n').encode());self.process.stdin.flush()
        assert self._read(1)==b'R','Motion worker failed to initialize'
    def _read(self,n):
        import os,select,time
        data=b'';deadline=time.monotonic()+15
        while len(data)<n:
            remaining=deadline-time.monotonic()
            if remaining<=0 or not select.select([self.process.stdout],[],[],remaining)[0]:raise RuntimeError('Motion worker response timed out')
            chunk=os.read(self.process.stdout.fileno(),n-len(data))
            if not chunk:raise RuntimeError('Motion worker exited; inspect motion-worker.log')
            data+=chunk
        return data
    def set_target(self,q,v):
        self.target=np.asarray(q)[self.indices].copy();self.feedforward=np.asarray(v)[self.indices].copy()
    def following_error(self,actual):
        return float(np.max(abs(np.asarray(actual)[self.indices]-self.target)))
    def progress_scale(self,actual):
        return float(np.clip((.030-self.following_error(actual))/.026,0.,1.))
    def step(self):
        self.p,self.v,self.a=self.exchange(self.target,self.feedforward)
        return self.p,self.v,self.a
    def exchange(self,target,feedforward):
        self.process.stdin.write(np.r_[target,feedforward].astype('<f8').tobytes());self.process.stdin.flush()
        result=np.frombuffer(self._read(42*8),dtype='<f8').copy().reshape(3,14)
        assert np.isfinite(result).all()
        return result
    def close(self):
        self.process.terminate();self.process.wait(timeout=5);self.log.close()

class OffsetLimiter:
    def __init__(self,names,initial,dt):
        from ruckig import Ruckig,InputParameter,OutputParameter,Synchronization
        self.otg=Ruckig(14,dt);self.input=InputParameter(14);self.output=OutputParameter(14)
        self.input.current_position=[0.]*14;self.input.current_velocity=[0.]*14;self.input.current_acceleration=[0.]*14
        self.input.target_velocity=[0.]*14;self.input.target_acceleration=[0.]*14
        self.input.max_velocity=[.025]*14;self.input.max_acceleration=[.4]*14;self.input.max_jerk=[50.]*14;self.input.synchronization=Synchronization.No
    def set_target(self,q,v):
        if np.max(abs(q))>.25:raise RuntimeError('Pose correction exceeds 0.25 rad')
        self.input.target_position=np.asarray(q).tolist()
    def step(self):
        result=self.otg.update(self.input,self.output)
        if int(result)<0:raise RuntimeError('Offset motion generation failed')
        self.output.pass_to_input(self.input)
        return np.array(self.output.new_position),np.array(self.output.new_velocity),np.array(self.output.new_acceleration)

class PathMotionLimiter(IsolatedMotionLimiter):
    def __init__(self,names,initial,worker_path,profile,dt=1/480):
        super().__init__(names,initial,worker_path,dt,worker_mode='offset');self.profile=profile;self.dt=dt;self.reference=0.;self.offset_target=np.zeros(14);self.previous_a=np.zeros(14);self.previous_v=np.zeros(14)
        self.command_fraction=float(profile.report().get('planning_fraction',1.))
    def progress_scale(self,actual):return 1.
    def peek_reference(self,dt):return self.profile.peek(dt)
    def set_reference(self,reference,hold,started):
        self.reference=reference;self.profile.gate(reference,hold,started)
    def set_target(self,q,v):
        self.target=np.asarray(q)[self.indices].copy();self.offset_target=self.target-self.profile.path(self.reference)
    def step(self):
        base_p,base_v,base_a=self.profile.advance(self.dt);op,ov,oa=self.exchange(self.offset_target,np.zeros(14))
        self.p=base_p+op;self.v=base_v+ov;self.a=base_a+oa
        lo,hi=position_velocity_bounds(self.p.reshape(2,7));lo*=self.command_fraction;hi*=self.command_fraction
        if np.any(self.p<np.tile(LOWER,2))or np.any(self.p>np.tile(UPPER,2))or np.any(self.v<lo.reshape(-1)-1e-6)or np.any(self.v>hi.reshape(-1)+1e-6):raise RuntimeError('FR3 command envelope exceeded')
        if np.max(abs(self.a))>10.*self.command_fraction or np.max(abs(self.a-self.previous_a))/self.dt>5000.*self.command_fraction:raise RuntimeError('FR3 command acceleration/jerk budget exceeded')
        if np.max(abs(self.v-self.previous_v))/self.dt>10.*self.command_fraction:raise RuntimeError('FR3 command velocity discontinuity')
        self.previous_v=self.v.copy();self.previous_a=self.a.copy()
        return self.p,self.v,self.a
