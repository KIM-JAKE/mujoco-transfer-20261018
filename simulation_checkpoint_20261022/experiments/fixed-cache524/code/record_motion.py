"""Record actuator targets and finite clamp commands for path retiming."""
import numpy as np
from isaac_motion import Audit


class RecordingAudit(Audit):
    def __init__(self,cmd,indices,dt):
        super().__init__(cmd,indices,dt)
        self.commands=[cmd[0].copy()];self.phases=['initial'];self.phase='initial'
        self.control_changes=[]

    def update(self,cmd):
        super().update(cmd)
        self.commands.append(cmd[0].copy());self.phases.append(self.phase)

    def control(self,part,value):
        self.control_changes.append((self.samples,part,float(value)))

    def save(self,path):
        control=np.zeros((len(self.commands),3));state=np.zeros(3);j=0
        for i in range(len(control)):
            while j<len(self.control_changes)and self.control_changes[j][0]<=i:
                _,part,value=self.control_changes[j];state[['ring1','ring2','pin'].index(part)]=value;j+=1
            control[i]=state
        np.savez_compressed(path,time_s=np.arange(len(self.commands))*self.dt,
                            command=self.commands,clamp=control,phase=self.phases)
