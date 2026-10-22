"""Bounded POST-RUN telemetry from the existing PhysX callback only.

NEVER a controller sensor: no target, force, friction, collider, command or
physics configuration is read or changed here. Raw points/impulses can later be
matched to the separately saved evaluator trace. Unknown data only loses logs.
"""
import numpy as np


class PinContactEvaluator:
    def __init__(self,pin_path,resolve_path,maximum_records=20000):
        self.pin_path=str(pin_path);self.resolve_path=resolve_path;self.paths={};self.maximum_records=int(maximum_records)
        if not 1<=self.maximum_records<=20000:raise ValueError('Invalid evaluator-only contact log bound')
        self.report=dict(EVALUATION_ONLY=True,used_for_control=False,geometry_or_physics_changes=False,force_control_input=False,sampling_control_step_stride=4,maximum_records=self.maximum_records,records=[],dropped_records=0,decode_errors=0,physical_commands_issued=0,physics_steps=0)
    def _path(self,value):
        if value not in self.paths:self.paths[value]=str(self.resolve_path(value))
        return self.paths[value]
    def capture(self,headers,data,physics_s,control_dt_s,physics_substeps):
        # Use the ORIGINAL callback's current timing only for reporting units.
        try:
            if int(round(float(physics_s)/float(control_dt_s)))%4:return
            substep_dt=float(control_dt_s)/int(physics_substeps)
            if not np.isfinite(substep_dt)or substep_dt<=0:raise ValueError('Invalid evaluator timestep')
            for header in headers:
                a,b=self._path(header.actor0),self._path(header.actor1)
                if self.pin_path not in (a,b):continue
                other=b if a==self.pin_path else a
                if not('right_arm_'in other or'right_NativeHand'in other):continue
                if len(self.report['records'])>=self.maximum_records:self.report['dropped_records']+=1;continue
                points=[];normal_impulse=0.;total_impulse=0.
                for i in range(header.contact_data_offset,header.contact_data_offset+header.num_contact_data):
                    row=data[i];item={}
                    for name in ['position','normal','impulse']:
                        value=getattr(row,name,None)
                        if value is not None:
                            value=np.asarray(value,float)
                            if value.shape!=(3,)or not np.isfinite(value).all():raise ValueError('Malformed contact vector')
                            item[name]=value.tolist()
                    if'normal'in item and'impulse'in item:
                        n=np.asarray(item['normal']);imp=np.asarray(item['impulse']);normal_impulse+=abs(float(n@imp));total_impulse+=float(np.linalg.norm(imp))
                    separation=getattr(row,'separation',None)
                    if separation is not None and np.isfinite(float(separation)):item['separation_m']=float(separation)
                    points.append(item)
                self.report['records'].append(dict(physics_s=float(physics_s),substep_dt_s=substep_dt,actor0=a,actor1=b,normal_force_magnitude_n=normal_impulse/substep_dt,total_impulse_magnitude_div_substep_n=total_impulse/substep_dt,contacts=points))
        except Exception:
            # A diagnostic failure MUST NOT alter the original safety callback.
            self.report['decode_errors']+=1
