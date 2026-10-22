"""Check native finger coupling with one motor and a passive follower.

Only robot encoder/actuator APIs are used. No workpiece pose is read and no
physical task success can be inferred from this isolated motor diagnostic.
"""
import numpy as np

def execute(c):
    art=c['art'];fi=c['fi'];cmd=c['cmd'];idx=c['idx'];dt=c['dt'];stiff=c['stiffness'];damp=c['damping'];forces=art.get_dof_max_forces().copy();forces[0,fi]=[30.,.0001];stiff[0,fi]=0.;damp[0,fi]=[4000.,.001];art.set_dof_max_forces(forces,idx);art.set_dof_stiffnesses(stiff,idx);art.set_dof_dampings(damp,idx);start=cmd[0,fi].copy();before=art.get_dof_positions()[0,fi].copy();peak=0.
    N=int(.8/dt)
    for k in range(1,N+1):
        u=k/N;h=u**3*(10-15*u+6*u*u);cmd[0,fi]=start+(.03-start)*h;c['step']('single_motor_coupling_diagnostic');q=art.get_dof_positions()[0,fi];peak=max(peak,float(abs(q[0]-q[1])))
    for _ in range(round(.1/dt)):c['step']('single_motor_coupling_hold')
    after=art.get_dof_positions()[0,fi].copy();return dict(state='native_single_motor_coupling_probe',encoder_before_m=before.tolist(),encoder_after_m=after.tolist(),peak_finger_difference_m=peak,follower_pass=bool(abs(after[0]-after[1])<.0001 and before[1]-after[1]>.008),scope='Robot motor coupling only; no pickup or task completion',full_task_pass=False)
