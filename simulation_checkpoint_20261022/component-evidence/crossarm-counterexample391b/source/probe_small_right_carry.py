"""Read-only original pair checker vs actual failure recording; no actor writes."""
from pathlib import Path
import os
import json,time
import numpy as np
from scipy.spatial.transform import Rotation
from predictive_pair_clearance import pair_clear

def execute(c):
    source=Path('/workspace/project/mujoco/isaac_port/runs/native-roas-table-scatter-stable-v387-final-direct-priority-GPU3-all3-450')
    report=json.loads((source/'pipeline-results.json').read_text())
    trace=np.load(source/'physx-sequence-trace.npz'); commands=np.load(source/'executed-actuator-commands.npz')
    rc=c['checkers']['right'];lc=c['checkers']['left'];ri=c['ids'];li=c['left_ids'];rf=c['fi'];lf=c['left_fi']
    def evaluate(label,row):
        row=np.asarray(row,float);r=row[ri];l=row[li];rw=float(row[rf].sum());lw=float(row[lf].sum())
        with rc.other_arm_at(lc,l,lw):a,wa=pair_clear(rc,r,rw)
        with lc.other_arm_at(rc,r,rw):b,wb=pair_clear(lc,l,lw)
        return dict(label=label,right_clear=bool(a),right_why=wa,left_clear=bool(b),left_why=wb,right_width_m=rw,left_width_m=lw,right_q=r.tolist(),left_q=l.tolist())
    results=[evaluate('abort_'+key,report['cross_arm_abort'][key])for key in ['command','encoder']]
    results.append(evaluate('last_trace',trace['q'][-1]));results.append(evaluate('last_saved_encoder',commands['actual_dof_position'][-1]));results.append(evaluate('last_saved_command',commands['command'][-1]))
    history=[]
    indices=np.where(commands['pin_phase']=='ring1_carry_to_handover')[0]
    for i in indices[::max(1,len(indices)//50)]:
        a=evaluate('issued',commands['command'][i]);a.update(index=int(i),wall_s=float(commands['time_s'][i]),kind='command');history.append(a)
        a=evaluate('actual',commands['actual_dof_position'][i]);a.update(index=int(i),wall_s=float(commands['time_s'][i]),kind='encoder');history.append(a)
    comparisons=[];paths=[str(x)for x in trace['camera_body_paths']]
    for ti in [0,max(0,len(trace['q'])-10),len(trace['q'])-1]:
        row=trace['q'][ti]
        for side,checker,ids,fi in [('right',rc,ri,rf),('left',lc,li,lf)]:
            model=checker.arm.bodies(row[ids],float(row[fi].sum()))
            for path,T in model.items():
                if path not in paths:continue
                actor=trace['camera_bodies'][ti,paths.index(path)]
                AR=Rotation.from_quat(actor[3:7]).as_matrix()
                # Polar projection removes only USD float matrix nonorthogonality.
                U,_,V=np.linalg.svd(T[:3,:3]);MR=U@V
                comparisons.append(dict(side=side,path=path,trace_index=ti,wall_s=float(trace['time_s'][ti]),position_error_m=float(np.linalg.norm(T[:3,3]-actor[:3])),rotation_error_rad=float(Rotation.from_matrix(MR@AR.T).magnitude())))
    result=dict(state='read_only_crossarm_counterexample',robot_motion_executed=False,uses_free_body_poses_for_control=False,source_run=source.name,approach_order=[x for x in report['planning_diagnostics']if x.get('event')=='approved_handover'][0]['approach_order'],indices=dict(right=list(map(int,ri)),left=list(map(int,li)),right_fingers=list(map(int,rf)),left_fingers=list(map(int,lf))),pair_results=results,carry_history=history,FK_vs_recorded_actor=comparisons,pair_box_counts=dict(right=len(rc.pair_boxes),left=len(lc.pair_boxes)))
    out=source.parent/os.environ['M80_ROAS_RUN']/'crossarm-counterexample.json';out.write_text(json.dumps(result,indent=2));print('CROSSARM_COUNTEREXAMPLE',json.dumps(result),flush=True)
    return result
