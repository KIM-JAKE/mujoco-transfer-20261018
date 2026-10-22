"""Check whether a successful lift retained the planned source grasp frame."""
from pathlib import Path
import json,sys,math
import numpy as np
HERE=Path(__file__).resolve().parent;RUN=HERE.parents[1]/'runs'/sys.argv[1]
report=json.load(open(RUN/'physx-pickup-results.json'));events={e['event']:e for e in report['events']};rows={}
def normal(tr):
    x,y,z,w=tr[3:];return np.array([2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)])
for part in ['ring1','ring2']:
    initial=np.array(events['settle']['parts'][part]);grasp=np.array(events['grasp_hold']['parts'][part]);translation=float(np.linalg.norm(initial[:3]-grasp[:3]));tilt=math.degrees(math.acos(np.clip(np.dot(normal(initial),normal(grasp)),-1,1)))
    rows[part]=dict(source_part_displacement_before_lift_m=translation,source_ring_axis_tilt_before_lift_deg=tilt,pass_planned_grasp_alignment=translation<.0005 and tilt<1.4)
result=dict(scope='Independent physical source-pose alignment check; ring axial yaw is unconstrained',source_report='physx-pickup-results.json',engine=report['engine'],full_task_pass=False,cycle_time_s=None,parts=rows,planned_grasp_alignment_pass=all(r['pass_planned_grasp_alignment']for r in rows.values()),physical_pickup_pass=report['pickup_pass'])
(RUN/'physx-pickup-alignment-audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
