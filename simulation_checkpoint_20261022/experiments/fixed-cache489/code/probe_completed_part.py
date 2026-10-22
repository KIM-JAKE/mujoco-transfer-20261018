"""Static source280 regression using archived RGB-D and robot encoders only."""
from pathlib import Path
import json,time,numpy as np
from placed_part_clearance import PlacedPartClearance
from isaac_kinematics import frame
from rgbd_geometry import axis_frame

def execute(c):
    here=Path(__file__).resolve().parent;r=here.parents[1]/'runs/native-roas-table-scatter-stable-v280-park-single-initial-gpu-all3-446';d=json.loads((r/'pipeline-results.json').read_text());x=np.load(r/'executed-actuator-commands.npz');names=d['dof_names'];ids=[names.index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)];fi=[names.index('right_native_'+n)for n in ['left','right']];t=x['time_s'];goal=np.asarray(c['goals']['ring1']['P']);obs=json.loads((r/'rgb-observations.json').read_text());available=[v for v in obs if v.get('part')=='ring1' and v['result'].get('pose')and np.linalg.norm(np.asarray(v['result']['pose']['center_m'])-goal)<.0015 and float(v['captured_wall_s'])<230];ob=max(available,key=lambda v:float(v['captured_wall_s']));pose=ob['result']['pose'];registry=PlacedPartClearance();checker=c['checkers']['right']
    for item in c['checkers'].values():item.placed_part_clearance=registry
    registry.register(checker,'ring1',c['part_collision_templates']['ring1'],frame(pose['center_m'],axis_frame(pose['axis'])),uncertainty_m=.0035)
    def query(k):
        q=x['actual_dof_position'][k,ids].astype(float);width=float(x['actual_dof_position'][k,fi].sum());ok,why=checker.same_arm_clearance.clear(q,width);return dict(source_row=int(k),source_wall_s=float(t[k]),clear=bool(ok),reason=why)
    good=query(int(np.argmin(abs(t-float(ob['captured_wall_s'])))))
    candidates=np.flatnonzero((t>=236.0)&(t<=237.4));bad=None
    for k in candidates:
        row=query(int(k))
        if not row['clear']and row['reason'].get('kind')=='robot_or_payload_vs_completed_part':bad=row;break
    result=dict(state='static_completed_part_RGBD_encoder_regression',approved=bool(good['clear']and bad is not None),physical_robot_motion=False,uses_free_body_poses=False,source_RGBD_capture_wall_s=ob['captured_wall_s'],registered_pose=pose,good_after_release=good,rejected_before_kick=bad,registry_statistics=registry.stats,full_task_pass=False)
    (c['vision'].out/'completed-part280-static.json').write_text(json.dumps(result,indent=2));return result
