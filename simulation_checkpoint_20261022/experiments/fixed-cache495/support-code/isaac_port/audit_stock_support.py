"""Check the added hollow supply guide against every planned robot pose."""
from pathlib import Path
import sys,json,hashlib
root=Path(__file__).resolve().parent
import numpy as np
import mujoco,fcl
from plan_right_posture import Checker
from prepare import SOURCES
from task_timing_fast import source_time
model=mujoco.MjModel.from_xml_path(str(SOURCES['cap_button']));data=mujoco.MjData(model);checker=Checker(root/'generated/contact_task/fast',source_clock=source_time);plan=np.load(root/'generated/contact_task/arm-posture.npz');record=np.load(root/'generated/contact_task/fast/trajectory.npz');assert np.allclose(plan['time_s'],record['time_s']);closest=1e6;pair=None
for f,t in enumerate(plan['time_s']):
 data.qpos[:]=record['qpos'][f]
 for side in ['left','right']:
  for j in range(7):data.qpos[int(model.joint(side+'_arm_fr3v2_joint'+str(j+1)).qposadr[0])]=plan[side+'_qpos'][f,j]
 mujoco.mj_kinematics(model,data)
 for row,obj in checker.robot:
  b=model.body(row['body']).id;r=data.xmat[b].reshape(3,3);p=data.xpos[b]+r@row['position'];obj.setTransform(fcl.Transform(r@checker.rot(row['quaternion']),p));distance=fcl.distance(obj,checker.stock_sleeve,fcl.DistanceRequest(),fcl.DistanceResult());limit=.003 if '_arm_'in row['body']else .0001
  if distance<closest:closest=distance;pair=dict(time_s=float(t),body=row['body'])
  assert distance>=limit,(float(t),row['body'],distance,limit)
report=dict(state='passed',samples=len(plan['time_s']),robot_hulls=len(checker.robot),minimum_clearance_m=float(closest),closest=pair,stock_geometry_sha256=hashlib.sha256((root/'stock_fixture_geometry.py').read_bytes()).hexdigest(),arm_plan_sha256=hashlib.sha256((root/'generated/contact_task/arm-posture.npz').read_bytes()).hexdigest())
(root/'generated/contact_task/stock-fixture-audit.json').write_text(json.dumps(report,indent=2)+'\n');p=root/'generated/contact_task/arm-posture.json';d=json.loads(p.read_text());d['stock_fixture_audit']=report;p.write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(report,indent=2))
