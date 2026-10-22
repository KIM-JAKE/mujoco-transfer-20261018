"""Verify that actuator lag cancels while real grasp slip remains observable."""
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from pose_feedback import PoseFeedback

p=object.__new__(PoseFeedback);p.parts=['ring1','ring2','pin']
p.model=SimpleNamespace(body=lambda name:SimpleNamespace(id=0))
planned_R=Rotation.from_euler('xyz',[.1,-.2,.3]);actual_R=Rotation.from_euler('xyz',[-.15,.22,.1])
planned_p=np.array([.4,.2,1.5]);actual_p=planned_p+np.array([.01,-.02,.005])
relative_p=np.array([.02,-.01,.1]);relative_R=Rotation.from_euler('x',.2)
p.data=SimpleNamespace(xpos=np.array([actual_p]),xmat=np.array([actual_R.as_matrix().reshape(-1)]))
p.tracking_data=SimpleNamespace(xpos=np.array([planned_p]),xmat=np.array([planned_R.as_matrix().reshape(-1)]))
goal=planned_p+planned_R.apply(relative_p);goal_R=planned_R*relative_R
p.target=lambda name,t:(goal,goal_R)
positions=np.tile(actual_p+actual_R.apply(relative_p),(3,1));quats=np.tile((actual_R*relative_R).as_quat()[[3,0,1,2]],(3,1))
e,r=p.compensated_errors('pin',0,positions,quats,'left')
assert np.linalg.norm(e)<1e-12 and np.linalg.norm(r)<1e-12,(e,r)
positions[2]+=actual_R.apply([.001,0,0])
e,r=p.compensated_errors('pin',0,positions,quats,'left')
assert abs(np.linalg.norm(e)-.001)<1e-12,e
print('PASS: rigid grasp with actuator lag gives zero correction; 1 mm grasp slip remains detected')
