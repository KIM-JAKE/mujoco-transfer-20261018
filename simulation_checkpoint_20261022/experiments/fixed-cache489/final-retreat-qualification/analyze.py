exec(open('/tmp/pin-exact-field-handoff/profile_numeric.py').read().split("z=np.load('/tmp/pin481-terminal-public/")[0])
from isaac_kinematics import frame
run=next((R/'isaac_port/runs').glob('*v488-*'));report=json.loads((run/'pipeline-results.json').read_text());model=json.loads((run/'robot-kinematics.json').read_text());arm=Arm.__new__(Arm)
for k in ['base','tip','axes','frames','bounds']:setattr(arm,k,np.asarray(model[k],float))
arm.link_paths=['/link'+str(i)for i in range(7)];arm.hand_path='/right_NativeHand';names=report['dof_names'];ids=[names.index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)];goals=json.loads((R/'isaac_port/runs/native-roas-table-bin-retimed14/arm-plan.json').read_text())['goals'];P=np.asarray([goals[p]['P']for p in ['ring1','ring2','pin']]);z=np.load(run/'executed-actuator-commands.npz');rows=np.flatnonzero(z['pin_phase']=='final_empty_RIGHT_clearance_retreat');records=[]
def root(q):return(arm.fk(q)@frame([0,0,-.1034],np.eye(3)))[:3,3]
for label,i in [('before',rows[0]-1),('last',rows[-1])]:
 c=z['command'][i,ids];a=z['actual_dof_position'][i,ids];cr,ar=root(c),root(a);records.append(dict(label=label,index=int(i),command_q=c.tolist(),actual_q=a.tolist(),command_root_m=cr.tolist(),actual_root_m=ar.tolist(),command_goal_distances_m=np.linalg.norm(P-cr,axis=1).tolist(),actual_goal_distances_m=np.linalg.norm(P-ar,axis=1).tolist(),tracking_q_rad=(a-c).tolist(),tracking_root_error_m=float(np.linalg.norm(ar-cr)),actual_velocity_rad_s=z['actual_dof_velocity'][i,ids].tolist()))
result=dict(run=run.name,scope='PUBLIC SDKactual/command encoders+original calibratedCADgoal; noactortruth',retreat_rows=len(rows),retreat_s=len(rows)/480,goals={p:goals[p]['P']for p in ['ring1','ring2','pin']},records=records)
Path('/tmp/fixed489-final-retreat/evidence488.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
