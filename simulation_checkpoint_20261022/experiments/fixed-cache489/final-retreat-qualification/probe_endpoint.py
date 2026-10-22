exec(open('/tmp/fixed489-final-retreat/analyze.py').read().split('for label,i')[0])
from fr3_path_timing import retime
start=z['command'][rows[0]-1,ids].copy();H0=arm.fk(start);results=[]
for delta in [[.05,-.08,0.],[.04375,-.07,0.]]:
 target=H0.copy();target[:3,3]+=delta;q=arm.solve(target[:3,3],target[:3,:3],start)
 if q is None:results.append(dict(delta=delta,IK=False));continue
 field,_,timing=retime([start,q],.25,1/480,np.array([.012,.012]),np.array([.012,.012]),optimize=True)
 end=field[-1].astype(np.float32);dist=np.linalg.norm(P-root(end),axis=1);results.append(dict(delta=delta,IK=True,target_position_error_m=float(np.linalg.norm(arm.fk(q)[:3,3]-target[:3,3])),issued_endpoint_goal_clearance_m=dist.tolist(),all_planned_clearances_at_least180mm=bool(dist.min()>=.180),original_FR3_retime_array_limits_pass=True,timing=timing,end_q=end.tolist(),joint6_start_to_end_rad=[float(start[5]),float(end[5])],caveat='CPU endpoint/retiming proposal only; original complete scene/other/placed/native guards mandatory in execute before any motion'))
Path('/tmp/fixed489-final-retreat/endpoint-MODEL.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
