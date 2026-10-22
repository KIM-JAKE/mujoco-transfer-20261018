exec(open('/tmp/fixed510-pin-canonical-branch/probe_public_ik.py').read().split('rows=[]')[0])
rows=[]
for n in (507,508,509):
 p=next((R/'isaac_port/runs').glob(f'*v{n}-*'));j=json.loads((p/'pipeline-results.json').read_text());m=json.loads((p/'robot-kinematics.json').read_text());arm=SimpleNamespace(**{k:np.array(m[k],float)for k in ('base','tip','frames','axes','bounds')});ik=mod.NumericIK(arm)
 T0=np.array(cand['pickup_H']);T0[2,3]+=.07;preq=ik.solve(T0[:3,3],T0[:3,:3],cand['pickup_q'])
 P=np.array(j['pin_skill_progress']['events'][0]['result']['pose']['center_m']);T=T0.copy();T[:3,3]+=P-np.array(catalog['pickup_center_from_RGBD'])
 for name,qseed in [('robot_seeds.pin_tail_approach',seed),('catalogue.pickup_q',cand['pickup_q']),('catalogue.pre_q',preq)]:
  q=ik.solve(T[:3,3],T[:3,:3],qseed)
  row=dict(run=n,source=name,seed=np.asarray(qseed).tolist(),solved_q=None if q is None else q.tolist(),whole_scene_clearance_qualified=False)
  if q is not None:
   F=arm.base.copy()
   for x,a,(pre,post)in zip(q,arm.axes,arm.frames):
    X=np.eye(4);X[:3,:3]=Rotation.from_rotvec(a*x).as_matrix();F=F@pre@X@post
   F=F@arm.tip;row.update(position_residual_m=float(np.linalg.norm(F[:3,3]-T[:3,3])),rotation_residual_rad=float(Rotation.from_matrix(F[:3,:3]@T[:3,:3].T).magnitude()),all_original_bounds=bool(np.all(q>=arm.bounds[:,0])and np.all(q<=arm.bounds[:,1])))
  rows.append(row)
(OUT/'public-all-canonical-ik.json').write_text(json.dumps(dict(scope='Public initialRGBD/currentcalibratedkinematics/existingliteralrobotjointseeds only. Original IK; no scene/physical admission.',rows=rows),indent=2)+'\n')
for row in rows:print(row['run'],row['source'],None if row['solved_q']is None else np.round(row['solved_q'],5),row.get('position_residual_m'))
