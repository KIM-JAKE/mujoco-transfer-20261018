from layout_search import *
res=[]
for base in parts[:2]:
 p=prepare_part(base);side=p['side'];Ri=p['R'];Rp=Ri@p['localR'].T;P=p['tcp']-Rp@p['localP'];found=[]
 for yaw in [0,-30,30,-60,60]:
  for pitch in [0,10,-10,20,-20,35]:
   R=Rotation.from_rotvec(Rp[:,2]*np.deg2rad(yaw)).as_matrix()@Ri@Rotation.from_euler('y',pitch,degrees=True).as_matrix()
   for xoffset in [-.003,-.0045,-.006]:
    for zoffset in [-.006,-.003,0,.003]:
     tcp=P+R[:,0]*xoffset+R[:,2]*zoffset;dist,hits=g.clearance(R,tcp,p['gap'])
     if hits or dist<.0007:continue
     q=ik(side,tcp,R,p['q_insert'])
     if q is None:continue
     ok,why=clear(side,q,p['gap'])
     if not ok:continue
     pre=ik(side,tcp+[0,0,-.055],R,q)
     if pre is None:continue
     ok,why=path_clear(side,[pre,q],p['gap'],11)
     if not ok:continue
     row=dict(part=p['name'],yaw_deg=yaw,pitch_delta_deg=pitch,x_offset_m=xoffset,axial_offset_m=zoffset,tcp=tcp.tolist(),R=R.tolist(),q_insert=q.tolist(),q_preinsert=pre.tolist(),localP=(Rp.T@(tcp-P)).tolist(),localR=(Rp.T@R).tolist(),gap=p['gap'],hand_machine_clearance_m=float(dist));found.append(row);print('REFINED',p['name'],yaw,pitch,xoffset,zoffset,round(dist*1000,3),flush=True)
     if len(found)>=6:break
    if len(found)>=6:break
   if len(found)>=6:break
  if len(found)>=6:break
 res.append(dict(part=p['name'],candidates=found));(HERE/'refined-grasp-bands.json').write_text(json.dumps(res,indent=2))
