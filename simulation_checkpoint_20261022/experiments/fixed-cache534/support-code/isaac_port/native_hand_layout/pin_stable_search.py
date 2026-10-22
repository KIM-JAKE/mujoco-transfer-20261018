from layout_search import *
p=prepare_part(parts[2]);side='left';Rp=p['R']@p['localR'].T;P=p['tcp']-Rp@p['localP'];res=[]
for band,gap in [(-.006,.0125),(0,.0125),(.012,.0075)]:
 for yaw in [0,45,-45,90,-90,135,180]:
  for pitch in [0,10,-10,20,-20,35,-35,60,-60,90,120,150]:
   R=Rotation.from_rotvec(Rp[:,2]*np.deg2rad(yaw)).as_matrix()@p['R']@Rotation.from_euler('y',pitch,degrees=True).as_matrix();tcp=P+Rp@np.array([0,-.00025,band]);dist,hits=g.clearance(R,tcp,gap)
   if hits or dist<.0003:continue
   q=ik(side,tcp,R,p['q_insert'])
   if q is None:continue
   ok,why=clear(side,q,gap)
   if not ok:continue
   pre=ik(side,tcp+[0,0,-.055],R,q)
   if pre is None:continue
   ok,why=path_clear(side,[pre,q],gap,15)
   if not ok:continue
   row=dict(part='pin',yaw_deg=yaw,pitch_delta_deg=pitch,local_band_z_m=band,tcp=tcp.tolist(),R=R.tolist(),q_insert=q.tolist(),q_preinsert=pre.tolist(),localP=(Rp.T@(tcp-P)).tolist(),localR=(Rp.T@R).tolist(),gap=gap,hand_machine_clearance_m=float(dist));res.append(row);print('PIN_STABLE_BAND',band,yaw,pitch,dist,flush=True);(HERE/'pin-stable-candidates.json').write_text(json.dumps(res,indent=2))
   if len(res)>=10:break
  if len(res)>=10:break
 if len(res)>=10:break
(HERE/'pin-stable-candidates.json').write_text(json.dumps(res,indent=2))
