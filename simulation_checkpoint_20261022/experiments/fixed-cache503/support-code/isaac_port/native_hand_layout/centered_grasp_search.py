from layout_search import *

def main():
 results=[]
 for base in parts:
  p=prepare_part(base);name=p['name'];side=p['side'];ins=base['endpoints'][1];Ri=np.array(ins['hand_rotation']);tcp=np.array(ins['tcp_world_m']);q0=np.array(ins['arm_clear_ik'][0]['qpos']);Rp=Ri@p['localR'].T;P=tcp-Rp@p['localP'];found=[];trials=[]
  for yaw in [0,30,-30,60,-60,90,-90,120,-120,180]:
   for pitch in [0,10,-10,20,-20,35,-35]:
    R=Rotation.from_rotvec(Rp[:,2]*np.deg2rad(yaw)).as_matrix()@Ri@Rotation.from_euler('y',pitch,degrees=True).as_matrix()
    for along in [0,-.003,-.006,.003]:
     # Center the ring along fingertip width; keep adjustable pad axial band.
     oldoffset=tcp-P;point=P+R[:,2]*along
     if name=='pin':point=P+Rp@p['localP']  # retain actual pin grasp band
     dist,hits=g.clearance(R,point,p['gap'])
     if hits:continue
     q=ik(side,point,R,q0)
     if q is None:continue
     ok,why=clear(side,q,p['gap'])
     if not ok:continue
     pre=ik(side,point+[0,0,-.055],R,q)
     if pre is None:continue
     ok,why=path_clear(side,[pre,q],p['gap'],13)
     if not ok:continue
     out=dict(part=name,yaw_deg=yaw,pitch_delta_deg=pitch,axial_offset_m=along,tcp=point.tolist(),R=R.tolist(),q_insert=q.tolist(),q_preinsert=pre.tolist(),localP=(Rp.T@(point-P)).tolist(),localR=(Rp.T@R).tolist(),gap=p['gap'],hand_machine_clearance_m=float(dist))
     found.append(out);print('CENTERED',name,yaw,pitch,along,dist,flush=True)
     if len(found)>=8:break
    if len(found)>=8:break
   if len(found)>=8:break
  results.append(dict(part=name,candidates=found));(HERE/'centered-grasps.json').write_text(json.dumps(results,indent=2))
if __name__=='__main__':main()
