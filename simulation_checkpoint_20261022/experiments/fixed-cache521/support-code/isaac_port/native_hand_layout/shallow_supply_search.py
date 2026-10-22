"""Small table-top ring rests: approach from above instead of lifting a fork.
Uses original CAD, native Hand contact bands, and explicit holder geometry.
"""
from compact_layout_plan import *

# Six-DOF fallback avoids fixing a redundant first joint at an unreachable seed.
def freeik(side,tcp,R,seed):
 goal=tcp-R@np.array([0,0,.1034]);rgoal=Rotation.from_matrix(R@Rotation.from_euler('z',45,degrees=True).as_matrix())
 def fun(q):
  data.qpos[AD[side]]=q;mujoco.mj_kinematics(model,data);return np.r_[10*(data.xpos[FL[side]]-goal),(Rotation.from_matrix(data.xmat[FL[side]].reshape(3,3))*rgoal.inv()).as_rotvec()]
 for bias in [0,.5,-.5]:
  start=np.array(seed);start[0]+=bias;fit=least_squares(fun,np.clip(start,LOWER+.005,UPPER-.005),bounds=(LOWER+.005,UPPER-.005),max_nfev=70)
  if np.linalg.norm(fun(fit.x)[:3])<1e-4 and np.linalg.norm(fun(fit.x)[3:])<1e-4:return fit.x
 return None

def robust(side,tcp,R,seed):
 q=ik(side,tcp,R,seed)
 return q if q is not None and np.min(np.r_[q-LOWER,UPPER-q])>.012 else freeik(side,tcp,R,seed)

def shallow_head(part,r,top=1.15,theta=0):
 R=np.array(r['R']);Rp=R@np.array(r['localR']).T;P=np.array(r['tcp'])-Rp@np.array(r['localP']);radius=.0375 if part=='ring1'else .031;half=.00775 if part=='ring1'else .007;rows=[];spec=[]
 # Three points surround the center of mass; no long tines enter the palm path.
 for k in range(3):
  a=np.deg2rad(theta+k*120);pos=P+[np.cos(a)*(radius-.0025),np.sin(a)*(radius-.0025),-half-.008]
  obj=fcl.CollisionObject(fcl.Cylinder(.002,.016),fcl.Transform(np.eye(3),pos));name=part+'_rest_'+str(k);rows.append((name,obj));spec.append(dict(name=name,type='cylinder',pos=pos.tolist(),R=np.eye(3).tolist(),radius=.002,length=.016))
 plate=P+[0,0,-half-.019];rows.append(shape_box(part+'_rest_plate',plate,[radius*2+.012,radius*2+.012,.006]));spec.append(dict(name=part+'_rest_plate',type='box',pos=plate.tolist(),size=[radius*2+.012,radius*2+.012,.006],R=np.eye(3).tolist()))
 bottom=np.array([P[0],P[1],top+.012]);post=plate+[0,0,-.003];rows.append(tube(part+'_post',post,bottom,.008));spec.append(dict(name=part+'_post',type='capsule',a=post.tolist(),b=bottom.tolist(),radius=.008))
 return rows,spec

def main():
 results=[];out=[];desk_info=[]
 for cx,cy,z in itertools.product([-.25,-.35],[-.30,-.32],[1.1,1.15,1.20]):
  desk=table(cx,cy,z);why=machine_hits(desk);good=not why and endpoint_clear(desk);desk_info.append(dict(center=[cx,cy,z],valid=good,hits=why))
 print('SHALLOW_TABLES',[x for x in desk_info if x['valid']],flush=True);(HERE/'shallow-tables.json').write_text(json.dumps(desk_info,indent=2))
 for part in ['ring1','ring2']:
  old=next(x for x in parts if x['part']==part);b=BANDS[part];s='right'if part=='ring1'else'left';source=old['endpoints'][0];Rp0=np.array(source['hand_rotation'])@np.array(old['constant_hand_to_part_rotation']).T;R0=Rp0@np.array(b['localR']);seed=np.array(source['arm_clear_ik'][0]['qpos']);wins=[]
  for yaw,x,y,z in itertools.product([0,45,-45,90],[-.05,-.15]if part=='ring1'else[-.30,-.40],[-.16,-.24,-.32],[1.28,1.38]):
   set_env([]);R=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@R0;tcp=np.array([x,y,z]);q=robust(s,tcp,R,seed)
   if q is None or not clear(s,q,.08)[0]:continue
   r=dict(part=part,side=s,R=R.tolist(),tcp=tcp.tolist(),localP=b['localP'],localR=b['localR'],gap=b['gap'],q_pick=q.tolist(),holder='three_point_rest',stock_yaw_deg=yaw)
   head=None
   for angle in [0,15,30,45,60,75,90,105]:
    rows,spec=shallow_head(part,r,1.15,angle)
    if machine_hits(rows)or not endpoint_clear(rows):continue
    if not clear(s,q,.08)[0]:continue
    qpre=robust(s,tcp+[0,0,.04],R,q)
    if qpre is None or not path_clear(s,[qpre,q],.08,21)[0]:continue
    if not path_clear(s,[q,qpre],r['gap'],21)[0]:continue
    head=(rows,spec,angle,qpre);break
   if head is None:continue
   score=segment_time(READY[s],qpre)+segment_time(qpre,np.array(b['q_preinsert']))
   if score>20:continue
   rows,spec,angle,qpre=head;r['q_pregrasp']=qpre.tolist();c=dict(part=part,recipe=r,geometry=spec,rest_angle_deg=angle,score_s=score,back_m=0);wins.append(c);print('SHALLOW_ENDPOINT',part,yaw,tcp,round(score,3),flush=True)
  wins.sort(key=lambda c:c['score_s']);out.extend(wins);(HERE/'shallow-head-candidates.json').write_text(json.dumps(out,indent=2));print('SHALLOW_COUNT',part,len(wins),flush=True)
if __name__=='__main__':main()
