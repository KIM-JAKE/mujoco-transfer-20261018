"""Dense audit of the selected kinematic timeline, including carried CAD parts.
Assembly contact is reported separately from free-space collision failures.
"""
from compact_compare import *
from fr3_motion_control import VMAX

def timeline(row,t):
 lanes=row['lanes'];steps=row['coordination']['steps'];left=np.array(READY['left']);right=np.array(READY['right']);state={s:(np.array(READY[s]),None)for s in AD}
 for step in steps:
  elapsed=t-step['start_s'];T=step['duration_s']
  for side in AD:
   k=step[side+'_index']
   if k is None:continue
   segment=lanes[side][k]
   if elapsed>=T:state[side]=(np.array(segment['b']),segment)
   elif elapsed>=0:state[side]=(state_at(segment,elapsed)[0],segment)
  if elapsed<T:return state
 elapsed=t-row['coordination']['duration_s']
 for segment in row['button_finish']:
  if elapsed<segment['duration_s']:
   state['right']=(state_at(segment,max(0,elapsed))[0],segment);return state
  state['right']=(np.array(segment['b']),segment);elapsed-=segment['duration_s']
 return state

def shape_for(part):
 if part.startswith('ring'):
  archive=np.load(PORT/'generated/contact_task/precision-cad.npz');key='instance_0542'if part=='ring1'else'instance_0549';return g.mesh_shape(archive[key+'_vertices'],archive[key+'_faces'])
 source=json.load(open(PORT/'generated/contact_task/contact-source.json'));archive=np.load(PORT/'generated/contact_task/collision.npz');precise=np.load(PORT/'generated/contact_task/precision-pin-main.npz');vertices=[];faces=[];offset=0
 for r in source['parts']['pin']['colliders']:
  key=r['mesh']
  if key=='pin_mesh_206':v=precise['instance_0557_vertices'];f=precise['instance_0557_faces']
  else:v=archive[key+'_vertices']@Rotation.from_quat(np.array(r['quaternion'])[[1,2,3,0]]).as_matrix().T+r['position'];f=archive[key+'_faces']
  vertices.append(v);faces.append(f+offset);offset+=len(v)
 return g.mesh_shape(np.concatenate(vertices),np.concatenate(faces))

def main():
 row=json.load(open(HERE/'compact-selected.json'));pieces=table(*row['table_center_m']);geometry=row['geometry']
 for c in json.load(open(HERE/'compact-path-candidates.json')).values():
  for candidate in c:
   part=candidate['recipe']['part']
   if np.linalg.norm(np.array(candidate['recipe']['q_pick'])-row['recipes'][part]['q_pick'])<1e-6:
    pieces+=candidate_pieces(candidate['candidate'],row['table_center_m'][2])[0];break
 set_env(pieces);matrix=[];jointreport=[]
 for side,lane in row['lanes'].items():
  for segment in lane:
   T=segment['duration_s'];a=np.array(segment['a']);b=np.array(segment['b']);delta=b-a;u=np.linspace(0,1,1001);h=u**3*(10-15*u+6*u*u);q=a+h[:,None]*delta;v=30*u[:,None]**2*(1-u[:,None])**2*delta/T;acc=60*u[:,None]*(1-u[:,None])*(1-2*u[:,None])*delta/T**2;jerk=(60-360*u[:,None]+360*u[:,None]**2)*delta/T**3;lo,hi=position_velocity_bounds(q)
   jointreport.append(dict(side=side,label=segment['label'],part=segment['part'],position_pass=bool(np.all(q>=LOWER)and np.all(q<=UPPER)),velocity_pass=bool(np.all(v>=.95*lo-1e-7)and np.all(v<=.95*hi+1e-7)),max_velocity_fraction=float(np.max(abs(v)/VMAX)),max_abs_acceleration_rad_s2=float(np.max(abs(acc))),max_abs_jerk_rad_s3=float(np.max(abs(jerk)))))
 collisions=[];last=None;samples=0
 # Source mesh queries are expensive; recheck the final combined scene at a
 # fixed 10ms time interval, independent of the planning samples.
 for t in np.arange(0,row['cycle_time_s']+.005,.01):
  poses=timeline(row,float(t));states={}
  for side,(q,s)in poses.items():
   gap=s['gap_m']if s else .08;states[side]=(q,gap)
   okay,why=ls_clear(side,q,gap)
   if not okay and len(collisions)<30:collisions.append(dict(time_s=float(t),side=side,label=s['label']if s else 'ready',why=why))
  if not arm_pair_clear(states['left'],states['right'])and len(collisions)<30:collisions.append(dict(time_s=float(t),why='cross_arm'))
  samples+=1
  if samples%300==0:print('DENSE_AUDIT',samples,len(collisions),flush=True)
 # Report carried-part/M80 intersections on each individual carrying segment;
 # mating contacts during insertion require a separate contact/force simulation.
 original_tree=fcl.DynamicAABBTreeCollisionManager();original_tree.registerObjects(ORIGINAL);original_tree.setup();names={**g.checker.static_names,**g.names};hits=[]
 for part,r in row['recipes'].items():
  shape=shape_for(part);L=np.array(r['localR']);lp=np.array(r['localP'])
  for si,s in enumerate(r['segments']):
   if s['label']not in('extract','lift','transfer','insert'):continue
   T=s['duration_s'];events=[]
   for t in np.linspace(0,T,max(9,int(T/.025)+2)):
    q=state_at(s,t)[0];tcp,R=fk(r['side'],q);Rp=R@L.T;P=tcp-Rp@lp;moving=fcl.CollisionObject(shape,fcl.Transform(Rp,P));found=[]
    def callback(a,b,_):
     if pairhit(a,b):
      pa=tuple(np.round(a.getTranslation(),7));pb=tuple(np.round(b.getTranslation(),7));found.append(names.get(pa,names.get(pb,'unidentified_machine_geom')))
     return False
    original_tree.collide(moving,None,callback)
    if found:events.append(dict(local_time_s=float(t),machine_parts=sorted(set(found))))
   if events:hits.append(dict(part=part,label=s['label'],segment=si,sample_hits=len(events),first=events[0],last=events[-1]))
 audit=dict(label=row['label'],task_time_s=row['cycle_time_s'],kinematic_dense_collision_sample_s=.01,samples=samples,robot_collisions=collisions,joint_limits=jointreport,all_joint_limits_pass=all(x['position_pass']and x['velocity_pass']and x['max_abs_acceleration_rad_s2']<=9.5+1e-6 and x['max_abs_jerk_rad_s3']<=4750+1e-5 for x in jointreport),carried_part_machine_intersections=hits,full_physics_assembly_tested=False,real_robot_tested=False)
 (HERE/'compact-dense-audit.json').write_text(json.dumps(audit,indent=2));print('AUDIT_DONE',len(collisions),audit['all_joint_limits_pass'],len(hits),flush=True)

# Use the accelerated static environment while retaining its full source cache.
from fast_environment import clear as ls_clear
if __name__=='__main__':main()
