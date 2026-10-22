from evaluate_layouts import *


def fixtures(frame,recipes):
 rows=[]
 def add(shape,R,P,label):rows.append((label,fcl.CollisionObject(shape,fcl.Transform(R,P))))
 for entry in frame['frames']:
  r=recipes[entry['part']];best=entry['best'];nodes=np.array(best['nodes']);part=entry['part'];R=np.array(r['R']);Rp=R@np.array(r['localR']).T;P=np.array(r['tcp'])-Rp@np.array(r['localP'])
  for i,(a,b)in enumerate(zip(nodes[:-1],nodes[1:])):
   v=b-a;length=np.linalg.norm(v)
   if length<1e-7:continue
   z=v/length;x=np.cross(z,[0,0,1]);x=np.array([1.,0,0])if np.linalg.norm(x)<1e-6 else x/np.linalg.norm(x);y=np.cross(z,x);rad=.02 if i==len(nodes)-2 else .004 if np.linalg.norm(a-nodes[0])<.25 else .0125;add(fcl.Capsule(rad,length),np.column_stack([x,y,z]),(a+b)/2,part+'_frame')
  if part.startswith('ring'):
   axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis);axis=Rotation.from_euler('z',best.get('fork_angle_deg')or 0,degrees=True).apply(axis);cross=np.cross([0,0,1],axis);B=np.column_stack([axis,cross,[0,0,1]]);rad=.0375 if part=='ring1'else .031;half=.00775 if part=='ring1'else .007
   for sign in [-1,1]:add(fcl.Box(2*(rad+.025),.006,.002),B,P-axis*.02+cross*(sign*rad*.7)+[0,0,-half-.001],part+'_fork')
   backbone=P-axis*(rad+.045)+[0,0,-half-.001];add(fcl.Box(.008,2*(rad*.7+.004),.004),B,backbone,part+'_spine')
  else:
   for sign in [-1,1]:add(fcl.Box(.010,.008,.008),Rp,P+Rp@np.array([sign*.0088,-.00025,.004]),'pin_head_clip_sweep')
   add(fcl.Cylinder(.0024,.002),Rp,P+Rp@np.array([0,-.00025,-.01875]),'pin_axial_datum')
 return rows

def main():
 record=json.load(open(HERE/'comparison-physical-band.json'));frames=json.load(open(HERE/'routed-independent-frames.json'));results=[];original_statics=list(statics)
 for layout in record['layouts']:
  if len(layout.get('recipes',{}))!=3:continue
  g.manager=fcl.DynamicAABBTreeCollisionManager();g.manager.registerObjects(original_statics);g.manager.setup()
  frame=next(x for x in frames if x['layout']==layout['label'])
  if not frame['all_found']:continue
  pieces=fixtures(frame,layout['recipes']);environment_hits=[]
  for name,obj in pieces:
   d=fcl.CollisionData();g.manager.collide(obj,d,fcl.defaultCollisionCallback)
   if d.result.is_collision:environment_hits.append(name)
  print('FIXTURE_ENVIRONMENT',layout['label'],environment_hits,flush=True)
  g.manager=fcl.DynamicAABBTreeCollisionManager();g.manager.registerObjects(original_statics+[o for name,o in pieces]);g.manager.setup()
  row=dict(label=layout['label'],recipes=layout['recipes'],support_frames=frame['frames'],fixture_environment_hits=environment_hits,valid=False,failures=[])
  if environment_hits:
   results.append(row);continue
  for name,r in row['recipes'].items():
   side=r['side'];segs=r['segments'];oldentry=[s for s in segs if s['label']=='empty_approach'];start=np.array(oldentry[0]['a']);qpick=np.array(r['q_pick']);R=np.array(r['R']);tcp=np.array(r['tcp']);gap=.08 if name.startswith('ring')else .02
   pre=ik(side,tcp+[0,0,-.025],R,qpick)
   if pre is None:row['failures'].append([name,'pregrasp_ik']);continue
   okay,why=path_clear(side,[pre,qpick],gap,21)
   if not okay:row['failures'].append([name,'pregrasp_collision',why]);continue
   entry=rrt(side,start,pre,gap,max_nodes=500)
   if entry is None:row['failures'].append([name,'empty_route']);continue
   entry.append(qpick);new=[]
   for a,b in zip(entry[:-1],entry[1:]):new.append(dict(a=a.tolist(),b=b.tolist(),duration_s=segment_time(a,b),label='empty_approach',gap_m=gap,part=name))
   r['segments']=new+[s for s in segs if s['label']!='empty_approach'];r['duration_s']=sum(s['duration_s']for s in r['segments'])
   repaired=[]
   for seg in r['segments']:
    okay,why=path_clear(side,[np.array(seg['a']),np.array(seg['b'])],gap if seg['label']in('empty_approach','release','withdraw')else r['gap'],25)
    if not okay and seg['label']=='transfer':
     reroute=rrt(side,np.array(seg['a']),np.array(seg['b']),r['gap'],max_nodes=350)
     if reroute:
      for a,b in zip(reroute[:-1],reroute[1:]):repaired.append(dict(a=a.tolist(),b=b.tolist(),duration_s=segment_time(a,b),label='transfer',part=name,gap_m=r['gap']))
      continue
    if not okay:row['failures'].append([name,seg['label'],why])
    repaired.append(seg)
   r['segments']=repaired;r['duration_s']=sum(s['duration_s']for s in repaired)
   print('FRAME_PATH',layout['label'],name,r['duration_s'],row['failures'],flush=True)
  row['valid']=not row['failures'];results.append(row);(HERE/'fixture-checked-layouts.json').write_text(json.dumps(results,indent=2))
  g.manager=fcl.DynamicAABBTreeCollisionManager();g.manager.registerObjects(original_statics);g.manager.setup()
 (HERE/'fixture-checked-layouts.json').write_text(json.dumps(results,indent=2))

if __name__=='__main__':main()
