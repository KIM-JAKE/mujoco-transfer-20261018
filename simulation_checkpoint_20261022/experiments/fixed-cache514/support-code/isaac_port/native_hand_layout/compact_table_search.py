"""Compact external-table layout, sampled FCL paths and honest timing estimates.
All supports are separate from M80. No full dynamic insertion certification.
"""
from evaluate_layouts import *
import copy,itertools
BASE=json.load(open(HERE/'comparison-physical-band.json'));BANDS=BASE['grasp_bands'];ORIGINAL=list(statics)
# The pin's original lower grasp band passed the contact bench; a pitch-10
# candidate in the earlier search metadata did not. Use the tested recipe.
_pin=next(x for x in BASE['layouts']if x['label']=='near_front')['recipes']['pin']
BANDS['pin']=dict(part='pin',R=_pin['R'],tcp=fk('left',np.array(_pin['q_path'][-1]))[0].tolist(),q_insert=_pin['q_path'][-1],q_preinsert=_pin['q_exit'],localP=_pin['localP'],localR=_pin['localR'],gap=.0097,hand_machine_clearance_m=.000772)
if(HERE/'compact-pin-band.json').exists():BANDS['pin']=json.load(open(HERE/'compact-pin-band.json'))

def set_env(pieces):
 g.manager=fcl.DynamicAABBTreeCollisionManager();g.manager.registerObjects(ORIGINAL+[o for _,o in pieces]);g.manager.setup()

def shape_box(name,P,size,R=np.eye(3)):
 return name,fcl.CollisionObject(fcl.Box(*size),fcl.Transform(R,np.array(P)))

def tube(name,a,b,rad=.006):
 a=np.array(a);b=np.array(b);v=b-a;length=np.linalg.norm(v);z=v/max(length,1e-9);x=np.cross(z,[0,0,1]);x=np.array([1.,0,0])if np.linalg.norm(x)<1e-6 else x/np.linalg.norm(x);y=np.cross(z,x)
 return name,fcl.CollisionObject(fcl.Capsule(rad,length),fcl.Transform(np.column_stack([x,y,z]),(a+b)/2))

def table(cx,cy,z,w=.5,d=.35):
 rows=[shape_box('independent_table_top',[cx,cy,z-.0125],[w,d,.025])]
 for sx,sy in itertools.product([-1,1],repeat=2):
  x=cx+sx*(w/2-.025);y=cy+sy*(d/2-.025);rows.append(tube('independent_table_leg',[x,y,.015],[x,y,z-.025],.015))
 return rows

def machine_hits(rows):
 set_env([]);hits=[]
 for name,obj in rows:
  cd=fcl.CollisionData();g.manager.collide(obj,cd,fcl.defaultCollisionCallback)
  if cd.result.is_collision:hits.append([name,'M80'])
  for row,other in g.checker.robot:
   if not row['body'].startswith('official_FR3_Duo'):continue
   b=model.body(row['body']).id;R=data.xmat[b].reshape(3,3);fixed=fcl.CollisionObject(g.checker.shape(row,False),fcl.Transform(R@g.checker.rot(row['quaternion']),data.xpos[b]+R@row['position']))
   if pairhit(obj,fixed):hits.append([name,row['body']]);break
 return hits

def heads(part,r,top_z,back=.12):
 R=np.array(r['R']);Rp=R@np.array(r['localR']).T;P=np.array(r['tcp'])-Rp@np.array(r['localP']);rows=[];spec=[]
 def box(name,pos,size,B=np.eye(3)):
  rows.append(shape_box(part+'_'+name,pos,size,B));spec.append(dict(name=part+'_'+name,type='box',pos=np.array(pos).tolist(),size=size,R=B.tolist()))
 def rod(name,a,b,rad):
  rows.append(tube(part+'_'+name,a,b,rad));spec.append(dict(name=part+'_'+name,type='capsule',a=np.array(a).tolist(),b=np.array(b).tolist(),radius=rad))
 if part.startswith('ring'):
  axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis);cross=np.cross([0,0,1],axis);B=np.column_stack([axis,cross,[0,0,1]]);rad=.0375 if part=='ring1'else .031;half=.00775 if part=='ring1'else .007
  for sign in [-1,1]:box('fork',P-axis*.02+cross*(sign*rad*.7)+[0,0,-half-.001],[2*(rad+.025),.006,.002],B)
  backbone=P-axis*(rad+.045)+[0,0,-half-.001];box('spine',backbone,[.008,2*(rad*.7+.004),.004],B)
  root=P-axis*back+[0,0,-half-.007];rod('head_link',backbone,root,.003);rod('post',root,[root[0],root[1],top_z+.012],.006)
 else:
  for sign in [-1,1]:box('clip_sweep',P+Rp@np.array([sign*.0088,-.00025,.004]),[.010,.008,.008],Rp)
  rows.append((part+'_datum',fcl.CollisionObject(fcl.Cylinder(.0024,.002),fcl.Transform(Rp,P+Rp@np.array([0,-.00025,-.01875])))))
  spec.append(dict(name=part+'_datum',type='cylinder',pos=(P+Rp@np.array([0,-.00025,-.01875])).tolist(),R=Rp.tolist(),radius=.0024,length=.002))
  root=P+[0,.065,.025];rod('clip_bridge',P+[0,0,.025],root,.003);rod('post',root,[root[0],root[1],top_z+.012],.006);rod('datum_bridge',P+Rp@np.array([0,-.00025,-.01975]),[root[0],root[1],P[2]-.01975],.002)
 return rows,spec

def endpoint_clear(rows):
 set_env(rows)
 for s in AD:
  if not clear(s,READY[s],.08)[0]:return False
 for p,b in BANDS.items():
  s='right'if p=='ring1'else'left'
  for q in [b['q_insert'],b['q_preinsert']]:
   if not clear(s,np.array(q),.08 if p!='pin'else .02)[0]:return False
 return True

def main():
 tables=[]
 for cx,cy,z in itertools.product([-.25,-.35,-.45],[-.25,-.30,-.40],[.85,1.0,1.1]):
  rows=table(cx,cy,z);hits=machine_hits(rows);okay=not hits and endpoint_clear(rows);tables.append(dict(center=[cx,cy,z],valid=okay,hits=hits))
 (HERE/'compact-tables.json').write_text(json.dumps(tables,indent=2));print('TABLES',[(t['center'])for t in tables if t['valid']],flush=True)
 endpoints=json.load(open(HERE/'compact-endpoints.json'));out=[]
 for p in ['ring1','ring2','pin']:
  b=BANDS[p];R=np.array(b['R']);s='right'if p=='ring1'else'left';rows=next(x['candidates']for x in endpoints if x['part']==p)
  if p=='pin':
   rows=[]
   for x,y,z in itertools.product([-.30,-.40,-.50],[-.40,-.30,-.22,-.16],[1.32,1.42,1.52]):
    tcp=np.array([x,y,z]);q=ik(s,tcp,R,np.array(b['q_preinsert']))
    if q is not None and clear(s,q,.02)[0]:rows.append(dict(q=q.tolist(),tcp=tcp.tolist()))
  for row in rows:
   q=np.array(row['q']);tcp=np.array(row['tcp']);r=dict(part=p,side=s,R=R.tolist(),tcp=tcp.tolist(),localP=b['localP'],localR=b['localR'],gap=b['gap'],q_pick=q.tolist())
   for back in [.10,.13,.16]:
    h,spec=heads(p,r,1.0,back)
    if machine_hits(h) or not endpoint_clear(h):continue
    if not clear(s,q,.08 if p!='pin'else .02)[0]:continue
    # Absolute endpoint timing lower bound is only for candidate ranking.
    score=segment_time(READY[s],q)+segment_time(q,np.array(b['q_preinsert']))
    if score>20:continue
    out.append(dict(part=p,back_m=back,score_s=score,recipe=r,geometry=spec));break
  print('COMPACT_HEADS',p,len([x for x in out if x['part']==p]),flush=True)
 (HERE/'compact-head-candidates.json').write_text(json.dumps(sorted(out,key=lambda x:x['score_s']),indent=2))
if __name__=='__main__':main()
