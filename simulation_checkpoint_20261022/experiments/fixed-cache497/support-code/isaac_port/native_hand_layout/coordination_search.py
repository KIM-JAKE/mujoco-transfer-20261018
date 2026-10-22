"""Schedule whole C2 stopped segments using a collision-tested coordination DAG."""
from evaluate_layouts import *
import heapq
record=json.load(open(HERE/'comparison-physical-band.json'));results=[]

def edge_clear(a,b,na,nb):
 T=max(a['duration_s']if a else 0,b['duration_s']if b else 0);step=.045
 for t in np.linspace(0,T,max(3,int(T/step)+2)):
  obs=[]
  for side,seg,stationary in [('left',a,na),('right',b,nb)]:
   if seg:
    u=min(1.,t/seg['duration_s']);h=u**3*(10-15*u+6*u*u);q=np.array(seg['a'])*(1-h)+np.array(seg['b'])*h;gap=seg['gap_m']
    if seg['label']in ('empty_approach','withdraw'):gap=.08 if seg['part'].startswith('ring')else .02
   else:q=stationary;gap=.08
   obs.append(objects(side,q,gap)[0])
  for name,x in obs[0]:
   if name.endswith('link0'):continue
   for other,y in obs[1]:
    if other.endswith('link0'):continue
    if pairhit(x,y):return False
 return True

for layout in record['layouts']:
 if len(layout.get('recipes',{}))!=3:continue
 recipes=layout['recipes'];button=button_path(recipes['ring1']['q_exit'])
 if button is None:continue
 _,approach,press=button;lanes=dict(left=recipes['ring2']['segments']+recipes['pin']['segments'],right=recipes['ring1']['segments']+approach)
 L,R=lanes['left'],lanes['right'];n,m=len(L),len(R);dist={(0,0):0.};parent={};queue=[(0.,0,0)];checked=0;tested=0
 while queue:
  elapsed,i,j=heapq.heappop(queue)
  if elapsed>dist[(i,j)]+1e-9:continue
  if (i,j)==(n,m):break
  left=np.array(L[i]['a']if i<n else L[-1]['b']);right=np.array(R[j]['a']if j<m else R[-1]['b'])
  for di,dj in [(1,1),(1,0),(0,1)]:
   if i+di>n or j+dj>m:continue
   a=L[i]if di else None;b=R[j]if dj else None
   tested+=1
   if tested%50==0:print('DAG_PROGRESS',layout['label'],tested,'nodes',len(dist),flush=True)
   if not edge_clear(a,b,left,right):continue
   checked+=1;cost=max(a['duration_s']if a else 0,b['duration_s']if b else 0);nextnode=(i+di,j+dj);value=elapsed+cost
   if value<dist.get(nextnode,float('inf')):
    dist[nextnode]=value;parent[nextnode]=((i,j),di,dj,cost);heapq.heappush(queue,(value,*nextnode))
 if (n,m)not in parent:
  results.append(dict(label=layout['label'],valid=False));continue
 steps=[];node=(n,m)
 while node!=(0,0):
  previous,di,dj,cost=parent[node];i,j=previous;steps.append(dict(left_index=i if di else None,right_index=j if dj else None,duration_s=cost));node=previous
 steps.reverse();elapsed=0
 for s in steps:s['start_s']=elapsed;elapsed+=s['duration_s']
 result=dict(label=layout['label'],valid=True,cycle_time_s=elapsed+sum(s['duration_s']for s in press),coordination=steps,lanes=lanes,button_finish=press,recipes=recipes,collision_sample_dt_s=.045,checked_valid_edges=checked)
 results.append(result);(HERE/'coordinated-comparison.json').write_text(json.dumps(results,indent=2));print('COORDINATED',layout['label'],result['cycle_time_s'],checked,flush=True)
(HERE/'coordinated-comparison.json').write_text(json.dumps(results,indent=2))
