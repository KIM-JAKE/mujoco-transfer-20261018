"""Move empty-arm prepositioning into other arm's independent pickup time.

This is a candidate only. Full contact simulation must accept the result.
"""
from pathlib import Path
import numpy as np,json
HERE=Path(__file__).resolve().parent;root=HERE.parents[1]/'runs'
src=root/'native-roas-table-bin-under30-capture01';dst=root/'native-roas-table-bin-parallel-source02';dst.mkdir(exist_ok=True)
z=np.load(src/'full-command-trace.npz');old=z['time_s'];report=json.load(open(src/'pipeline-results.json'));names=report['dof_names'];q=z['command'];labels=z['phase'];N=len(old)
def span(label):
 ids=np.flatnonzero(labels==label);return old[ids[0]-1],old[ids[-1]]
a,b=span('recipient_route_ring1');c,d=span('recipient_route_ring2')
left=old.copy();right=old.copy()
for start,end,new_start,new_end in [(a,b,2.,20.95),(c,d,42.98,48.8)]:
 mask=(old>=new_start)&(old<=new_end);left[mask]=start+(old[mask]-new_start)*(end-start)/(new_end-new_start);left[(old>new_end)&(old<end)]=end
shift=75.7625-70.6833333333
right[old>=70.6833333333]=np.minimum(old[old>=70.6833333333]+shift,old[-1])
outq=q.copy()
for side,clock in [('left',left),('right',right)]:
 for i,n in enumerate(names):
  if n.startswith(side+'_'):outq[:,i]=np.interp(clock,old,q[:,i])
# Smooth the opening before the early second route, without moving the arm.
for i,n in enumerate(names):
 if n.startswith('left_native'):
  x=(old>=42.98)&(old<43.5);u=np.clip((old[x]-42.98)/.4,0,1);v=np.interp(42.98,old,q[:,i]);outq[x,i]=v+(.04-v)*(u*u*(3-2*u))
control=z['clamp'].copy();control[:,2]=np.interp(right,old,control[:,2])
ph=[]
for l,r in zip(left,right):
 ll=str(labels[min(N-1,np.searchsorted(old,l))]).split(' || ')[0];rr=str(labels[min(N-1,np.searchsorted(old,r))]).split(' || ')[-1];ph.append(ll+' || '+rr)
end=np.searchsorted(old,old[-1]-shift)+1
np.savez_compressed(dst/'full-command-trace.npz',time_s=old[:end],command=outq[:end],clamp=control[:end],phase=ph[:end]);report.update(full_task_pass=False,continuous_cycle_pass=False,cycle_time_s=None,state='unvalidated_parallel_candidate');(dst/'pipeline-results.json').write_text(json.dumps(report))
print('PARALLEL_CANDIDATE',dst,'routes',(a,b,c,d),'jump',np.max(abs(np.diff(outq[:end],axis=0))),flush=True)
