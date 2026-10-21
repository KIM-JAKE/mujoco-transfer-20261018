from pathlib import Path
import json,numpy as np,trimesh
import argparse
parser=argparse.ArgumentParser(description='Developer-only: rebuild conservative cover from bundled pinned official collision meshes (NumPy/trimesh required). No device I/O.')
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
A=Path(__file__).resolve().parents[1]/'src/isaac_port/fieldkit/assets/fr3v2-factory'
provenance=json.loads((A/'provenance.json').read_text())
import hashlib
for source in provenance['files']:
 path=A/source['packaged_file']
 if hashlib.sha256(path.read_bytes()).hexdigest()!=source['sha256']:raise ValueError('Pinned source changed: '+str(path))
def clip(poly,axis,bound,upper):
 if not len(poly):return poly
 out=[]
 for p,q in zip(poly,np.roll(poly,-1,axis=0)):
  pin=p[axis]<=bound if upper else p[axis]>=bound;qin=q[axis]<=bound if upper else q[axis]>=bound
  if pin:out.append(p)
  if pin!=qin:out.append(p+(q-p)*((bound-p[axis])/(q[axis]-p[axis])))
 return np.asarray(out)
rows=[]
for link in range(8):
 mesh=trimesh.load(A/f'link{link}.stl',process=False);v=mesh.vertices;tris=v[mesh.faces];axis=int(np.argmax(np.ptp(v,axis=0)));cross=[i for i in range(3)if i!=axis];edges=[np.linspace(v[:,i].min(),v[:,i].max(),9)for i in cross]
 for i in range(8):
  for j in range(8):
   bounds=[(edges[0][i],edges[0][i+1]),(edges[1][j],edges[1][j+1])];poly=[]
   for tri in tris:
    if any(tri[:,a].max()<lo or tri[:,a].min()>hi for a,(lo,hi)in zip(cross,bounds)):continue
    p=tri
    for a,(lo,hi)in zip(cross,bounds):p=clip(clip(p,a,lo,False),a,hi,True)
    if len(p):poly.extend(p)
   if not poly:continue
   p=np.asarray(poly);a=np.zeros(3);b=np.zeros(3)
   for k,(lo,hi)in zip(cross,bounds):a[k]=b[k]=(lo+hi)/2
   a[axis]=p[:,axis].min();b[axis]=p[:,axis].max();radius=float(np.hypot(*[(hi-lo)/2 for lo,hi in bounds])+2e-6)
   rows.append(dict(name=f'link{link}_column_{i}_{j}',link_index=link,a=a.tolist(),b=b.tolist(),radius_m=radius))
print('columns',len(rows))
args.output.write_text(json.dumps(dict(schema_version=1,kind='conservative_CAD_column_cover',transverse_grid=8,maximum_additional_skin_m=2e-6,method='Clip every original collision triangle against each transverse rectangle; enclose complete axial surface range in capsule. Interior is between first and last boundary intersections. Nonconvex gaps may be filled, never deleted.',capsules=rows),indent=2)+'\n')
