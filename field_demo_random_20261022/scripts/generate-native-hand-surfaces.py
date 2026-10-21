#!/usr/bin/env python3
"""Offline fixed official CAD sampling; requires NumPy, no device I/O."""
from pathlib import Path
import numpy as np,xml.etree.ElementTree as E,json,hashlib
R=Path(__file__).resolve().parents[1]/'src/isaac_port/fieldkit/assets/native-hand';ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
def load(p):
 root=E.parse(p).getroot();out=[]
 for mesh in root.findall('.//c:mesh',ns):
  sources={v.attrib['id']:np.fromstring(v.find('c:float_array',ns).text,sep=' ').reshape(-1,int(v.find('c:technique_common/c:accessor',ns).attrib['stride']))for v in mesh.findall('c:source',ns)}
  vertices={v.attrib['id']:v.find('c:input',ns).attrib['source'][1:]for v in mesh.findall('c:vertices',ns)}
  for tri in mesh.findall('c:triangles',ns):
   inputs=tri.findall('c:input',ns);stride=max(int(i.attrib['offset'])for i in inputs)+1;v=next(i for i in inputs if i.attrib['semantic']=='VERTEX');idx=np.fromstring(tri.find('c:p',ns).text,sep=' ',dtype=int).reshape(-1,stride)[:,int(v.attrib['offset'])];triangles=sources[vertices[v.attrib['source'][1:]]][idx].reshape(-1,3,3)
   for t in triangles:
    n=max(1,int(np.ceil(max(np.linalg.norm(t[1]-t[0]),np.linalg.norm(t[2]-t[0]),np.linalg.norm(t[2]-t[1]))/.0007)))
    out.extend(t[0]+i/n*(t[1]-t[0])+j/n*(t[2]-t[0])for i in range(n+1)for j in range(n+1-i))
 return np.unique(np.round(np.asarray(out),8),axis=0)
arrays={k:load(R/n)for k,n in [('palm_points','hand-visual.dae'),('finger_points','finger.dae')]};arrays={k:v[np.sort(np.unique(np.floor(v/.00035).astype(int),axis=0,return_index=True)[1])]for k,v in arrays.items()};np.savez_compressed(R/'surfaces.npz',**arrays);print({k:len(v)for k,v in arrays.items()})
(R/'surface-provenance.json').write_text(json.dumps(dict(source='official franka_description fixed visual CAD; no actor poses',sampling_max_triangle_edge_m=.0007,voxel_subsample_m=.00035,maximum_additional_sample_displacement_m=float(3**.5*.00035),source_sha256={n:hashlib.sha256((R/n).read_bytes()).hexdigest()for n in ['hand-visual.dae','finger.dae']},generated_arrays={k:len(v)for k,v in arrays.items()}),indent=2)+'\n')
