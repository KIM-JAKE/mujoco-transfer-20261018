"""One-time approximate ESDF of fixed CAD; original full checker stays required."""
from pathlib import Path
import time,json,hashlib,numpy as np,trimesh,warp as wp
from service_paths import ROOT as root
wp.init()
@wp.kernel
def esdf(mesh:wp.uint64,values:wp.array(dtype=wp.float32),origin:wp.vec3,voxel:wp.float32,ny:int,nz:int):
 i=wp.tid();x=i//(ny*nz);y=(i//nz)%ny;z=i%nz;p=origin+wp.vec3((float(x)+.5)*voxel,(float(y)+.5)*voxel,(float(z)+.5)*voxel)
 q=wp.mesh_query_point(mesh,p,.4);d=float(.4)
 if q.result:
  closest=wp.mesh_eval_position(mesh,q.face,q.u,q.v);d=wp.length(p-closest)*q.sign
 values[i]=d
start=time.perf_counter();source=root/'fixed-CAD.ply';m=trimesh.load(source,process=False);mesh=wp.Mesh(points=wp.array(m.vertices.astype(np.float32),dtype=wp.vec3,device='cuda:0'),indices=wp.array(m.faces.ravel().astype(np.int32),dtype=wp.int32,device='cuda:0'));shape=[300,350,300];size=.008;dims=np.array(shape)*size;center=np.array([0.,-.2,1.2]);origin=center-dims/2;values=wp.empty(int(np.prod(shape)),dtype=wp.float32,device='cuda:0');wp.launch(esdf,dim=len(values),inputs=[mesh.id,values,wp.vec3(*origin),size,shape[1],shape[2]],device='cuda:0');wp.synchronize();arr=values.numpy().reshape(shape).astype(np.float16);np.save(root/'fixed-proposal-esdf.npy',arr);manifest=dict(shape=shape,dims_m=dims.tolist(),center_world_m=center.tolist(),voxel_size_m=size,source_mesh_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),max_query_distance_m=.4,elapsed_s=time.perf_counter()-start,approximate_proposal_only=True,original_fixed_pair_payload_FR3_guards_mandatory=True);(root/'fixed-proposal-esdf.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest))
