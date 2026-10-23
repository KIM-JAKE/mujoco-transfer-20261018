"""Strict interior of CLOSED official visual Hand components, never a hull.

Open palm material meshes are excluded, not filled or convexified. Signed
closed components preserve cavities. A whole voxel is attributed to this body
only when its bounding sphere plus explicit uncertainty lies inside material.
"""
from functools import lru_cache
from pathlib import Path
import hashlib,xml.etree.ElementTree as ET
import numpy as np
from .calibration import rigid

ROOT=Path(__file__).resolve().parents[1]/'isaac_port/fieldkit/assets/native-hand'
NS={'c':'http://www.collada.org/2005/11/COLLADASchema'}


def _closed(vertices,faces):
    edges=np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]);ordered=np.sort(edges,axis=1);_,inverse,count=np.unique(ordered,axis=0,return_inverse=True,return_counts=True)
    if not np.all(count==2):return False
    signs=np.where(edges[:,0]<edges[:,1],1,-1);return bool(np.all(np.bincount(inverse,weights=signs)==0))


@lru_cache(maxsize=1)
def models():
    result={};audit=[]
    for file,body in [('hand-visual.dae','palm'),('finger.dae','finger')]:
        path=ROOT/file;tree=ET.parse(path);asset=tree.find('c:asset',NS)
        if asset.find('c:unit',NS).get('meter')!='1'or asset.find('c:up_axis',NS).text!='Z_UP':raise ValueError('Official Hand visual meter/Z_UP asset required')
        # Exact source currently has identity scene-node transforms only.
        for matrix in tree.findall('.//c:visual_scene//c:matrix',NS):
            if not np.array_equal(np.fromstring(matrix.text,sep=' ').reshape(4,4),np.eye(4)):raise ValueError('Unimplemented visual-node transform; do not assume raw vertices physical')
        solids=[]
        for geometry in tree.findall('.//c:library_geometries/c:geometry',NS):
            mesh=geometry.find('c:mesh',NS);sources={}
            for src in mesh.findall('c:source',NS):
                arr=src.find('c:float_array',NS);acc=src.find('c:technique_common/c:accessor',NS)
                if arr is not None:sources[src.get('id')]=np.fromstring(arr.text,sep=' ').reshape(-1,int(acc.get('stride')))
            references={v.get('id'):v.find("c:input[@semantic='POSITION']",NS).get('source')[1:]for v in mesh.findall('c:vertices',NS)}
            for tri in mesh.findall('c:triangles',NS):
                inputs=tri.findall('c:input',NS);inp=next(i for i in inputs if i.get('semantic')=='VERTEX');stride=max(int(i.get('offset'))for i in inputs)+1
                faces=np.fromstring(tri.find('c:p',NS).text,sep=' ',dtype=int).reshape(-1,stride)[:,int(inp.get('offset'))].reshape(-1,3)
                v=sources[references[inp.get('source')[1:]]][:,:3];v,inv=np.unique(v,axis=0,return_inverse=True);faces=inv[faces];closed=_closed(v,faces)
                audit.append(dict(file=file,geometry=geometry.get('id'),closed_oriented=closed,faces=len(faces),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
                if closed:solids.append((v,faces))
        result[body]=tuple(solids)
    return result,tuple(audit)


def _distances(points,triangles):
    """Exact closest-triangle distances, bounded batches and NumPy only."""
    best=np.full(len(points),np.inf)
    for start in range(0,len(triangles),128):
        tri=triangles[start:start+128];a=tri[:,0];ab=tri[:,1]-a;ac=tri[:,2]-a;p=points[:,None,:]-a
        n=np.cross(ab,ac);n2=np.sum(n*n,axis=1);den=np.maximum(n2,np.finfo(float).tiny)
        u=np.sum(np.cross(p,ac)*n,axis=2)/den;v=np.sum(np.cross(ab,p)*n,axis=2)/den
        plane=np.sum(p*n,axis=2)**2/den;dist=np.where((u>=0)&(v>=0)&(u+v<=1)&(n2>0),plane,np.inf)
        for x,y in [(tri[:,0],tri[:,1]),(tri[:,1],tri[:,2]),(tri[:,2],tri[:,0])]:
            d=y-x;norm=np.sum(d*d,axis=1);fraction=np.clip(np.sum((points[:,None,:]-x)*d,axis=2)/np.maximum(norm,np.finfo(float).tiny),0,1)
            dist=np.minimum(dist,np.sum((points[:,None,:]-x-fraction[:,:,None]*d)**2,axis=2))
        best=np.minimum(best,dist.min(1))
    return np.sqrt(best)


def contains_eroded(points,vertices,faces,erosion_m):
    p=np.asarray(points,float);result=np.zeros(len(p),bool)
    candidate=np.flatnonzero(np.all((p>vertices.min(0)+erosion_m)&(p<vertices.max(0)-erosion_m),axis=1))
    if not len(candidate):return result
    tri=vertices[faces]
    for offset in range(0,len(candidate),256):
        ids=candidate[offset:offset+256];x=p[ids];winding=np.zeros(len(x))
        for start in range(0,len(tri),128):
            delta=tri[start:start+128][None]-x[:,None,None];a,b,c=delta[:,:,0],delta[:,:,1],delta[:,:,2];la=np.linalg.norm(a,axis=2);lb=np.linalg.norm(b,axis=2);lc=np.linalg.norm(c,axis=2)
            numerator=np.sum(a*np.cross(b,c),axis=2);denominator=la*lb*lc+np.sum(a*b,axis=2)*lc+np.sum(b*c,axis=2)*la+np.sum(c*a,axis=2)*lb
            winding+=np.sum(2*np.arctan2(numerator,denominator),axis=1)/(4*np.pi)
        # Require positive material winding. Negative cavity components retain
        # their sign; no absolute value or arbitrary hole filling is used.
        inside=winding>1-1e-6
        if np.any(inside):result[ids[inside]]=_distances(x[inside],tri)>erosion_m
    return result


@lru_cache(maxsize=1)
def collision_envelope_models():
    path=ROOT.parent/'fr3v2-factory/collision-models.npz';result={};audit=[]
    with np.load(path,allow_pickle=False)as z:
        for name in ['palm','finger']:
            raw=z[name+'_vertices']
            if name=='finger':
                with np.load(ROOT/'collision-finger.npz',allow_pickle=False)as finger:
                    if not np.array_equal(raw,finger['vertices']):raise ValueError('Finger collision topology does not match pinned envelope vertices')
                    source_faces=finger['faces'].copy()
            else:source_faces=np.arange(len(raw)).reshape(-1,3)
            v,inv=np.unique(raw,axis=0,return_inverse=True);faces=inv[source_faces]
            if not _closed(v,faces):raise ValueError('Pinned official collision envelope must be closed/oriented; no automatic hull or repair')
            result[name]=((v,faces),);audit.append(dict(model=name,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),topology_source_sha256=hashlib.sha256((ROOT/'collision-finger.npz').read_bytes()).hexdigest()if name=='finger'else hashlib.sha256(path.read_bytes()).hexdigest(),geometry_basis='PINNED_COLLISION_ENVELOPE_ASSUMPTION_not_physical_visual_solid',closed_oriented=True,faces=len(faces)))
    return result,tuple(audit)


def material_owner(snapshot,actual_states,geometry_uncertainty_m,*,collision_envelope=False):
    from .residual_occupancy import UNKNOWN,FREE
    if type(geometry_uncertainty_m)not in(int,float)or not np.isfinite(geometry_uncertainty_m)or not 0<geometry_uncertainty_m<=.003:raise ValueError('Explicit actual installed visual-geometry uncertainty required')
    bounds=np.asarray(snapshot.bounds_cell_m);spacing=(bounds[1]-bounds[0])/snapshot.shape;voxel_radius=float(np.linalg.norm(spacing)/2);ids=np.flatnonzero(snapshot.states.ravel()!=FREE if collision_envelope else snapshot.states.ravel()==UNKNOWN);indices=np.array(np.unravel_index(ids,snapshot.shape)).T;points=bounds[0]+(indices+.5)*spacing;owner=np.zeros(int(np.prod(snapshot.shape)),np.uint8);owners=[];model,audit=collision_envelope_models()if collision_envelope else models()
    for state in actual_states:
        T=rigid(state['T_cell_tcp'])@np.linalg.inv(rigid(state['F_T_EE']))@rigid(state['T_flange_hand']);local=(points-T[:3,3])@T[:3,:3]
        uncertainty=state['pose_uncertainty_m']+geometry_uncertainty_m+max(s['depth_uncertainty_m']for s in snapshot.sources)
        for body in ['palm','left_finger','right_finger']:
            owners.append((state['side'],body));code=len(owners);q=local.copy();meshes=model['palm'if body=='palm'else'finger']
            if body!='palm':
                sign=1 if body=='left_finger'else-1;q[:,2]-=.0584;q[:,1]-=sign*state['width_m']/2;q[:,:2]*=sign
            contained=np.zeros(len(q),bool)
            for vertices,faces in meshes:contained|=contains_eroded(q,vertices,faces,voxel_radius+uncertainty)
            # A claimed voxel cannot belong to two robot bodies. Such cases
            # retain UNKNOWN rather than selecting an arbitrary owner.
            overlap=(owner[ids]!=0)&contained;owner[ids[contained]]=code;owner[ids[overlap]]=255
    owner[owner==255]=0
    return owner.reshape(snapshot.shape),tuple(owners),dict(kind='eroded_pinned_official_collision_envelope_assumption'if collision_envelope else'strictly_eroded_closed_official_visual_material',erosion_includes_whole_voxel_sphere=True,geometry_uncertainty_m=geometry_uncertainty_m,open_palm_meshes_ignored=not collision_envelope,collision_envelope_assumption=collision_envelope,physical_solid_certified=False,free_voxels_created=0,source_components=list(audit),typed_unknown_voxels=int(np.count_nonzero(owner)))
