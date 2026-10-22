"""Offline native-self and carried-CAD/own-robot envelope CLEAR certificates.

Two independent bitsets authorize exact stored q rows. Robot finger hulls
contain the complete allowed width interval; held CAD contains the full public
RGB-D grasp uncertainty domain. Installed parts, opposite arm, fixed world,
actual entry, encoder binding and physical/contact limits remain live.
"""
from pathlib import Path
from contextlib import contextmanager
import hashlib,json,math,time
import numpy as np
from fixed_static_route_certificate import (exact_row_key,expanded_vertices,
    payload_geometry_sha256,grasp_in_envelope,rigid)
_VERSION='original-fcl-own-envelope-v1'

def _bounds(rows,transforms,margin):
    centers=np.asarray([g[0]for g in rows]);axes=np.asarray([g[1]for g in rows]);half=np.asarray([g[2]for g in rows]);T=np.asarray(transforms)
    C=np.einsum('nij,nj->ni',T[:,:3,:3],centers)+T[:,:3,3]
    A=np.matmul(T[:,:3,:3],axes);E=np.einsum('nij,nj->ni',abs(A),half+margin)+1e-9
    return C-E,C+E

class OwnRows:
    def __init__(self,metadata,native,loaded):self.metadata=metadata;self.native=set(native);self.loaded=set(loaded)
    def save(self,path):Path(path).write_text(json.dumps(dict(metadata=self.metadata,native=sorted(self.native),loaded=sorted(self.loaded)),indent=2)+'\n')
    @classmethod
    def load(cls,path):
        j=json.loads(Path(path).read_text())
        if j['metadata'].get('version')!=_VERSION:raise ValueError('Unknown own envelope certificate')
        return cls(j['metadata'],j['native'],j['loaded'])

def compile_rows(checker,qs,width_interval_m,*,world_sha256,model_sha256,nominal_hand_object_T,translation_m=.003,rotation_rad=.06):
    import fcl
    from fcl_transport import convex
    from predictive_pair_clearance import proxy
    guard=checker.same_arm_clearance
    if guard.geometries is None:raise ValueError('Original FCL own geometry backend required')
    if not checker.payload_path or not checker.payload_boxes:raise ValueError('Authored class CAD required')
    G=np.asarray(nominal_hand_object_T,float)
    if not rigid(G):raise ValueError('Rigid nominal public grasp required')
    lo,hi=map(float,width_interval_m);mid=(lo+hi)/2
    if not 0<=lo<hi<=.08 or not .0001<=translation_m<=.01 or not .0001<=rotation_rad<=.1:raise ValueError('Invalid conservative domain')
    margin=float(guard.margin_m);threshold=2*margin+1e-9
    original_rows=list(guard.rows);native_rows=[];own_rows=[]
    # Native self uses requested symmetric width. Preserve ALL original
    # authored pair exclusions; enlarge each finger geometry at the midpoint.
    for body,source,shape in original_rows:
        v=np.asarray(shape[3],float)
        if body.endswith('_finger'):
            d=(hi-lo)/4;v=expanded_vertices(np.concatenate([v+[0,-d,0],v+[0,d,0]]),1e-9)
        native_rows.append((body,source,proxy(v)))
    # Own-payload policy uses exactly original clipped proximal-finger CAD.
    # These hand-frame hulls cover EACH finger position in [lo/2,hi/2],
    # including asymmetric measured binding intervals admitted at runtime.
    for body,source,shape in guard.own_payload_clearance.robot_rows:
        v=np.asarray(shape[3],float);query_body=body
        if body.endswith('_finger'):
            side=checker.side;positive=body==checker.arm.hand_path+'/'+side+'_left_finger';sign=1 if positive else -1
            R=np.eye(3)if positive else np.diag([-1.,-1.,1.])
            v=expanded_vertices(np.concatenate([v@R.T+[0,sign*w/2,.0584]for w in [lo,hi]]),1e-9);query_body=checker.arm.hand_path
        own_rows.append((query_body,source,proxy(v)))
    payload=[]
    for _,source,shape in checker.payload_boxes:
        v=np.asarray(shape[3],float);radius=float(np.max(np.linalg.norm(v,axis=1)))
        d=translation_m+2*radius*math.sin(rotation_rad/2)+1e-6
        payload.append((checker.arm.hand_path,source,proxy(expanded_vertices(v@G[:3,:3].T+G[:3,3],d))))
    def objects(rows):
        geometries=[convex(s[3])for _,_,s in rows]
        return geometries,[fcl.CollisionObject(g)for g in geometries]
    ng,no=objects(native_rows);og,oo=objects(own_rows);pg,po=objects(payload)
    native_keys=[];loaded_keys=[];rejections=[];began=time.perf_counter()
    def place(objs,Ts):
        for obj,T in zip(objs,Ts):obj.setTransform(fcl.Transform(np.ascontiguousarray(T[:3,:3]),np.ascontiguousarray(T[:3,3])))
    def valid_pairs(a,b,pairs):
        for i,j in pairs:
            d=float(fcl.distance(a[i],b[j],fcl.DistanceRequest(),fcl.DistanceResult()))
            if not np.isfinite(d)or d<=threshold:return False
        return True
    npairs=np.asarray(guard.pairs,dtype=int).reshape(-1,2)
    for index,q in enumerate(qs):
        key=exact_row_key(q)
        if key is None:raise ValueError('Finite exact q required')
        T=checker.arm.bodies(np.asarray(q,float),mid);T[guard.base_path]=checker.arm.base
        nt=[T[b]for b,_,_ in native_rows];ot=[T[b]for b,_,_ in own_rows];pt=[T[checker.arm.hand_path]]*len(payload)
        place(no,nt);place(oo,ot);place(po,pt)
        low,high=_bounds([s for _,_,s in native_rows],nt,margin)
        a,b=npairs.T;active=np.all(high[a]>=low[b],axis=1)&np.all(high[b]>=low[a],axis=1)
        nc=valid_pairs(no,no,npairs[active])
        low,high=_bounds([s for _,_,s in own_rows],ot,margin);pl,ph=_bounds([s for _,_,s in payload],pt,margin)
        candidates=np.argwhere(np.all(high[:,None,:]>=pl[None,:,:],axis=2)&np.all(ph[None,:,:]>=low[:,None,:],axis=2))
        pc=valid_pairs(oo,po,candidates)
        if nc:native_keys.append(key)
        if pc:loaded_keys.append(key)
        rejections.append(dict(native=not nc,loaded=not pc))
        if index and index%256==0:print('OWN_ENVELOPE_COMPILE',dict(rows=index+1,native=len(native_keys),loaded=len(loaded_keys)),flush=True)
    metadata=dict(version=_VERSION,world_sha256=world_sha256,model_sha256=model_sha256,payload_geometry_sha256=payload_geometry_sha256(checker),width_interval_m=[lo,hi],nominal_hand_object_T=G.tolist(),translation_m=float(translation_m),rotation_rad=float(rotation_rad),margin_m=margin,original_native_pairs=list(map(list,guard.pairs)),compiler_s=time.perf_counter()-began)
    return OwnRows(metadata,native_keys,loaded_keys),rejections

@contextmanager
def bound_own_rows(checker,cert,*,current_world_sha256,current_model_sha256):
    sentinel=object();guard=checker.same_arm_clearance;old=getattr(guard,'scoped_own_route_certificate',sentinel)
    stats=dict(native_self_certificate_hits=0,own_payload_certificate_hits=0,own_certificate_misses=0)
    m=cert.metadata;lo,hi=m['width_interval_m'];geometry=payload_geometry_sha256(checker);version=tuple((body,source,id(shape))for body,source,shape in checker.payload_boxes)
    model_ok=(m['world_sha256']==current_world_sha256 and m['model_sha256']==current_model_sha256 and m['payload_geometry_sha256']==geometry and guard.margin_m==m['margin_m'])
    def permits(current,q,width):
        native=loaded=False
        if not model_ok or current is not checker or not np.isfinite(width)or not lo+1e-9<=width<=hi-1e-9:return False,False
        key=exact_row_key(q);native=key in cert.native
        same_geometry=version==tuple((body,source,id(shape))for body,source,shape in current.payload_boxes)
        binding=getattr(current,'loaded_contact_aperture',None)
        binding_ok=(binding is None or not binding.active or(binding.payload_path==current.payload_path and np.all(binding.lower>=lo/2+1e-9)and np.all(binding.upper<=hi/2-1e-9)))
        if key in cert.loaded and same_geometry and current.payload_path and binding_ok:
            loaded=grasp_in_envelope(np.asarray(m['nominal_hand_object_T']),current.payload_relative,m['translation_m'],m['rotation_rad'])
        stats['native_self_certificate_hits']+=int(native);stats['own_payload_certificate_hits']+=int(loaded);stats['own_certificate_misses']+=int(not native or not loaded)
        return native,loaded
    guard.scoped_own_route_certificate=permits
    try:yield stats
    finally:
        if old is sentinel:delattr(guard,'scoped_own_route_certificate')
        else:guard.scoped_own_route_certificate=old

def prepare_like_static(checker,qs,static_certificate,cache_dir):
    """Prepare while the caller has attached the SAME nominal class CAD."""
    m=static_certificate.metadata;source_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    field_sha=hashlib.sha256(np.asarray(qs,dtype=np.float64).tobytes()).hexdigest()
    from fcl_transport import convex
    import inspect
    convex_source_sha=hashlib.sha256(Path(inspect.getsourcefile(convex)).read_bytes()).hexdigest()
    identity=dict(static_domain=m,source_sha256=source_sha,convex_source_sha256=convex_source_sha,exact_field_sha256=field_sha)
    key=hashlib.sha256(json.dumps(identity,sort_keys=True,separators=(',',':')).encode()).hexdigest();target=Path(cache_dir)/('own-'+key+'.json');began=time.perf_counter();cache_hit=False
    if target.exists():
        try:
            cert=OwnRows.load(target)
            cache_hit=(cert.metadata.get('identity')==identity and cert.metadata['payload_geometry_sha256']==payload_geometry_sha256(checker))
        except(ValueError,KeyError,OSError):cache_hit=False
    if not cache_hit:
        cert,why=compile_rows(checker,qs,m['width_interval_m'],world_sha256=m['world_sha256'],model_sha256=m['model_sha256'],nominal_hand_object_T=np.asarray(m['nominal_hand_object_T']),translation_m=m['translation_m'],rotation_rad=m['rotation_rad']);cert.metadata['identity']=identity
        import os
        target.parent.mkdir(parents=True,exist_ok=True);temporary=target.with_suffix('.'+str(os.getpid())+'.tmp');cert.save(temporary);temporary.replace(target)
    return dict(certificate=cert,summary=dict(ready=True,cache_hit=cache_hit,preparation_s=time.perf_counter()-began,canonical_rows=len(qs),native_self_certified_unique_rows=len(cert.native),own_payload_certified_unique_rows=len(cert.loaded),proof_file=str(target),compile_excluded_from_cycle=True,placed_opposite_entry_physics_checks_remain_live=True))
