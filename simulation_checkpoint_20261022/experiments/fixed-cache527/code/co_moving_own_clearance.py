"""Scope-local ORIGINAL FCL CLEAR reuse for co-moving native Hand rows only.

The actual relative geometry is checked on every query. No acceptance crosses
this immutable planning scope; unknown/boundary/mutation uses original FCL.
"""
from contextlib import contextmanager
import numpy as np

_RESERVE=1e-6

def _relative(H,T):
    return np.linalg.solve(H,np.asarray(T,float))

def _rigid(T,tolerance=1e-12):
    return (T.shape==(4,4)and np.isfinite(T).all()and np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))<=tolerance and abs(np.linalg.det(T[:3,:3])-1)<=tolerance and np.max(abs(T[3]-[0,0,0,1]))<=1e-14 and np.max(abs(T[:3,3]))<=10)

class Memo:
    def __init__(self,checker):
        self.checker=checker;self.entries={};self.geometry={};self.stats=dict(queries=0,fast_attempts=0,fast_returns=0,clear_queries=0,skipped_candidate_pairs=0,first_seen=0,proofs=0,unknown=0,relative_misses=0,maximum_nonrigid_point_bound_m=0.,numeric_reserve_m=_RESERVE)
    def clear_rows(self,checker,H,rows,Ts,objects,payload,payload_objects,width,margin,*,allow_compute=True,count_query=True):
        import fcl
        if count_query:self.stats['queries']+=1
        if not np.isfinite(margin)or not 0<=margin<=.002:return set()
        if checker is not self.checker or width is None or not np.isfinite(width)or not 0<=width<=.08:return set()
        H=np.asarray(H,float);G=np.asarray(checker.payload_relative,float)
        # Held G is an exact-keyed fixed linear map; preserve the original
        # accepted calibration precision, without orthogonalizing its matrix.
        # The changing Hand is validated at original calibration precision;
        # its small nonrigid component is explicitly bounded below.
        if not _rigid(H,1e-6)or not _rigid(G,1e-6):return set()
        hand=checker.arm.hand_path;side=hand.rsplit('/',1)[-1].split('_')[0]
        allowed={hand,hand+'/'+side+'_left_finger',hand+'/'+side+'_right_finger'}
        ids=[i for i,r in enumerate(rows)if r[0]in allowed]
        if not ids or not payload:return set()
        key=(id(checker.arm),checker.payload_path,G.tobytes(),np.float64(width).tobytes(),float(margin),tuple((i,rows[i][0],rows[i][1],id(rows[i][2]))for i in ids),tuple((b,s,id(g))for b,s,g in payload))
        sentinel=object();previous=self.entries.get(key,sentinel)
        if previous is sentinel:
            if not allow_compute:return set()
            if len(self.entries)>=256:self.entries.clear()
            self.entries[key]=None;self.stats['first_seen']+=1;return set()
        relative=np.asarray([_relative(H,Ts[i])for i in ids])
        if not all(_rigid(t)for t in relative):return set()
        if previous is None:
            if not allow_compute:return set()
            # All original query objects are already at THIS query's original
            # world transforms. Full all-payload FCL proof is stronger than
            # the original OBB-filtered subset, with an inward numeric reserve.
            distance=float('inf')
            for i in ids:
                for obj in payload_objects:
                    d=float(fcl.distance(objects[i],obj,fcl.DistanceRequest(),fcl.DistanceResult()))
                    if not np.isfinite(d):distance=float('-inf');break
                    distance=min(distance,d)
                if distance<=2*margin+_RESERVE:break
            radii=np.asarray([np.max(np.linalg.norm(np.asarray(rows[i][2][3]),axis=1))for i in ids])
            if not np.isfinite(radii).all()or np.any(radii<0):return set()
            hand_scale=float(np.linalg.norm(H[:3,:3],ord=2))
            robot_radius=max(float(np.linalg.norm(t[:3,3])+np.linalg.norm(t[:3,:3],ord=2)*r)for t,r in zip(relative,radii))
            payload_radius=max(float(np.max(np.linalg.norm(np.asarray(g[3]),axis=1)))for _,_,g in payload)
            payload_radius=float(np.linalg.norm(G[:3,3])+np.linalg.norm(G[:3,:3],ord=2)*payload_radius)
            previous=dict(relative=relative.copy(),radii=radii,distance=distance,hand_inverse=np.linalg.inv(H[:3,:3]),world_radius_sum=hand_scale*(robot_radius+payload_radius))
            self.entries[key]=previous;self.stats['proofs']+=1
        clearance=previous['distance']-2*margin
        if not np.isfinite(clearance)or clearance<=_RESERVE:self.stats['unknown']+=1;return set()
        # Hausdorff point-displacement bound in the current Hand frame.
        # Payload G is exact-keyed; only actual native relative row transforms
        # may differ. Frobenius norm bounds the rotational operator norm.
        if not np.isfinite(previous['radii']).all()or not np.isfinite(previous['world_radius_sum'])or previous['world_radius_sum']<0:return set()
        delta=relative-previous['relative'];drift=np.linalg.norm(delta[:,:3,3],axis=1)+np.linalg.norm(delta[:,:3,:3],axis=(1,2))*previous['radii']
        # Authored FK matrices have small, preserved nonorthogonal roundoff.
        # Bound the actual change from the nearest rigid world motion rather
        # than normalize any commanded/query pose or assume exact rigidity.
        if not np.isfinite(drift).all():return set()
        A=H[:3,:3]@previous['hand_inverse']
        if not np.isfinite(A).all():return set()
        U,_,V=np.linalg.svd(A);Q=U@V
        if np.linalg.det(Q)<0:return set()
        affine_bound=float(np.linalg.norm(A-Q))*previous['world_radius_sum']
        self.stats['maximum_nonrigid_point_bound_m']=max(self.stats['maximum_nonrigid_point_bound_m'],affine_bound)
        error=float(np.linalg.norm(H[:3,:3],ord=2))*float(np.max(drift))+affine_bound+_RESERVE
        if not np.isfinite(error)or error<0 or error>=clearance:self.stats['relative_misses']+=1;return set()
        if count_query:self.stats['clear_queries']+=1
        return set(ids)


    def fast_clear(self,checker,H,rows,Ts,payload,width,margin):
        self.stats['fast_attempts']+=1
        co=self.clear_rows(checker,H,rows,Ts,None,payload,None,width,margin,allow_compute=False,count_query=False)
        if not co:return False
        remaining=[i for i in range(len(rows))if i not in co]
        if not remaining:self.stats['fast_returns']+=1;return True
        version=(tuple((b,s,id(g))for b,s,g in rows),tuple((b,s,id(g))for b,s,g in payload))
        packed=self.geometry.get(version)
        if packed is None:
            def local(items):return tuple(np.asarray([r[2][k]for r in items],float)for k in [0,1,2])
            packed=(local(rows),local(payload))
            if not all(np.isfinite(a).all()for parts in packed for a in parts)or any(np.any(parts[2]<0)for parts in packed):return False
            if len(self.geometry)>=16:self.geometry.clear()
            self.geometry[version]=packed
        def bounds(parts,T):
            centers,axes,half=parts;R=T[:,:3,:3]
            C=np.einsum('nij,nj->ni',R,centers)+T[:,:3,3];A=np.matmul(R,axes)
            # SAME original OBB-margin predicate, enlarged outward to retain
            # every numerical boundary. An overlap uses original full FCL.
            E=np.einsum('nij,nj->ni',abs(A),half+margin)+_RESERVE
            return C-E,C+E
        full_T=np.asarray(Ts,float)
        if full_T.shape!=(len(rows),4,4)or not np.isfinite(full_T).all():return False
        lo,hi=bounds(packed[0],full_T)
        P=np.asarray(H)@np.asarray(checker.payload_relative)
        pl,ph=bounds(packed[1],np.broadcast_to(P,(len(payload),4,4)))
        if not all(np.isfinite(a).all()for a in [lo,hi,pl,ph]):return False
        lo,hi=lo[remaining],hi[remaining]
        overlap=np.all(hi[:,None,:]>=pl[None,:,:],axis=2)&np.all(ph[None,:,:]>=lo[:,None,:],axis=2)
        if np.any(overlap):return False
        self.stats['fast_returns']+=1;return True

@contextmanager
def co_moving_scope(c,*checkers):
    sentinel=object();installed=[]
    try:
        for checker in checkers:
            own=getattr(getattr(checker,'same_arm_clearance',None),'own_payload_clearance',None)
            if own is None or getattr(own,'robot_geometries',None)is None:continue
            previous=getattr(own,'scoped_co_moving_proof',sentinel);memo=Memo(checker);own.scoped_co_moving_proof=memo;installed.append((own,previous,memo))
        yield
    finally:
        for own,previous,memo in installed:
            if previous is sentinel:delattr(own,'scoped_co_moving_proof')
            else:own.scoped_co_moving_proof=previous
            c['planning_diagnostics'].append(dict(event='co_moving_native_own_FCL_scope',side=getattr(memo.checker,'side','unknown'),scope='one immutable preflight; original world FCL all-payload proof, actual Hand-relative geometry checked each query',all_other_guards_unchanged=True,**memo.stats))
            memo.entries.clear();memo.geometry.clear()
