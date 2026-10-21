"""Reuse width-invariant collision proofs on one immutable planned curve.

No physics is advanced and no physical actor is moved. The caller must hold
its payload/other-arm query context and scene unchanged throughout this call.
The original full path verifier is authoritative at the first aperture.
Every body whose actual FK transform changes at another aperture is checked
again against fixed CAD AND every opposite-arm/payload proxy, with the same
existing filtering and conservative geometry as the original verifier.
"""
import numpy as np
from predictive_pair_clearance import transform_proxy, shape_overlap


def _samples(checker,qs):
    """Exactly Checker.path's sample sequence, including duplicate q corners."""
    for j,(a,b) in enumerate(zip(qs[:-1],qs[1:])):
        for sample,u in enumerate(np.linspace(0,1,max(2,int(np.ceil(np.max(abs(np.array(a)-b))/getattr(checker,'resolution_rad',.035)))+1))):
            if j and sample==0:continue
            yield j,float(u),np.array(a)*(1-u)+np.array(b)*u


def _transforms(checker,q,opening):
    transforms=checker.arm.bodies(q,opening)
    if checker.payload_path:
        transforms[checker.payload_path]=transforms[checker.arm.hand_path]@checker.payload_relative
    return transforms


def _changed_pair_clear(checker,transforms,changed):
    # This is pair_clear's exact geometry/filtering, restricted ONLY to own
    # bodies whose transforms changed from the already-accepted reference.
    # All opposite-arm bodies/payloads remain present without exclusions.
    other=checker.preview_other_boxes
    if not other:return True,None
    own=[]
    for body,source,box in checker.pair_boxes+checker.payload_boxes:
        if body in changed:own.append((body,source,transform_proxy(box,transforms[body])))
    margin=.0005
    if not own:return True,None
    def bounds(rows):
        centers=np.array([row[2][0] for row in rows]);extents=np.array([abs(row[2][1])@(row[2][2]+margin) for row in rows]);return centers-extents,centers+extents
    amin,amax=bounds(own);bmin,bmax=bounds(other)
    candidates=np.all(amax[:,None,:]>=bmin[None,:,:],axis=2)&np.all(bmax[None,:,:]>=amin[:,None,:],axis=2)
    for i,j in np.argwhere(candidates):
        body,source,box=own[i];other_body,other_source,other_box=other[j]
        if shape_overlap(box,other_box,margin=margin):
            return False,dict(query=source,actor=other_body,collider=other_source,predictive_pair_box=True,minimum_pair_clearance_m=2*margin)
    return True,None


def path_openings(checker,qs,openings,selected=None,diagnostics=None):
    """Equivalent clearance verdict to original path() for every aperture.

    Returns (clear, rejection). A rejection also identifies its aperture.
    No collision row lists, payload bindings or scene filters are replaced.
    Unknown width-dependent bodies are checked too; this does not assume
    only native fingers can move in a future robot model.
    """
    qs=np.asarray(qs);openings=np.asarray(openings,float)
    if qs.ndim!=2 or len(openings)==0 or openings.ndim!=1 or not np.isfinite(qs).all() or not np.isfinite(openings).all():
        raise ValueError('Finite path and nonempty aperture vector required')
    stats=diagnostics if diagnostics is not None else {}
    stats.update(apertures_m=openings.tolist(),reference_full_paths=1,dependent_configurations=0,invariant_configurations_reused=0,fallback_full_configurations=0,changed_body_names=[])
    accepted,why=checker.path(qs,float(openings[0]),selected)
    if not accepted:return False,dict(opening_index=0,opening_m=float(openings[0]),**why)
    # Cache only geometric FK transforms. No acceptance persists across calls.
    samples=list(_samples(checker,qs));reference=[_transforms(checker,q,float(openings[0])) for _,_,q in samples]
    for wi,width in enumerate(openings[1:],1):
        for (segment,u,q),old in zip(samples,reference):
            current=_transforms(checker,q,float(width))
            if old.keys()!=current.keys():
                # Unexpected model change is handled by the complete original
                # checker, never by guessing a subset of query shapes.
                stats['fallback_full_configurations']+=1;clear,why=checker.clear(q,float(width),selected)
            else:
                changed={body for body in current if not np.array_equal(current[body],old[body])}
                if not changed:stats['invariant_configurations_reused']+=1;continue
                stats['dependent_configurations']+=1
                stats['changed_body_names']=sorted(set(stats['changed_body_names'])|changed)
                self_guard=getattr(checker,'same_arm_clearance',None)
                if self_guard is not None:clear,why=self_guard.clear(q,float(width),changed_bodies=changed)
                else:clear,why=True,None
                if clear and checker.preview_other_boxes is not None:clear,why=_changed_pair_clear(checker,current,changed)
                if clear:
                    # Original callback retains own/stale actors, selected-tip,
                    # loose-stock and ignored-future filtering without changes.
                    clear,why=checker._clear_transforms({body:current[body]for body in changed},selected)
            if not clear:return False,dict(opening_index=wi,opening_m=float(width),segment=segment,fraction=u,collision=why)
    stats['path_sample_count']=len(samples)
    return True,None
