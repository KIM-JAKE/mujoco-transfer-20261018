"""Reuse only proofs independent of the stationary other Hand aperture.

The caller must hold the scene, both robot joint paths, local CAD, observed
payload transforms/bindings and installed-part map unchanged during this call.
No physical steps or sensor/actor-state updates occur. The first other-aperture
context gets the complete original path_openings. Later contexts recheck ALL
own robot/payload shapes against every opposite shape whose FK changed.
Unchanged own self/fixed/placed and unchanged opposite bodies retain their
already completed exact proofs. Unexpected FK keys use the original full path.
"""
import numpy as np
from aperture_path_check import path_openings, _samples
from predictive_pair_clearance import at_pose,shape_overlap


def _transforms(checker,q,width):
    transforms=checker.arm.bodies(q,float(width))
    if checker.payload_path:
        transforms[checker.payload_path]=transforms[checker.arm.hand_path]@checker.payload_relative
    return transforms


def _pair_bounds(rows):
    margin=.0005
    centers=np.array([r[2][0]for r in rows]);extents=np.array([abs(r[2][1])@(r[2][2]+margin)for r in rows])
    return centers-extents,centers+extents


def _restricted_pair(checker,q,width,other_rows,*,own_cache=None,other_bounds=None,diagnostics=None):
    if not other_rows:return True,None
    # A whole-pair CLEAR certificate also proves this changed-body subset.
    # Unknown/current-context mismatch retains the original exact narrowphase.
    certificate=getattr(checker,'scoped_gpu_pair_clear_certificate',None)
    if certificate is not None:
        try:
            if certificate(checker,q,float(width)):return True,None
        except Exception:pass
    # Only the new immutable-context entrypoints supply this local cache.
    # Original legacy callers keep their original uncached query sequence.
    key=None if own_cache is None else(np.asarray(q).tobytes(),float(width))
    cached=None if own_cache is None else own_cache.get(key)
    if cached is None:
        own=at_pose(checker,q,width,lazy=True)
        if not own:return True,None
        amin,amax=_pair_bounds(own)
        if own_cache is not None:
            # Limit memory without changing a verdict: eviction only repeats
            # original deterministic FK/projection work on the next query.
            if len(own_cache)>=32768:own_cache.clear()
            own_cache[key]=(own,amin,amax)
            if diagnostics is not None:diagnostics['own_geometry_cache_misses']=diagnostics.get('own_geometry_cache_misses',0)+1
    else:
        own,amin,amax=cached
        if diagnostics is not None:diagnostics['own_geometry_cache_hits']=diagnostics.get('own_geometry_cache_hits',0)+1
    margin=.0005
    bmin,bmax=_pair_bounds(other_rows)if other_bounds is None else other_bounds
    candidates=np.all(amax[:,None,:]>=bmin[None,:,:],axis=2)&np.all(bmax[None,:,:]>=amin[:,None,:],axis=2)
    for i,j in np.argwhere(candidates):
        body,source,box=own[i];other_body,other_source,other_box=other_rows[j]
        if shape_overlap(box,other_box,margin=margin):
            return False,dict(query=source,actor=other_body,collider=other_source,predictive_pair_box=True,minimum_pair_clearance_m=2*margin)
    return True,None


def path_dual_openings(checker,other,other_q,other_openings,qs,openings,selected=None,diagnostics=None):
    """Equivalent clearance verdict for all requested two-Hand widths."""
    qs=np.asarray(qs);other_q=np.asarray(other_q,float);widths=np.asarray(other_openings,float);own_widths=np.asarray(openings,float)
    if qs.ndim!=2 or widths.ndim!=1 or not len(widths)or own_widths.ndim!=1 or not len(own_widths)or not np.isfinite(np.r_[other_q,widths,own_widths,qs.ravel()]).all():
        raise ValueError('Finite path/other joint state and nonempty aperture vectors required')
    stats=diagnostics if diagnostics is not None else {}
    stats.update(full_reference_other_contexts=1,full_fallback_other_contexts=0,changed_other_bodies=[],pair_only_queries=0,unchanged_other_contexts_reused=0,other_apertures_m=widths.tolist(),own_apertures_m=own_widths.tolist())
    reference=_transforms(other,other_q,float(widths[0]));samples=None
    for other_index,other_width in enumerate(widths):
        current=_transforms(other,other_q,float(other_width))
        with checker.other_arm_at(other,other_q,float(other_width)):
            if other_index==0 or reference.keys()!=current.keys():
                if other_index:stats['full_fallback_other_contexts']+=1
                ok,why=path_openings(checker,qs,own_widths,selected)
                if not ok:return False,dict(other_opening_index=other_index,other_opening_m=float(other_width),**why)
                continue
            changed={body for body in current if not np.array_equal(current[body],reference[body])}
            stats['changed_other_bodies']=sorted(set(stats['changed_other_bodies'])|changed)
            if not changed:stats['unchanged_other_contexts_reused']+=1;continue
            # Reference proved every unchanged opposite collision row. Keep
            # every shape of changed bodies, including any carried class CAD.
            other_rows=[row for row in checker.preview_other_boxes if row[0]in changed]
            if not other_rows:continue
            if samples is None:samples=list(_samples(checker,qs))
            for wi,width in enumerate(own_widths):
                for segment,fraction,q in samples:
                    stats['pair_only_queries']+=1
                    ok,why=_restricted_pair(checker,q,float(width),other_rows)
                    if not ok:return False,dict(other_opening_index=other_index,other_opening_m=float(other_width),opening_index=wi,opening_m=float(width),segment=segment,fraction=fraction,collision=why)
    return True,None


def rows_dual_openings(checker,other,other_q,other_openings,qs,row_openings,selected=None,diagnostics=None):
    """Preserve each exact issued (q, aperture) pair across other-Hand widths.

    This mode never replaces a changing commanded aperture with a sparse width
    grid. The first context invokes original Checker.clear for EVERY issued
    row. Later contexts repeat the original pair narrowphase for every changed
    opposite body at that SAME row's aperture. No path interpolation is added
    or removed; this matches callers that check their exact retimed rows.
    """
    qs=np.asarray(qs,float);other_q=np.asarray(other_q,float);widths=np.asarray(other_openings,float);row_widths=np.asarray(row_openings,float)
    if qs.ndim!=2 or row_widths.shape!=(len(qs),)or not len(qs)or widths.ndim!=1 or not len(widths)or not np.isfinite(np.r_[other_q,widths,row_widths,qs.ravel()]).all():
        raise ValueError('Finite issued q/aperture pairs and nonempty opposite aperture vector required')
    stats=diagnostics if diagnostics is not None else {}
    stats.update(exact_issued_rows=len(qs),full_reference_other_contexts=1,full_fallback_other_contexts=0,changed_other_bodies=[],pair_only_queries=0,unchanged_other_contexts_reused=0,other_apertures_m=widths.tolist(),per_row_apertures_preserved=True)
    reference=_transforms(other,other_q,float(widths[0]))
    for other_index,other_width in enumerate(widths):
        current=_transforms(other,other_q,float(other_width))
        with checker.other_arm_at(other,other_q,float(other_width)):
            full=other_index==0 or reference.keys()!=current.keys()
            if other_index and full:stats['full_fallback_other_contexts']+=1
            changed=set()if full else {body for body in current if not np.array_equal(current[body],reference[body])}
            stats['changed_other_bodies']=sorted(set(stats['changed_other_bodies'])|changed)
            if not full and not changed:stats['unchanged_other_contexts_reused']+=1;continue
            other_rows=[]if full else[row for row in checker.preview_other_boxes if row[0]in changed]
            for i,(q,width) in enumerate(zip(qs,row_widths)):
                if full:ok,why=checker.clear(q,float(width),selected)
                else:
                    stats['pair_only_queries']+=1;ok,why=_restricted_pair(checker,q,float(width),other_rows)
                if not ok:return False,dict(row=i,other_opening_index=other_index,other_opening_m=float(other_width),collision=why)
    return True,None


def path_other_contexts(checker,other,contexts,qs,openings,selected=None,diagnostics=None):
    """Reuse own fixed/self proofs across an immutable set of other poses.

    This is the same reference/full and changed-body/pair-only proof used by
    path_dual_openings, including every path sample and original narrowphase.
    Every accepted context becomes the next pair-proof reference, so switching
    to a new joint pose does not repeat its unchanged links at every aperture.
    The caller may not advance physics, change binding, CAD, payload, scene or
    own path/apertures until this function returns. No cross-call cache exists.
    """
    qs=np.asarray(qs,float);own_widths=np.asarray(openings,float)
    contexts=[(np.asarray(q,float).copy(),float(w))for q,w in contexts]
    if qs.ndim!=2 or qs.shape[1]!=7 or own_widths.ndim!=1 or not len(own_widths)or not contexts or not np.isfinite(np.r_[qs.ravel(),own_widths]).all():
        raise ValueError('Finite immutable own path/apertures and other contexts required')
    for q,w in contexts:
        if q.shape!=(7,)or not np.isfinite(np.r_[q,w]).all()or not 0<=w<=.08:
            raise ValueError('Invalid other encoder/aperture context')
    stats=diagnostics if diagnostics is not None else {}
    stats.update(full_reference_other_contexts=1,full_fallback_other_contexts=0,changed_other_bodies=[],pair_only_queries=0,unchanged_other_contexts_reused=0,other_context_count=len(contexts),own_apertures_m=own_widths.tolist(),cross_joint_context_reuse=True)
    reference=_transforms(other,*contexts[0]);samples=None;own_cache={}
    for index,(other_q,other_width)in enumerate(contexts):
        current=_transforms(other,other_q,other_width)
        with checker.other_arm_at(other,other_q,other_width):
            if index==0 or reference.keys()!=current.keys():
                if index:stats['full_fallback_other_contexts']+=1
                ok,why=path_openings(checker,qs,own_widths,selected)
                if not ok:return False,dict(other_context_index=index,other_q=other_q.tolist(),other_opening_m=other_width,**why)
                reference=current
                continue
            changed={body for body in current if not np.array_equal(current[body],reference[body])}
            stats['changed_other_bodies']=sorted(set(stats['changed_other_bodies'])|changed)
            if not changed:stats['unchanged_other_contexts_reused']+=1;reference=current;continue
            other_rows=[row for row in checker.preview_other_boxes if row[0]in changed]
            if not other_rows:reference=current;continue
            other_bounds=_pair_bounds(other_rows)
            if samples is None:samples=list(_samples(checker,qs))
            for wi,width in enumerate(own_widths):
                for segment,fraction,q in samples:
                    stats['pair_only_queries']+=1
                    ok,why=_restricted_pair(checker,q,float(width),other_rows,own_cache=own_cache,other_bounds=other_bounds,diagnostics=stats)
                    if not ok:return False,dict(other_context_index=index,other_q=other_q.tolist(),other_opening_m=other_width,opening_index=wi,opening_m=float(width),segment=segment,fraction=fraction,collision=why)
            reference=current
    return True,None


def rows_other_contexts(checker,other,contexts,qs,row_openings,selected=None,diagnostics=None):
    """Identical exact-row checks with immutable cross-joint proof reuse.

    Unlike path mode this adds NO interpolated rows or aperture combinations.
    Each q retains its original issued aperture. The no-mutation contract of
    path_other_contexts applies until return; every accepted other context is
    the reference for the next context's changed-body narrowphase.
    """
    qs=np.asarray(qs,float);row_widths=np.asarray(row_openings,float)
    contexts=[(np.asarray(q,float).copy(),float(w))for q,w in contexts]
    if qs.ndim!=2 or qs.shape[1]!=7 or not len(qs)or row_widths.shape!=(len(qs),)or not contexts or not np.isfinite(np.r_[qs.ravel(),row_widths]).all():
        raise ValueError('Finite immutable exact issued q/aperture pairs required')
    for q,w in contexts:
        if q.shape!=(7,)or not np.isfinite(np.r_[q,w]).all()or not 0<=w<=.08:
            raise ValueError('Invalid other encoder/aperture context')
    stats=diagnostics if diagnostics is not None else {}
    stats.update(exact_issued_rows=len(qs),full_reference_other_contexts=1,full_fallback_other_contexts=0,changed_other_bodies=[],pair_only_queries=0,unchanged_other_contexts_reused=0,other_context_count=len(contexts),per_row_apertures_preserved=True,cross_joint_context_reuse=True)
    reference=_transforms(other,*contexts[0]);own_cache={}
    for index,(other_q,other_width)in enumerate(contexts):
        current=_transforms(other,other_q,other_width)
        full=index==0 or reference.keys()!=current.keys()
        if index and full:stats['full_fallback_other_contexts']+=1
        changed=set()if full else{body for body in current if not np.array_equal(current[body],reference[body])}
        stats['changed_other_bodies']=sorted(set(stats['changed_other_bodies'])|changed)
        if not full and not changed:stats['unchanged_other_contexts_reused']+=1;reference=current;continue
        with checker.other_arm_at(other,other_q,other_width):
            other_rows=[]if full else[row for row in checker.preview_other_boxes if row[0]in changed]
            other_bounds=None if not other_rows else _pair_bounds(other_rows)
            for i,(q,width)in enumerate(zip(qs,row_widths)):
                if full:ok,why=checker.clear(q,float(width),selected)
                else:
                    stats['pair_only_queries']+=1;ok,why=_restricted_pair(checker,q,float(width),other_rows,own_cache=own_cache,other_bounds=other_bounds,diagnostics=stats)
                if not ok:return False,dict(row=i,other_context_index=index,other_q=other_q.tolist(),other_opening_m=other_width,collision=why)
        reference=current
    return True,None
