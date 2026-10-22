"""One immutable exact-state scope for conservative GPU pair CLEAR proofs.

GPU uncertainty, timeout, unsupported geometry or any context mismatch retains
the original CPU pair test. Fixed-world, own self and payload queries remain
the responsibility of the unchanged Checker.clear calls in the caller.
"""
from contextlib import contextmanager
import time
import numpy as np


def _qkey(q):
    q=np.asarray(q)
    return(q.dtype.str,q.shape,q.tobytes())


def _state_key(left,lw,right,rw):
    return(_qkey(left),np.float64(lw).tobytes(),_qkey(right),np.float64(rw).tobytes())


def _geometry_version(checker):
    relative=getattr(checker,'payload_relative',None)
    return(id(checker.arm),tuple((body,source,id(box))for body,source,box in checker.pair_boxes),getattr(checker,'payload_path',None),tuple((body,source,id(box))for body,source,box in checker.payload_boxes),None if relative is None else _qkey(relative))


@contextmanager
def certified_pair_scope(c,left,right,states=None,*,pairs=None):
    """Authorize only whole-pair CLEAR rows from this one frozen schedule."""
    config=c['config']
    if not config.get('gpu_pair_schedule_certificate',False):
        yield
        return
    if pairs is None:pairs=[(l[:7],float(l[7:].sum()),r[:7],float(r[7:].sum()))for l,r in states]
    else:pairs=list(pairs)
    started=time.perf_counter();stats=dict(event='GPU_exact_pair_schedule_certificate',schedule_states=len(pairs),GPU_clear_states=0,CPU_pair_fallback_states=len(pairs),exact_scope_hits=0,context_misses=0,uses_free_body_poses=False,own_fixed_self_checks_preserved=True,cache_survives_physics=False)
    approved=set();installed=False;sentinel=object();old_left=old_right=sentinel
    try:
        try:
            from gpu_pair_schedule import prove
            left_version=_geometry_version(left);right_version=_geometry_version(right)
            diagnostics={}
            socketcfg=dict(socket=config.get('gpu_pair_schedule_socket','/workspace/project/mujoco/.local/collision-gpu2/pair.sock'),timeout_s=float(config.get('gpu_pair_schedule_timeout_s',5.)),diagnostics=diagnostics)
            mask=np.asarray(prove(left,right,pairs,socketcfg))
            if mask.shape!=(len(pairs),)or mask.dtype!=np.bool_:
                raise RuntimeError('GPU pair proof must return one boolean CLEAR bit per exact state')
            # Geometry is fixed local CAD plus observed held transforms. The
            # helper registers its canonical SHA; identity/transform guards
            # below also reject any in-scope Python geometry replacement.
            if _geometry_version(left)!=left_version or _geometry_version(right)!=right_version:
                raise RuntimeError('Geometry changed while GPU pair schedule was being certified')
            geometry_sha=diagnostics.get('geometry_sha256')
            if not isinstance(geometry_sha,str)or len(geometry_sha)!=64 or any(ch not in '0123456789abcdef'for ch in geometry_sha):
                raise RuntimeError('GPU proof lacks its canonical registered local-geometry SHA')
            approved={(geometry_sha,_state_key(*pair))for pair,clear in zip(pairs,mask)if clear}
            stats.update(GPU_clear_states=int(mask.sum()),CPU_pair_fallback_states=int((~mask).sum()),service=diagnostics)
            def accepts(checker,q,opening):
                try:
                    if _geometry_version(left)!=left_version or _geometry_version(right)!=right_version:
                        stats['context_misses']+=1;return False
                    context=getattr(checker,'preview_other_configuration',None)
                    if context is None:stats['context_misses']+=1;return False
                    other,other_q,other_width=context
                    if checker is left and other is right:key=_state_key(q,opening,other_q,other_width)
                    elif checker is right and other is left:key=_state_key(other_q,other_width,q,opening)
                    else:stats['context_misses']+=1;return False
                    if(geometry_sha,key)in approved:stats['exact_scope_hits']+=1;return True
                    stats['context_misses']+=1;return False
                except Exception:
                    stats['context_misses']+=1;return False
            if approved:
                old_left=getattr(left,'scoped_gpu_pair_clear_certificate',sentinel);old_right=getattr(right,'scoped_gpu_pair_clear_certificate',sentinel)
                installed=True
                left.scoped_gpu_pair_clear_certificate=accepts;right.scoped_gpu_pair_clear_certificate=accepts
        except Exception as error:
            stats['fallback_reason']=type(error).__name__+': '+str(error)
        from co_moving_own_clearance import co_moving_scope
        with co_moving_scope(c,left,right):
            yield
    finally:
        if installed:
            for checker,previous in[(left,old_left),(right,old_right)]:
                if previous is sentinel:
                    if hasattr(checker,'scoped_gpu_pair_clear_certificate'):delattr(checker,'scoped_gpu_pair_clear_certificate')
                else:checker.scoped_gpu_pair_clear_certificate=previous
        approved.clear()
        stats['wall_s']=time.perf_counter()-started
        c['planning_diagnostics'].append(stats)
