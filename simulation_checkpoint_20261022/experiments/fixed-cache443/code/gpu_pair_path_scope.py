"""Batch the dynamic pair part of one original single-arm path preflight."""
from contextlib import contextmanager
import numpy as np
from aperture_path_check import _samples
from gpu_pair_schedule_scope import certified_pair_scope


@contextmanager
def certified_path_scope(c,checker,qs,openings):
    """Current opposite encoders/CAD stay frozen throughout this context.

    The caller still invokes original full path_openings/Checker.path. GPU
    receipts replace only their original pair component at exact q/apertures.
    A changed LEFT endpoint needs one fresh batch, with no old park replay.
    """
    if not c['config'].get('gpu_pair_schedule_certificate',False):
        yield
        return
    context=getattr(checker,'preview_other_configuration',None)
    if context is None:
        yield
        return
    other,other_q,other_width=context
    left=c['checkers']['left'];right=c['checkers']['right']
    if not((checker is left and other is right)or(checker is right and other is left)):
        yield
        return
    samples=list(_samples(checker,np.asarray(qs)))
    pairs=[]
    for opening in np.asarray(openings,float):
        for _,_,q in samples:
            if checker is left:pairs.append((q,float(opening),other_q,float(other_width)))
            else:pairs.append((other_q,float(other_width),q,float(opening)))
    with certified_pair_scope(c,left,right,pairs=pairs):
        yield
