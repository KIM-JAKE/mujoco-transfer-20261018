"""GPU CLEAR proposals for the complete original serial current-context grid.

All own/self/fixed/placed tests and original interpolation remain in the
caller's original path_other_contexts. Uncertain bits use original CPU pair
checks. No certificate survives a physical step or this immutable scope.
"""
from contextlib import contextmanager
import numpy as np
from aperture_path_check import _samples
from gpu_pair_schedule_scope import certified_pair_scope

@contextmanager
def serial_path_scope(c,checker,other,contexts,qs,widths):
    if not c['config'].get('gpu_pair_schedule_certificate',False):
        yield
        return
    qs=np.asarray(qs,float)
    # Match path_other_contexts EXACTLY: it makes float64 copies of contexts.
    contexts=[(np.asarray(q,float).copy(),float(w))for q,w in contexts]
    samples=[q for _,_,q in _samples(checker,qs)]
    left,right=c['checkers']['left'],c['checkers']['right']
    if checker is right and other is left:
        pairs=[(oq,ow,q,float(w))for q in samples for w in widths for oq,ow in contexts]
    elif checker is left and other is right:
        pairs=[(q,float(w),oq,ow)for q in samples for w in widths for oq,ow in contexts]
    else:
        yield
        return
    with certified_pair_scope(c,left,right,pairs=pairs):
        yield
