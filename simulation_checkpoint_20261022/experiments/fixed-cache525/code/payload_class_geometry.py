"""Read fixed class-local CAD without cancelling a free body's world pose."""
import numpy as np

def class_local_transform(cache,prim,root):
    transform,reset=cache.ComputeRelativeTransform(prim,root)
    if reset:raise ValueError('Payload collision template resets its ancestor transform stack')
    result=np.asarray(transform,dtype=float).T.copy()
    if result.shape!=(4,4) or not np.isfinite(result).all():raise ValueError('Invalid class-local CAD transform')
    return result
