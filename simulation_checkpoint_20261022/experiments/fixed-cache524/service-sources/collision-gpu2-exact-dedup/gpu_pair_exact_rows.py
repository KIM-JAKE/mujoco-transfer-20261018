"""Deduplicate exact original transform-index rows within ONE GPU request.

Banks, registered geometry and the conservative pair kernel are unchanged.
Equal index rows identify byte-identical input transforms; no numeric value is
rounded. The inverse restores every original output row and its original order.
"""
from dataclasses import dataclass
import time
import numpy as np

@dataclass(frozen=True)
class ExactRows:
    left: np.ndarray
    right: np.ndarray
    first: np.ndarray
    inverse: np.ndarray
    original_rows: int
    deduplicate_s: float

    def expand(self, values):
        # Do not coerce unknown numbers, objects or malformed kernel replies
        # into authority. A failure propagates to the service's original
        # fallback_required response; no partial bitmask may be returned.
        if not isinstance(values,list) or len(values)!=len(self.first) or any(type(v)is not bool for v in values):
            raise ValueError('Exact dedup requires one original boolean kernel result per unique row')
        return np.asarray(values,dtype=np.bool_)[self.inverse].tolist()

    def diagnostics(self):
        return dict(exact_input_rows=self.original_rows,exact_unique_rows=len(self.first),
                    exact_duplicate_rows=self.original_rows-len(self.first),
                    exact_row_deduplication_s=self.deduplicate_s,
                    exact_index_rows_only=True,original_kernel_unchanged=True)


def deduplicate(left_index,right_index):
    """Return exact first-occurrence input rows and a complete inverse map.

    The caller still validates both original banks/index bounds/geometry and
    request policy BEFORE calling this helper. No result cache crosses requests.
    """
    began=time.perf_counter()
    if any(not isinstance(a,np.ndarray) or a.dtype!=np.int32 or a.ndim!=2 for a in (left_index,right_index)):
        raise ValueError('Exact int32 matrix indices required')
    count=len(left_index)
    if not 0<count<=16384 or len(right_index)!=count or not left_index.shape[1] or not right_index.shape[1]:
        raise ValueError('Equal bounded nonempty original row counts required')
    joined=np.concatenate((left_index,right_index),axis=1)
    # Fixed-width void compares literal int32 bytes. Original validated index
    # values are nonnegative; byte sorting is only an implementation detail.
    key=joined.view(np.dtype((np.void,joined.shape[1]*joined.dtype.itemsize))).reshape(-1)
    _,first,inverse=np.unique(key,return_index=True,return_inverse=True)
    order=np.argsort(first,kind='stable');first=first[order]
    remap=np.empty(len(order),dtype=np.intp);remap[order]=np.arange(len(order),dtype=np.intp)
    inverse=remap[inverse]
    # Retain original first-occurrence order, reducing irrelevant regrouping.
    return ExactRows(left_index[first],right_index[first],first,inverse,count,time.perf_counter()-began)
