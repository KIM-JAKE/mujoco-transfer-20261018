"""Exact offline proof superset for the unchanged saved RIGHT command field.

Includes all original command rows AND every original _samples row at the
stricter 5mrad future-park resolution. No commands or runtime samples change.
Only exact float64 duplicates are removed from offline collision compilation.
"""
import hashlib
from types import SimpleNamespace
import numpy as np
from aperture_path_check import _samples

PROOF_RESOLUTION_RAD = .005


def rows_and_identity(command_rows):
    raw=np.asarray(command_rows,dtype=np.float64)
    if raw.ndim!=2 or raw.shape[1]!=7 or len(raw)<2 or not np.isfinite(raw).all():
        raise ValueError('Finite original saved command field required')
    unique={}
    for q in raw:
        unique.setdefault(np.ascontiguousarray(q).tobytes(),q.copy())
    checker=SimpleNamespace(resolution_rad=PROOF_RESOLUTION_RAD)
    for _,_,q in _samples(checker,raw):
        unique.setdefault(np.ascontiguousarray(q,dtype=np.float64).tobytes(),q.copy())
    rows=np.asarray(list(unique.values()),dtype=np.float64)
    identity=dict(exact_proof_rows_sha256=hashlib.sha256(rows.tobytes()).hexdigest(),
        exact_proof_rows=len(rows),saved_command_rows=len(raw),
        proof_resolution_rad=PROOF_RESOLUTION_RAD,
        proof_sampling='original_aperture_path_check._samples_float64_plus_unchanged_commands')
    return rows,identity
