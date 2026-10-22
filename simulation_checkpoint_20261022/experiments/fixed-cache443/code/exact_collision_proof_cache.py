"""Persist exact CLEAR certificates; keys never quantize candidate geometry.

The caller supplies an immutable, complete model/context fingerprint once per
planning scope. Changed/missing context, unknown state and database errors all
fall back to the original checker. Only successful original checks are stored.
"""
from contextlib import contextmanager
import hashlib
import inspect
import importlib.metadata
import sys
from pathlib import Path
import sqlite3
import struct
import numpy as np

_VERSION = b'original-own-proof-v1'

def _array(h,value):
    a=np.ascontiguousarray(value)
    h.update(a.dtype.str.encode());h.update(repr(a.shape).encode());h.update(a.tobytes())

def own_model_fingerprint(guard):
    h=hashlib.sha256(_VERSION)
    h.update(sys.version.encode())
    for package in ('numpy','scipy','python-fcl'):
        try:version=importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:version='unavailable'
        h.update((package+'='+version).encode())
    for name in ('same_arm_clearance.py','own_payload_clearance.py','predictive_pair_clearance.py','isaac_kinematics.py','numeric_fk.py','isaac_collision_queries.py','fixed_static_route_certificate.py'):
        path=Path(__file__).with_name(name)
        if path.exists():h.update(name.encode());h.update(path.read_bytes())
    arm=guard.arm
    for function in (type(guard).clear,type(arm).bodies):
        try:h.update(inspect.getsource(function).encode())
        except (OSError,TypeError):raise ValueError('Active native checker/FK source is unavailable for persistent proof identity')
    for value in (arm.base,arm.frames,arm.axes,arm.tip,arm.bounds):_array(h,value)
    h.update(repr((arm.link_paths,arm.hand_path,guard.base_path,guard.pairs,guard.margin_m,guard.geometries is not None)).encode())
    for body,source,shape in guard.rows:
        h.update(body.encode());h.update(source.encode())
        for value in shape:_array(h,value)
    return h.hexdigest()

def state_key(q,opening):
    # The original same-arm checker itself casts q and opening to float64.
    a=np.ascontiguousarray(q,dtype=np.float64)
    width=float(opening)
    if a.shape!=(7,) or not np.isfinite(a).all() or not np.isfinite(width):return None
    return hashlib.sha256(a.tobytes()+struct.pack('<d',width)).digest()

class ExactClearCache:
    def __init__(self,path):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        self.db=sqlite3.connect(str(self.path),timeout=.05)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('CREATE TABLE IF NOT EXISTS clear_v1 (model TEXT NOT NULL, state BLOB NOT NULL, PRIMARY KEY(model,state)) WITHOUT ROWID')
        self.memory={};self.pending=0
        self.stats=dict(hits=0,misses=0,recorded=0,errors=0)
    def contains(self,model,q,width):
        key=state_key(q,width)
        if key is None:return False
        full=(model,key)
        if full in self.memory:self.stats['hits']+=1;return True
        try:
            if self.db.execute('SELECT 1 FROM clear_v1 WHERE model=? AND state=?',full).fetchone():
                self.memory[full]=True;self.stats['hits']+=1;return True
        except sqlite3.Error:self.stats['errors']+=1
        self.stats['misses']+=1;return False
    def record(self,model,q,width):
        key=state_key(q,width)
        if key is None:return
        full=(model,key)
        try:
            self.db.execute('INSERT OR IGNORE INTO clear_v1(model,state) VALUES (?,?)',full)
            self.memory[full]=True;self.pending+=1;self.stats['recorded']+=1
            if self.pending>=256:self.flush()
        except sqlite3.Error:self.stats['errors']+=1
    def flush(self):
        try:self.db.commit();self.pending=0
        except sqlite3.Error:self.stats['errors']+=1
    def close(self):self.flush();self.db.close()

@contextmanager
def native_self_scope(guard,cache):
    """One immutable planning call; no actuation, model edits or calibration.

    Hash all native CAD, FK/calibration, exclusions and original checker sources
    at scope entry. Payload G, completed objects and opposite arm are NOT cached.
    A new scope after any scene/model change recomputes the full fingerprint.
    """
    old=getattr(guard,'native_self_proof_cache',None)
    guard.native_self_proof_cache=(cache,own_model_fingerprint(guard))
    try:yield
    finally:guard.native_self_proof_cache=old;cache.flush()
