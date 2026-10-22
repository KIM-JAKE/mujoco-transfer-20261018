"""Optional CPU implementation of the same USD-frame damped-least-squares IK."""
from pathlib import Path
import ctypes,hashlib,numpy as np
HERE=Path(__file__).resolve().parent;LIB=HERE/'bin-picking/fastik/libm80_numeric_ik.so'
class NumericIK:
    def __init__(self,arm):
        self.lib=ctypes.CDLL(str(LIB));ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS');self.lib.m80_ik_create.argtypes=[ptr]*5;self.lib.m80_ik_create.restype=ctypes.c_void_p;self.lib.m80_ik_solve.argtypes=[ctypes.c_void_p,ptr,ptr,ptr,ptr,ctypes.c_double,ctypes.c_double,ctypes.c_int];self.lib.m80_ik_solve.restype=ctypes.c_int;self.lib.m80_ik_destroy.argtypes=[ctypes.c_void_p];self.model=self.lib.m80_ik_create(*[np.ascontiguousarray(x,dtype=np.float64)for x in [arm.base,arm.tip,arm.frames,arm.axes,arm.bounds]])
    def solve(self,P,R,seed,position_tolerance=1e-5,rotation_tolerance=1e-4):
        result=np.empty(7,np.float64);args=[np.ascontiguousarray(x,dtype=np.float64)for x in [P,R,seed]]
        if [a.shape for a in args]!=[(3,),(3,3),(7,)] or not all(np.isfinite(a).all() for a in args):return None
        ok=self.lib.m80_ik_solve(self.model,*args,result,position_tolerance,rotation_tolerance,220);return result if ok else None
    def __del__(self):
        if getattr(self,'model',None):self.lib.m80_ik_destroy(self.model);self.model=None

def enable(Arm):
    if not LIB.exists():return dict(enabled=False,reason='Optional numeric IK library missing; Python fallback')
    if not getattr(Arm,'_m80_numeric_ik_enabled',False):
        Arm._python_solve=Arm.solve
        def solve(self,P,R,seed,position_tolerance=1e-5,rotation_tolerance=1e-4):
            if not hasattr(self,'_m80_numeric_ik'):self._m80_numeric_ik=NumericIK(self)
            return self._m80_numeric_ik.solve(P,R,seed,position_tolerance,rotation_tolerance)
        Arm.solve=solve;Arm._m80_numeric_ik_enabled=True
    return dict(enabled=True,device='CPU',algorithm='Same USD frames, joint bounds, DLS damping and 220-iteration limit; Eigen C++ implementation',library_sha256=hashlib.sha256(LIB.read_bytes()).hexdigest(),source_sha256=hashlib.sha256((LIB.parent/'numeric_ik.cpp').read_bytes()).hexdigest())
