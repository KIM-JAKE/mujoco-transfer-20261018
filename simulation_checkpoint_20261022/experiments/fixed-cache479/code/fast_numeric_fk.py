"""Optional numeric USD FK acceleration; original geometry guards unchanged."""
from pathlib import Path
import ctypes,hashlib,numpy as np
ROOT=Path('/workspace/project/mujoco/.local/fastfk368')
if not ROOT.exists():ROOT=Path('/home/gpuadmin/workspace/jake/mujoco/.local/fastfk368')
LIB=ROOT/'libm80_numeric_fk.so'
class NumericFK:
    def __init__(self,arm):
        self.values=[np.ascontiguousarray(v,dtype=np.float64) for v in (arm.base,arm.tip,arm.frames,arm.axes)]
        self.key=tuple(v.tobytes()for v in self.values)
        self.lib=ctypes.CDLL(str(LIB));p=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
        self.lib.fk_create.argtypes=[p]*4;self.lib.fk_create.restype=ctypes.c_void_p
        self.lib.fk_destroy.argtypes=[ctypes.c_void_p]
        self.lib.fk_tcp.argtypes=[ctypes.c_void_p,p,p]
        self.lib.fk_bodies.argtypes=[ctypes.c_void_p,p,ctypes.c_double,p]
        self.handle=self.lib.fk_create(*self.values)
    def unchanged(self,arm):
        return self.key==tuple(np.asarray(v,dtype=np.float64).tobytes()for v in (arm.base,arm.tip,arm.frames,arm.axes))
    def __del__(self):
        if getattr(self,'handle',None):self.lib.fk_destroy(self.handle);self.handle=None
def enable(Arm):
    if not LIB.exists():return dict(enabled=False,reason='Optional numeric FK library missing; original Python fallback')
    if not getattr(Arm,'_m80_numeric_fk_enabled',False):
        original_fk,original_bodies=Arm.fk,Arm.bodies
        Arm._python_fk,Arm._python_bodies=original_fk,original_bodies
        def kernel(arm):
            v=getattr(arm,'_m80_numeric_fk',None)
            if v is None or not v.unchanged(arm):v=NumericFK(arm);arm._m80_numeric_fk=v
            return v
        def fk(arm,q):
            q=np.ascontiguousarray(q,dtype=np.float64)
            if q.shape!=(7,)or not np.isfinite(q).all():return original_fk(arm,q)
            out=np.empty((4,4),np.float64);v=kernel(arm);v.lib.fk_tcp(v.handle,q,out);return out
        def bodies(arm,q,opening):
            q=np.ascontiguousarray(q,dtype=np.float64)
            if q.shape!=(7,)or not np.isfinite(q).all()or not np.isfinite(opening):return original_bodies(arm,q,opening)
            out=np.empty((10,4,4),np.float64);v=kernel(arm);v.lib.fk_bodies(v.handle,q,float(opening),out)
            side=arm.hand_path.split('/')[-1].split('_')[0]
            paths=list(arm.link_paths)+[arm.hand_path]+[arm.hand_path+'/'+side+'_'+label+'_finger'for label in ['left','right']]
            return dict(zip(paths,out))
        Arm.fk,Arm.bodies=fk,bodies;Arm._m80_numeric_fk_enabled=True
    return dict(enabled=True,device='CPU',algorithm='Same live USD frames/Rodrigues/native finger transforms; Eigen numeric FK',library_sha256=hashlib.sha256(LIB.read_bytes()).hexdigest(),source_sha256=hashlib.sha256((ROOT/'numeric_fk.cpp').read_bytes()).hexdigest(),model_mutations_invalidate=True)
