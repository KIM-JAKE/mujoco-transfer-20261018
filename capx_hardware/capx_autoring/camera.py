"""Original persistent RealSense I/O with measured profile binding at every read."""
import numpy as np
from .field_demo.camera import RealSenseCapture

class MeasuredCamera(RealSenseCapture):
 def __init__(self,config,hardware):
  self.expected=hardware
  super().__init__(config,width=hardware['width'],height=hardware['height'],fps=hardware['fps'],max_age_s=hardware['max_age_s'])
 def read(self):
  packet,metadata=super().read();K=np.asarray(packet['K'],float);want=np.asarray(self.expected['K'],float);tol=self.expected['intrinsic_tolerance_px']
  if want.shape!=(3,3)or K.shape!=(3,3)or not np.isfinite(K).all()or type(tol)not in (int,float)or not 0<=tol<=1e-6 or np.max(abs(K-want))>tol:raise RuntimeError('Actual camera intrinsics do not match measured profile')
  shape=(self.expected['height'],self.expected['width'])
  if np.asarray(packet['depth_m']).shape!=shape or np.asarray(packet['rgb']).shape!=(*shape,3):raise RuntimeError('Actual RGB/depth size differs from measured profile')
  return packet,metadata
