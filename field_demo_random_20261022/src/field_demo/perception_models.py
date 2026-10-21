"""Fixed class CAD surfaces only; no object poses or simulator state."""
from functools import lru_cache
from pathlib import Path
import numpy as np
@lru_cache(maxsize=3)
def model_points(part):
 if part not in ('ring1','ring2','pin'):raise ValueError('Known class CAD required')
 root=Path(__file__).resolve().parents[1]/'isaac_port/fieldkit/assets/part-models.npz'
 with np.load(root,allow_pickle=False)as z:points=z[part+'_points'].copy()
 points.setflags(write=False);return points
