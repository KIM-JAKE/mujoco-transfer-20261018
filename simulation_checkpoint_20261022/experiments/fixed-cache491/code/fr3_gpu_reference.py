"""Map calibrated identical FR3 chains to one proposal-model base and TCP.

Only fixed robot-model transforms are used. The caller must independently
validate every returned joint solution with its ORIGINAL arm FK and checks.
"""
from pathlib import Path
import json,hashlib,numpy as np

def model_hash(raw):
    fields=['base','tip','frames','axes','bounds'];data={k:np.round(np.asarray(raw[k],float),12).tolist()for k in fields}
    return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()

def reference_gauge(arm,config):
    raw={k:np.asarray(getattr(arm,k),float)for k in ['base','tip','frames','axes','bounds']};own_hash=model_hash(raw);I=np.eye(4)
    if not config.get('gpu_dual_arm_reference'):return own_hash,I,I,False
    path=Path(config.get('gpu_reference_model_json',str(Path(__file__).resolve().parent/'gpu_planning/reference-right-native-kinematics.json')));reference=json.loads(path.read_text())
    for key in ['frames','axes','bounds']:
        R=np.asarray(reference[key],float)
        if R.shape!=raw[key].shape or not np.isfinite(R).all()or np.max(abs(R-raw[key]))>1e-10:raise ValueError('GPU reference FR3 intrinsic chain or bounds mismatch: '+key)
    for T in [raw['base'],raw['tip'],np.asarray(reference['base']),np.asarray(reference['tip'])]:
        if T.shape!=(4,4)or not np.isfinite(T).all()or not np.allclose(T[3],[0,0,0,1],atol=1e-10,rtol=0)or np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))>1e-5 or np.linalg.det(T[:3,:3])<.99999:raise ValueError('Invalid rigid robot base/TCP reference calibration')
    V=np.asarray(reference['base'])@np.linalg.inv(raw['base']);C=np.linalg.inv(reference['tip'])@raw['tip'];reference_hash=model_hash(reference)
    return reference_hash,V,C,reference_hash!=own_hash
