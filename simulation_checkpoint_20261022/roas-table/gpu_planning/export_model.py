"""Export exact production frame chain, not a Panda model; no robot commands."""
import hashlib,json,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from service_paths import ROOT,SOURCE,URDF
raw=json.loads(SOURCE.read_text())
tree=ET.Element('robot',name='production_native_fr3_right')
ET.SubElement(tree,'link',name='world')
def addjoint(name,parent,child,T,kind='fixed',axis=None,bounds=None):
    ET.SubElement(tree,'link',name=child);j=ET.SubElement(tree,'joint',name=name,type=kind);ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=child);ET.SubElement(j,'origin',xyz=' '.join(map(str,T[:3,3])),rpy=' '.join(map(str,Rotation.from_matrix(T[:3,:3]).as_euler('xyz'))))
    if axis is not None:ET.SubElement(j,'axis',xyz=' '.join(map(str,axis)));ET.SubElement(j,'limit',lower=str(bounds[0]),upper=str(bounds[1]),velocity='2.0',effort='87')
addjoint('mount','world','body0',np.asarray(raw['base']))
for i,(a,b)in enumerate(raw['frames']):
    addjoint(f'joint{i+1}',f'body{i}',f'rotor{i+1}',np.asarray(a),'revolute',raw['axes'][i],raw['bounds'][i]);addjoint(f'frame{i+1}',f'rotor{i+1}',f'body{i+1}',np.asarray(b))
addjoint('native_tcp','body7','tcp',np.asarray(raw['tip']));urdf=URDF;ET.indent(tree);ET.ElementTree(tree).write(urdf,encoding='unicode')
canonical={k:np.round(np.asarray(raw[k],float),12).tolist() for k in ['base','tip','frames','axes','bounds']}
manifest=dict(model_sha256=hashlib.sha256(json.dumps(canonical,sort_keys=True,separators=(',',':')).encode()).hexdigest(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),proposal_only=True,actuator_limits_not_defined_by_URDF=True)
(ROOT/'robot-model-manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(manifest,indent=2))
