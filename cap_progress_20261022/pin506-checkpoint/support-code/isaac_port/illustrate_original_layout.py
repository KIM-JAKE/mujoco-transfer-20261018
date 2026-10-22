"""Static CAD identification figure. This is not a simulated successful assembly."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
s=json.loads((ROOT/'generated/contact_task/contact-source.json').read_text());z=np.load(ROOT/'generated/contact_task/collision.npz');fine=np.load(ROOT/'generated/contact_task/precision-cad.npz');cfg=json.loads((ROOT/'generated/contact_task/fast/config.json').read_text())
fig=plt.figure(figsize=(15,8));ax=fig.add_subplot(121,projection='3d');side=fig.add_subplot(122)
colors={369:'#df7018',380:'#df7018',371:'#7b46ba',384:'#7b46ba',373:'#007f9d',382:'#007f9d'}
for row in s['geoms']:
 identifier=int(row['name'].split('_')[-1])if row['name'].startswith('instance_')else -1
 if identifier not in colors:continue
 v=Rotation.from_quat(np.array(row['quaternion'])[[1,2,3,0]]).apply(z[row['mesh']+'_vertices'])+row['position'];f=z[row['mesh']+'_faces'];poly=Poly3DCollection(v[f],facecolor=colors[identifier],edgecolor=colors[identifier],linewidth=.1,alpha=.85);ax.add_collection3d(poly)
 side.scatter(v[:,1]*1000,v[:,2]*1000,s=.6,color=colors[identifier])
 if identifier in [369,371,373]:
  label={369:'A: 0369 / 0380',371:'B: 0371 / 0384',373:'C: 0373 / 0382'}[identifier];mid=(v.min(0)+v.max(0))/2;ax.text(-.65,mid[1],1.63,label,color=colors[identifier],fontsize=10)
for name,ident,angle in [('ring1',542,0),('ring2',549,60)]:
 p=np.array(next(x['goal_origin_m']for x in cfg['parts']if x['name']==name));v=Rotation.from_euler('z',angle,degrees=True).apply(fine[f'instance_{ident:04d}_vertices'])+p;f=fine[f'instance_{ident:04d}_faces'];ax.add_collection3d(Poly3DCollection(v[f[::12]],facecolor='#666666',edgecolor='none',alpha=.45));side.scatter(v[::30,1]*1000,v[::30,2]*1000,s=1,color='#666666');ax.text(*p,name,color='black',fontsize=9)
side.axhspan(1524,1539,color='gray',alpha=.1,label='Ring target height range')
ax.set(xlim=(-.7,.16),ylim=(.16,.35),zlim=(1.49,1.66),xlabel='World X (m)',ylabel='World Y (m)',zlabel='World Z (m)',title='Highlighted components in ORIGINAL imported CAD')
ax.set_box_aspect((3.5,1,1));ax.view_init(elev=25,azim=-58)
side.set(xlim=(185,330),ylim=(1500,1620),xlabel='World Y (mm)',ylabel='World Z (mm)',title='Side projection: same components and ring targets');side.grid(alpha=.25);side.legend(loc='upper left')
fig.suptitle('The three long component rows previously called "rails"\nA/B/C are identification labels; exact mechanical function is unconfirmed',fontsize=16)
fig.text(.5,.025,'Original imported mesh coordinates. Gray: nominal ring targets. Static illustration, not a physics pass or manufacturing drawing.',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.05,1,.9));out=ROOT/'reports/hardware-audit/long-components-original-cad.png';fig.savefig(out,dpi=160);print(out)
