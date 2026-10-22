"""Extract the existing supply basket CAD; never invent a replacement basket.

The small three-hole pin block is a proposed printable part, not a CAD asset
claimed to exist. Coordinates remain in the original M80 scene frame.
"""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
PORT=HERE.parents[1]
OUT=HERE/'bin-picking';OUT.mkdir(exist_ok=True)
source_path=PORT/'generated/contact_task/contact-source.json'
source=json.load(open(source_path))
archive=np.load(PORT/'generated/contact_task/collision.npz')
manifest=json.load(open(PORT.parent/'keti_m80_mujoco_20261013/insertion_20261013/cad/cache_manifest.json'))
names={r['index']:r['name']for r in manifest['parts']}
report=dict(source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),units='m',baskets=[],pin_holder=dict(status='proposed_three_hole_block_from_user_photo',additional_standalone_table=False,size_m=[.120,.035,.018],hole_diameter_m=.0054,hole_pitch_m=.035,hole_count=3,insertion_depth_m=.009,top_height_m=1.000,position_world_m=[.230,-.790,.991],dimensions_not_field_verified=True),hardware_tested=False,full_task_pass=False)
arrays={}
for index in [670,671,672,673,674,675]:
    key=f'instance_{index:04d}';row=next(r for r in source['geoms']if r['name']==key)
    q=row['quaternion'];R=Rotation.from_quat(q[1:]+q[:1]).as_matrix();P=np.array(row['position'])
    # CAD local +X is the basket opening direction. The broad inner bottom
    # plane is local X=-43.505 mm, measured from the actual triangle faces.
    B=np.column_stack([-R[:,1],-R[:,2],R[:,0]])
    O=P+R@np.array([-.043505,0,0])
    v=archive[row['mesh']+'_vertices'];f=archive[row['mesh']+'_faces']
    local=(v@R.T+P-O)@B
    arrays[key+'_v']=local;arrays[key+'_f']=f
    world=v@R.T+P
    record=dict(id=key,cad_name=names[index],original_row=row,floor_origin_world_m=O.tolist(),basket_to_world_R=B.tolist(),outer_bounds_world_m=[world.min(0).tolist(),world.max(0).tolist()],outer_dimensions_in_basket_frame_m=np.ptp(local,axis=0).tolist(),inner_bottom_plane_local_cad_x_m=-.043505,nominal_product_dimensions_m=[.380,.240,.105])
    report['baskets'].append(record)
np.savez_compressed(OUT/'existing-baskets.npz',**arrays)
(OUT/'asset-audit.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
