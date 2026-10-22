from evaluate_layouts import *
record=json.load(open(HERE/'comparison-physical-band.json'));layout=next(x for x in record['layouts']if x['label']=='near_front');fine=np.load(PORT/'generated/contact_task/precision-cad.npz');recording=np.load(PORT/'runs/contact-trim-03/recording/trajectory.npz');installed={}
for name,i,k in [('ring1',542,0),('ring2',549,1)]:
 key=f'instance_{i:04d}';P=recording['mocap_pos'][-1,k];R=Rotation.from_quat(recording['mocap_quat'][-1,k][[1,2,3,0]]).as_matrix();installed[name]=fcl.CollisionObject(g.mesh_shape(fine[key+'_vertices'],fine[key+'_faces']),fcl.Transform(R,P))
result=[]
for part,r in layout['recipes'].items():
 q=np.array(r['q_path'][-1]);hits=[]
 for name,o in objects(r['side'],q,r['gap'])[0]:
  for other,p in installed.items():
   if other!=part and pairhit(o,p):hits.append([name,'installed_'+other])
 result.append(dict(part=part,hits=hits));print('INSTALLED_PARTS',part,hits,flush=True)
(HERE/'installed-part-endpoints.json').write_text(json.dumps(result,indent=2))
