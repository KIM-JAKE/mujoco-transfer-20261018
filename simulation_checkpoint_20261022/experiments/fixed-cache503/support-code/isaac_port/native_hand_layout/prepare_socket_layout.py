"""Adapt previously screened paths to bottom-bearing, finger-window sockets.

Every changed native path is screened anew. Full carried-CAD and coordinated
motion validation remain separate, explicit stages.
"""
from audit_five_geometry import *
from five_cycle_storage import socket_pin_specs,part_pose


def prepare(height):
 source=HERE/f'five-layout-right_high_h{height}_p35_yaw-45-payload.json'
 row=json.load(open(source));row['label']=row['label']+'_socket';row['nominal_payload_paths_pass']=False;row['individual_paths_complete']=False;row['failures']=[]
 r=row['pin_slots'][0]['recipe'];R=np.asarray(r['R']);row_axis=R[:,1].copy();row_axis[2]=0;row_axis/=np.linalg.norm(row_axis);P,_=part_pose(r);center=P+row_axis*.07
 specs,centers,meta=socket_pin_specs(center,R,.035);row['geometry']=[s for s in row['geometry']if not s['name'].startswith('pin_')]+specs;row['storage_metadata']['pin']=meta;row['label_centers']['pin']=center.tolist();row['render_grasp']=True
 row['size_caption']='상판 500 × 350mm · 바닥 지지 포켓 5칸 · 핀 하단 6mm 삽입'
 pieces=table(*row['table_center_m'])+pieces_from_specs([s for s in row['geometry']if not s.get('render_only')]);hits=machine_hits(pieces)
 if hits:row['failures'].append(dict(stage='fixture_machine_intersection',hits=hits))
 for i,(name,a)in enumerate(pieces):
  if name.startswith('independent_table'):continue
  for other,b in pieces[i+1:]:
   if other.startswith('independent_table')or other.split('_')[0]==name.split('_')[0]:continue
   if pairhit(a,b):row['failures'].append(dict(stage='cross_fixture_intersection',pair=[name,other]))
 set_env(pieces)
 for slot in row['pin_slots']:
  r=slot['recipe'];R=np.asarray(r['R']);tcp=np.asarray(r['tcp']);pick=np.asarray(r['q_pick']);axis=R[:,2].copy();axis[2]=0;axis=-axis/np.linalg.norm(axis)
  pre=robust('left',tcp+axis*.04,R,pick);raised=robust('left',tcp+[0,0,.010],R,pick)
  ext=robust('left',tcp+axis*.04+[0,0,.010],R,raised)if raised is not None else None
  if pre is None or raised is None or ext is None:
   row['failures'].append(dict(slot=slot['slot'],stage='socket_IK'));continue
  spans=[s for s in r['segments']if s['label']=='empty_approach'];oldpre=np.asarray(spans[-1]['b']);bridge=route('left',oldpre,pre,.02,allow_search=False)
  # A front approach can require another elbow branch. The original below
  # approach is also physically checked in the socket contact benchmark.
  if bridge is None:
   pre=oldpre;bridge=[oldpre]
  modified=[]
  for s in r['segments']:
   label=s['label']
   if label=='clip_open':continue
   if label=='pregrasp':
    modified.extend(seg(a,b,'empty_approach','pin',.02)for a,b in zip(bridge[:-1],bridge[1:]));modified.append(seg(pre,pick,'pregrasp','pin',.02))
   elif label=='clear_datum':modified.append(seg(pick,raised,label,'pin',r['gap']))
   elif label=='extract':modified.append(seg(raised,ext,label,'pin',r['gap']))
   elif label=='lift':modified.append(seg(ext,np.asarray(s['b']),label,'pin',r['gap']))
   else:modified.append(s)
  r.update(segments=modified,pregrasp_delta_m=(fk('left',pre)[0]-tcp).tolist(),extraction_distance_m=.04,storage_pre_lift_m=.010,duration_s=sum(s['duration_s']for s in modified))
 row['recipes']['pin']=copy.deepcopy(row['pin_slots'][0]['recipe'])
 out=source.with_name(source.stem.replace('-payload','')+'-socket.json')
 # Check every native motion including the new socket extraction. Intended
 # workpiece contacts are covered in the separate contact benchmark.
 for r in [row['recipes']['ring1'],row['recipes']['ring2']]+[s['recipe']for s in row['pin_slots']]:
  for index,s in enumerate(r['segments']):
   count=max(21,int(np.ceil(np.max(abs(np.asarray(s['b'])-s['a']))/.02))+1)
   ok,why=path_clear(r['side'],[np.asarray(s['a']),np.asarray(s['b'])],s['gap_m'],count)
   if not ok:row['failures'].append(dict(part=r['part'],tcp=r['tcp'],segment=index,label=s['label'],why=why))
  print('SOCKET_NATIVE_CHECKED',height,r['part'],r['tcp'],len(row['failures']),flush=True)
 row['individual_paths_complete']=not row['failures'];out.write_text(json.dumps(row,indent=2));print('SOCKET_PREPARED',out,row['individual_paths_complete'],row['failures'],flush=True)
 return out


if __name__=='__main__':
 for argument in sys.argv[1:]:prepare(int(argument))
