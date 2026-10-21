from pathlib import Path
T=Path('/tmp/field-azimuth596')
for mode in ['original','swap']:
 s=(Path('/tmp/field-convex-world-screen')/'original.py').read_text().replace('/tmp/field-convex-world-screen','/tmp/field-azimuth596').replace('pickup_search_max_plans=24','pickup_search_max_plans=32').replace('if len(campaign)>=24','if len(campaign)>=32')
 s=s.replace("pickup_ik_seed_offsets_rad=[[0.]*7,[0.,0.,.8,0.,0.,0.,0.],[0.,0.,-.8,0.,0.,0.,0.]]", "pickup_ik_seed_offsets_rad=[[0.]*7]")
 marker=" entry['grasp_families']=[family]"
 code=''' import types,importlib.util
 cap_path=Path('/tmp/field-azimuth596/native-contact/receiver_pad_contact.py');cap_spec=importlib.util.spec_from_file_location('fixed_native_contact_geometry',cap_path);cap_module=importlib.util.module_from_spec(cap_spec);cap_spec.loader.exec_module(cap_module)
 with np.load(R/'src/isaac_port/fieldkit/assets/part-models.npz',allow_pickle=False)as models_:estimator=types.SimpleNamespace(models={'ring1':models_['ring1_points']},normals={'ring1':models_['ring1_normals']})
 contact_context=dict(config={},observed_grasp_clearance=types.SimpleNamespace(estimator=estimator));opening=float(next(e for e in picked['events']if e['event']=='outer_lifted_fresh_opposed_contact')['contact']['actual_opening_m']);families=[];certificates=[]
 for degrees in range(0,360,45):
  z=np.eye(4);z[:3,:3]=Rotation.from_euler('z',degrees,degrees=True).as_matrix();G_phi=source_G@z
  part_hand=np.linalg.inv(np.linalg.inv(source_tcp_hand)@G_phi)
  SWAP_PLACEHOLDER
  g_for_native_contact=source_tcp_hand@np.linalg.inv(part_hand);certificate=cap_module.contact_state(contact_context,'ring1',g_for_native_contact,opening)
  certificates.append(dict(phi_deg=degrees,native_geometry=certificate,T_tcp_part_class_prior=G_phi.tolist(),translation_preserved=bool(np.array_equal(G_phi[:3,3],source_G[:3,3]))))
  if not certificate['opposed_contact_compatible']:continue
  f=copy.deepcopy(family);f.update(name='SIM348_oblique_class_axis_yaw_'+str(degrees)+'_MODE_PLACEHOLDER',phi_deg=degrees,T_part_tcp=(part_hand@np.linalg.inv(actual_tcp_hand)).tolist(),pregrasp_offset_part_m=(-part_hand[:3,2]*.06).tolist());families.append(f)
 entry['grasp_families']=families
 record['azimuth_geometry_certificates']=certificates;record['bounded_azimuth_family_count']=len(families);record['family_mode']='MODE_PLACEHOLDER';record['only_one_actual_q_seed']=True
'''
 if mode=='swap':code=code.replace('SWAP_PLACEHOLDER',"flip=np.eye(4);flip[:2,:2]=[[-1.,0.],[0.,-1.]];part_hand=part_hand@flip")
 else:code=code.replace('  SWAP_PLACEHOLDER\n','')
 code=code.replace('MODE_PLACEHOLDER',mode)
 assert marker in s;s=s.replace(marker,code)
 s=s.replace("row=dict(instance_id=observed['selected_instance']['instance_id'],part=part,", "phi=next((f['phi_deg']for f in entry['grasp_families']if np.allclose(np.asarray(observed['T_cell_part'])@np.asarray(f['T_part_tcp']),goal,atol=1e-8)),None)\n   row=dict(phi_deg=phi,instance_id=observed['selected_instance']['instance_id'],part=part,")
 s=s.replace("Path('/tmp/field-azimuth596/original-'+os.environ['DIAG_TRACKING_RAD']+'.json')", "Path('/tmp/field-azimuth596/"+mode+"-'+os.environ['DIAG_TRACKING_RAD']+'.json')")
 (T/(mode+'.py')).write_text(s)
