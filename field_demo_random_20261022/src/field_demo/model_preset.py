"""Author conservative FR3-v2/native-Hand and M80 CAD starter geometry offline.

No device I/O. Uses pinned official collision vertices and actual read-only SDK
F_T_EE snapshots. All generated numeric records remain model_derived,false.
Factory candidates are not calibrated real-robot grasps or mesh-accurate paths.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import numpy as np
from .calibration import rigid
from .model_records import digest,installation,subject_hash

ROOT=Path(__file__).resolve().parents[2]
ASSETS=ROOT/'src/isaac_port/fieldkit/assets/fr3v2-factory'

def template():
    return dict(schema_version=1,kind='actual_installation_match',verified=False,by=None,at_utc=None,evidence_file=None,
        facts=dict(robot_model=None,hand_model=None,factory_mount=None,no_unmodeled_adapters=None,
                   cable_radial_allowance_m=None,part_cad=None,part_load_basis=None),
        expected=dict(robot_model='fr3v2',hand_model='franka_hand',factory_mount='flange_Rz_minus_45deg',
                      part_cad='m80-ring1-ring2-pin',part_load_basis='accepted_nominal_CAD_mass_COM_inertia'),
        note='Fill this one installation match from the actual hardware/CAD. verified applies to the match, never to physically measuring every derived capsule or CAD inertia. Unknown cable/adapters/model/part mismatch rejects the preset.')

def transform_points(points,T):
    return np.asarray(points)@T[:3,:3].T+T[:3,3]

def enclosing_capsule(vertices,name):
    v=np.asarray(vertices,float);center=(v.min(0)+v.max(0))/2
    axis=int(np.argmax(np.ptp(v,axis=0)));a=center.copy();b=center.copy();a[axis]=v[:,axis].min();b[axis]=v[:,axis].max()
    direction=b-a;u=np.clip((v-a)@direction/(direction@direction),0,1)if direction@direction else np.zeros(len(v))
    radius=float(np.max(np.linalg.norm(v-a-u[:,None]*direction,axis=1))+1e-6)
    return dict(name=name,a=a.tolist(),b=b.tolist(),radius_m=radius)

def tool_columns(vertices,T,name,*,y_bins=4,z_bins=8,padding=0.,y_sweep=0.):
    v=np.asarray(vertices);lo=v.min(0);hi=v.max(0);lo[1]-=y_sweep;hi[1]+=y_sweep
    rows=[];ys=np.linspace(lo[1],hi[1],y_bins+1);zs=np.linspace(lo[2],hi[2],z_bins+1)
    for i in range(y_bins):
        for j in range(z_bins):
            a=np.array([lo[0],(ys[i]+ys[i+1])/2,(zs[j]+zs[j+1])/2]);b=a.copy();b[0]=hi[0]
            r=float(np.hypot((ys[i+1]-ys[i])/2,(zs[j+1]-zs[j])/2)+padding+1e-6)
            rows.append(dict(name=f'{name}_y{i}_z{j}',a=transform_points(a[None],T)[0].tolist(),b=transform_points(b[None],T)[0].tolist(),radius_m=r))
    return rows

def sdk_state(path):
    value=json.loads(Path(path).read_text());s=value.get('state',value)
    if value.get('read_only')is not True or value.get('server_protocol_version')!=10:raise ValueError('Actual read-only SDK protocol10 state snapshot required')
    for key in ['q','q_d','dq','dq_d']:
        if np.asarray(s[key]).shape!=(7,)or not np.isfinite(s[key]).all():raise ValueError('Actual stationary SDK state required')
    if np.max(np.abs(np.asarray(s['q'])-s['q_d']))>.005 or max(np.max(np.abs(s['dq'])),np.max(np.abs(s['dq_d'])))>.001:raise ValueError('Actual stopped SDK state required')
    # Positive actual SDK tool data is copied as configured data, never invented Hand mass.
    from .payload_boundary import parameters
    def row(m,c,I):return dict(units='kg,m,kg*m^2',inertia_about='center_of_mass',frame='flange',mass_kg=s[m],center_m=s[c],inertia_column_major_kg_m2=s[I])
    tool=row('m_ee_kg','F_x_Cee_m','I_ee_column_major_kg_m2');empty=row('m_load_kg','F_x_Cload_m','I_load_column_major_kg_m2')
    parameters(tool);parameters(empty)
    if tool['mass_kg']<=0 or empty['mass_kg']!=0:raise ValueError('Actual configured positive Hand/tool and empty object-load SDK state required')
    return rigid(np.asarray(s['F_T_EE_column_major']).reshape(4,4,order='F')),tool,empty

def author(output,installation_path,left_state,right_state,*,workspace_path=None):
    output=Path(output).resolve();installation_path=Path(installation_path).resolve();inst,evidence=installation(installation_path)
    if not all((output/n).is_file()for n in ['random-motion.json','random-cycle.json','payload-profile.json']):raise ValueError('Run init-random first, or pass model inputs to init-random')
    presetpath=output/'model-preset.json'
    if presetpath.exists():raise FileExistsError('Existing derived preset is preserved; use a new site directory')
    motion=json.loads((output/'random-motion.json').read_text());cycle=json.loads((output/'random-cycle.json').read_text());payload=json.loads((output/'payload-profile.json').read_text())
    provenance=json.loads((ASSETS/'provenance.json').read_text())
    for source in provenance['files']:
        if digest(ASSETS/source['packaged_file'])!=source['sha256']:raise ValueError('Pinned official model source changed: '+source['packaged_file'])
    for name,expected in json.loads((ASSETS/'derived-provenance.json').read_text())['outputs'].items():
        if digest(ASSETS/name)!=expected:raise ValueError('Packaged derived CAD geometry changed: '+name)
    proof=json.loads((ASSETS/'enclosure-proof.json').read_text());z=np.load(ASSETS/'collision-models.npz',allow_pickle=False)
    mount=np.eye(4);a=-np.pi/4;mount[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]
    pad=float(inst['facts']['cable_radial_allowance_m']);subjects={};assign=[];tcp_hand={}
    def bind(key,obj):subjects[key]=copy.deepcopy(obj);assign.append((key,obj));return obj
    for side,state_path in [('left',left_state),('right',right_state)]:
        F,tool,empty=sdk_state(state_path);T=np.linalg.inv(F)@mount;tcp_hand[side]=T.copy()
        # Static width-sweep columns enclose EVERY opening0..80mm; runtime cannot assume commanded width was reached.
        tools=tool_columns(z['palm_vertices'],T,'palm',y_bins=4,z_bins=8,padding=pad)
        for sign,label in [(1,'left_finger'),(-1,'right_finger')]:
            v=z['finger_vertices'].copy();v[:,:2]*=sign;v[:,1]+=sign*.020;v[:,2]+=.0584
            tools+=tool_columns(v,T,label,padding=pad,y_sweep=.020)
        link_columns=copy.deepcopy(json.loads((ASSETS/'arm-columns.json').read_text())['capsules'])
        for row in link_columns:row['radius_m']+=pad
        for row in tools:row['geometry_role']='finger'if'finger'in row['name']else'palm'
        radii=(np.asarray(proof['link_radii_m'])+pad).tolist()
        envelope=dict(native_hand_aperture_geometry=dict(T_tcp_hand=T.tolist(),cable_radial_allowance_m=pad,palm_column_count=32),geometry_backend='sdk_link_columns',link_capsules_link_m=link_columns,tool_capsule_offset=len(link_columns),F_T_EE_column_major=F.ravel(order='F').tolist(),link_radii_m=radii,link_endpoint='sdk_flange',tool_in_link_envelopes=False,tool_capsules_tcp_m=tools,
            whole_robot_hand_tool_cables_enclosed=True,self_ignore_pairs=[],payload_robot_ignore_capsules=[len(link_columns)+i for i,row in enumerate(tools)if row['geometry_role']=='finger'],
            model_notes='Conservative pinned official collision-mesh enclosures plus actual cable allowance. Actual SDK per-link FK with tight triangle-clipped column enclosures; all-width Hand sweep can reject feasible paths. Fixed-base support contact needs one actual support-plane record; no moving-link exception.')
        motion['robots'][side]=bind('robot_'+side,envelope)
        payload['robots'][side]['tool']=bind('sdk_configured_tool_'+side,tool);payload['robots'][side]['empty_payload']=bind('sdk_empty_load_'+side,empty)
        cycle['inventory_inspection']['native_hand_mount'][side]=bind('mount_'+side,dict(T_flange_hand=mount.tolist()))
    for part,entry in zip(['ring1','ring2','pin'],cycle['parts']):
        model=proof['parts'][part];caps=[enclosing_capsule(z[f'{part}_sector{i}'],f'{part}_sector{i}')for i in range(model['groups'])]
        v=np.concatenate([z[f'{part}_sector{i}']for i in range(model['groups'])]);radius=float(np.linalg.norm(v[:,:2],axis=1).max());length=float(np.ptp(v[:,2]))
        dim=bind('dimensions_'+part,dict(radius_m=radius,length_m=length))
        env=bind('part_'+part,dict(capsules_part_m=caps,payload_dimensions=copy.deepcopy(dim)))
        motion['loose_part_envelopes'][part]=env;entry['payload_dimensions']=copy.deepcopy(dim);entry['capsules_part_m']=copy.deepcopy(caps)
        # Nominal class-body COM/inertia are explicitly accepted CAD model assumptions, not measured mass properties.
        p=dict(units='kg,m,kg*m^2',inertia_about='center_of_mass',frame='part',mass_kg=model['mass_kg'],center_m=[0.,0.,0.],inertia_column_major_kg_m2=np.diag(model['inertia']).ravel(order='F').tolist())
        payload['parts'][part]=bind('load_'+part,p)
        families=[];family_keys=[]
        for flip in [False,True]:
            for degrees in [0,45,90,135]:
                angle=np.deg2rad(degrees);rz=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
                R=rz@(np.diag([1.,-1.,-1.])if flip else np.eye(3))
                # Nominal physical Hand contact datum from official Hand xacro;
                # actual Desk TCP can have ANY configured rigid offset/yaw.
                P_hand=np.eye(4);P_hand[:3,:3]=R;P_hand[:3,3]=-R@np.array([0.,0.,.1034])
                P_tcp=P_hand@np.linalg.inv(tcp_hand['right'])
                key=f'grasp_{part}_{int(flip)}_{degrees}';family_keys.append(key)
                families.append(bind(key,dict(name=f'native_external_CAD_face{int(flip)}_yaw{degrees}',T_part_tcp=P_tcp.tolist(),
                    pregrasp_offset_part_m=(-R[:,2]*.06).tolist(),physical_Hand_contact_datum_m=[0.,0.,.1034])))
        entry['grasp_families']=families;entry['pickup_side']='right';entry['insert_side']='right'
        entry['pregrasp_offset_part_m']=[0,0,-.06];entry['lift_offsets_cell_m']=[[0,0,.07],[0,0,.10]]
        entry['insert_entry_offset_part_m']=[0,0,-.05];entry['withdraw_offset_cell_m']=[0,0,-.06];entry['min_tcp_clearance_m']=.04
        width={'ring1':.075,'ring2':.062,'pin':.013}[part]
        actions=dict(open=dict(width_m=.08,speed_m_s=.01),grasp=dict(width_m=width,speed_m_s=.01,force_n=10,epsilon_inner_m=.002,epsilon_outer_m=.002),release=dict(width_m=.08,speed_m_s=.01))
        for family,key in zip(families,family_keys):
            family['actions']=copy.deepcopy(actions)
            subjects[key]['actions']=copy.deepcopy(actions)
        entry['candidate_only']=True
    # Finger identity is derived from the fixed native Hand model, not64
    # separate onsite ruler measurements. Inventory/bin/STOP facts remain actual.
    finger_rows=[]
    for i,row in enumerate(tools):
        if row.get('geometry_role')=='finger':
            finger_rows.append(bind('finger_identity_'+str(i),dict(index=len(link_columns)+i,name=row['name'])))
    for phase in motion['phases'].values():
        phase['target_contact_tool_capsules']=[r['index']for r in finger_rows]
        phase['handover_contact_tool_capsules']=[r['index']for r in finger_rows]
    motion['bounded_loose_stock_contact_example']['finger_capsules']=finger_rows
    motion['bounded_loose_stock_departure_example']['contact_policy']['finger_capsules']=finger_rows
    # Common algorithm budgets are explicit conservative commissioning defaults, not claimed physical measurements.
    motion['robot_mounting_supports']={side:dict(obstacle_name=None,normal_cell=None,origin_cell_m=None,no_base_cables_below_support=None,measurement=dict(verified=False,by=None,at_utc=None,evidence_file=None))for side in ['left','right']}
    motion.update(collision_margin_m=.001,max_observation_age_s=2.,max_state_read_s=2.,other_arm_q_tolerance_rad=.001,target_position_tolerance_m=.001,target_angle_tolerance_rad=.01)
    cycle.update(feedback_timeout_s=5.,feedback_dwell_s=.15,max_task_s=300.,max_observation_age_s=2.,held_roi_radius_m=.045,target_position_tolerance_m=.001,target_angle_tolerance_rad=.01,release_width_tolerance_m=.002,insert_pose_confirmation_tolerance_m=.001)
    if workspace_path:
        workspace=json.loads(Path(workspace_path).read_text());from .random_cycle import measured
        measured(workspace['measurement'],Path(workspace_path).resolve().parent)
        from .collision import box
        box(workspace)
        # Rebase evidence; one ACTUAL measured workspace, reused as a broad corridor, never as obstacle omission.
        record=copy.deepcopy(workspace['measurement']);record['evidence_file']=str((Path(workspace_path).resolve().parent/record['evidence_file']).resolve())
        motion['scene']['workspace']={k:copy.deepcopy(workspace[k])for k in ['T_cell_box','half_extents_m']}
        for phase in motion['phases'].values():phase['corridors']=[copy.deepcopy(motion['scene']['workspace'])];phase['tcp_corridor_radius_m']=.001;phase['measurement']=copy.deepcopy(record)
    sourcepaths=[ASSETS/'derived-provenance.json',ASSETS/'arm-columns.json',ASSETS/'collision-models.npz',ASSETS/'enclosure-proof.json',ASSETS/'provenance.json',Path(left_state).resolve(),Path(right_state).resolve()]
    sourcepaths += list(ASSETS.glob('link*.stl'))
    data=dict(schema_version=1,kind='factory_CAD_starter',hardware_tested=False,installation_file=os.path.relpath(installation_path,output),installation_sha256=digest(installation_path),
        source_dependencies=[dict(path=os.path.relpath(p,output),sha256=digest(p))for p in sourcepaths],subjects=subjects,
        limitations=['Conservative all-width tool capsules may reject narrow grasps; no enclosure is shrunk to get a pass.','Class grasp transforms are untested candidate families, not sim-trace replay or proven field skills.','Actual scene/base/camera/bin/jig/STOP/clamp and installation evidence remain required.','Base capsule may overlap the real mounting support and require an exact model-supported interface; support is not bypassed.'])
    presetpath.write_text(json.dumps(data,indent=2)+'\n');ph=digest(presetpath)
    def add_record(key,obj):obj['measurement']=dict(kind='model_derived',verified=False,model_preset_file='model-preset.json',model_preset_sha256=ph,subject_key=key,subject_sha256=subject_hash(subjects[key]))
    for key,obj in assign:add_record(key,obj)
    # Copies created before binding need the SAME content-bound record.
    for part,entry in zip(['ring1','ring2','pin'],cycle['parts']):
        add_record('dimensions_'+part,entry['payload_dimensions']);entry['shape_model_measurement']=copy.deepcopy(motion['loose_part_envelopes'][part]['measurement']);add_record('dimensions_'+part,motion['loose_part_envelopes'][part]['payload_dimensions'])
    for name,value in [('random-motion.json',motion),('random-cycle.json',cycle),('payload-profile.json',payload)]:
        if name=='random-cycle.json':value['payload_profile_sha256']=hashlib.sha256((json.dumps(payload,indent=2)+'\n').encode()).hexdigest()
        (output/name).write_text(json.dumps(value,indent=2)+'\n')
    return dict(state='model_derived_starter_authored',hardware_ready=False,hardware_tested=False,output=str(output),model_preset_sha256=ph,derived_links=16,derived_arm_columns_per_arm=len(link_columns),derived_tool_capsules_per_arm=len(tools),derived_ring_sectors=128,remaining='Actual base/camera/bin/jig/workspace/obstacles/STOP/clamp and checked onsite grasp commissioning; see docs/MODEL_STARTER.md')

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='command',required=True)
    t=s.add_parser('template');t.add_argument('--output',type=Path,required=True)
    a=s.add_parser('author');a.add_argument('--site',type=Path,required=True);a.add_argument('--installation',type=Path,required=True);a.add_argument('--left-state',type=Path,required=True);a.add_argument('--right-state',type=Path,required=True);a.add_argument('--workspace',type=Path)
    args=p.parse_args(argv)
    if args.command=='template':
        if args.output.exists():raise FileExistsError('Existing installation evidence preserved')
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(template(),indent=2)+'\n');result=dict(state='unconfirmed_installation_template',path=str(args.output))
    else:result=author(args.site,args.installation,args.left_state,args.right_state,workspace_path=args.workspace)
    print(json.dumps(result,indent=2));return 0
