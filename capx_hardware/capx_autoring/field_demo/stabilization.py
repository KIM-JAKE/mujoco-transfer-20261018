"""Import completed stabilization evidence without modifying or executing it.

Source code is copied as reference data. A changed detector is never silently
loaded into a live robot process; compatibility is reported for a tested port.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

ROOT = Path(__file__).resolve().parents[2]
NAMES = ('rgbd_scattered_rings.py', 'rgbd_geometry.py', 'rgbd_initial_fast.py',
         'rgbd_fast_tracker.py', 'loose_ring_pick.py', 'loose_bin_task.py',
         'direct_pin_skill.py', 'grasp_transfer_feasibility.py', 'paired_handover.py',
         'fr3_path_timing.py', 'isaac_kinematics.py', 'fcl_transport.py', 'run_realtime_recipe.py',
         'path_local_timing.py','process_vision.py','realtime_camera_worker.py','rgbd_ring_translation.py')
DETECTOR = ('rgbd_scattered_rings.py', 'rgbd_geometry.py')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def inspect_run(source):
    source = Path(source).resolve()
    report_path, manifest_path = source / 'pipeline-results.json', source / 'code-manifest.json'
    report, manifest = json.loads(report_path.read_text()), json.loads(manifest_path.read_text())
    if report.get('full_task_pass') is not True:
        raise ValueError('Only a completed full-task success can be selected as stabilization evidence')
    if report.get('hardware_tested') is not False:
        raise ValueError('Expected explicitly simulated source evidence, not an ambiguous hardware result')
    code = source / 'code_snapshot'
    buffers, hashes = {}, {}
    for name in NAMES:
        p = code / name
        if not p.exists():
            if name in DETECTOR:
                raise ValueError('Missing detector snapshot: '+name)
            continue
        if not p.resolve().is_relative_to(code.resolve()) or p.is_symlink():
            raise ValueError('Source snapshot must contain local immutable files: '+name)
        data = p.read_bytes()
        if name not in manifest or sha(data) != manifest[name]:
            raise ValueError('Source snapshot does not match its code manifest: '+name)
        buffers[name], hashes[name] = data, sha(data)
    current = json.loads((ROOT / 'stabilization/v119/IMPORT.json').read_text())['copied_verified_source']
    summary = dict(source_run=source.name, source_directory=str(source), full_task_pass=True,
                   hardware_tested=False, original_modified=False,
                   wall_verified_elapsed_s=report.get('wall_verified_elapsed_s'),
                   final_checks=report.get('final_checks'),
                   source_result_sha256=sha(report_path.read_bytes()),
                   source_manifest_sha256=sha(manifest_path.read_bytes()), source_hashes=hashes,
                   packaged_detector_matches=all(hashes.get(n) == current[n] for n in DETECTOR),
                   source_code_executed=False, simulator_joint_trace_used_for_robot_motion=False,
                   changed_reference_files=[n for n,h in hashes.items() if current.get(n) != h],
                   note='New source is reference data. Different detector code requires a reviewed port and regression tests; no live activation.')
    return summary, buffers


def import_run(source, output):
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError('Use a new import directory; existing snapshots are preserved')
    summary, buffers = inspect_run(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='.'+output.name+'-', dir=output.parent))
    try:
        for name, data in buffers.items():
            (temporary/name).write_bytes(data)
        summary['copied_verified_source']=dict(summary['source_hashes'])
        records={}
        source_path=Path(source).resolve()
        for name in ('pipeline-results.json','code-manifest.json','trial-config.json','rgb-observations.json','fixed-receiver-catalog.json','robot-kinematics.json'):
            record=source_path/name
            if record.is_file() and not record.is_symlink():
                data=record.read_bytes();(temporary/name).write_bytes(data);records[name]=sha(data)
        for name in ('observation-001-initial','observation-008-initial'):
            record=source_path/'rgb-perception'/name/'rgbd.npz'
            if record.is_file() and not record.is_symlink():
                target=temporary/'replay-frames'/(name+'.npz');target.parent.mkdir(exist_ok=True)
                data=record.read_bytes();target.write_bytes(data);records[str(target.relative_to(temporary))]=sha(data)
        summary['copied_verified_records']=records
        summary['imported_at_utc'] = datetime.now(timezone.utc).isoformat()
        (temporary/'IMPORT.json').write_text(json.dumps(summary, indent=2)+'\n')
        temporary.rename(output)
    except BaseException:
        shutil.rmtree(temporary)
        raise
    return summary



PROFILE_ID='v128-stopped-payload-experimental'
ENGINE_FILES=('src/field_demo/pin_pickup.py','src/isaac_port/fieldkit/assets/native-hand/hand.stl','src/field_demo/convex_self.py','src/isaac_port/fieldkit/assets/convex-links/model.json','src/isaac_port/fieldkit/assets/convex-links/hulls.npz','src/field_demo/common_duo_mount.py','src/isaac_port/fieldkit/assets/common-duo-mount/LICENSE','src/isaac_port/fieldkit/assets/common-duo-mount/fr3_duo.xacro','src/isaac_port/fieldkit/assets/common-duo-mount/fr3_duo_mount.xacro','src/isaac_port/fieldkit/assets/common-duo-mount/kinematics.yaml','src/isaac_port/fieldkit/assets/common-duo-mount/model.json','src/isaac_port/fieldkit/assets/common-duo-mount/provenance.json','src/field_demo/pin_cap.py','src/field_demo/pin_cap_geometry.py','src/field_demo/pin_precision.py','src/isaac_port/fieldkit/assets/part-models.npz','src/field_demo/low_speed_profile.py','src/field_demo/ik_branches.py','src/field_demo/static_scene_lease.py','src/field_demo/observations.py','src/isaac_port/fieldkit/assets/native-hand/collision-finger.npz','src/field_demo/native_hand_solid.py','src/isaac_port/fieldkit/assets/native-hand/hand-visual.dae','src/isaac_port/fieldkit/assets/native-hand/finger.dae','src/field_demo/known_hand_surface.py','src/field_demo/known_hand_capture.py','src/field_demo/hand_aperture.py','src/field_demo/stopped_arm_monitor.py','src/isaac_port/fieldkit/assets/fr3v2-factory/derived-provenance.json','src/field_demo/model_records.py','src/field_demo/model_preset.py','src/field_demo/model_geometry.py','src/field_demo/mount_support.py','src/field_demo/stock_departure.py','src/field_demo/floor_constraint.py','src/isaac_port/fieldkit/assets/ring-support-vertices.npz','src/isaac_port/fieldkit/assets/ring-support-provenance.json','src/field_demo/commissioning_test.py','src/field_demo/__main__.py','src/field_demo/stock_contact.py','src/field_demo/selected_inventory_guard.py','src/field_demo/native_hand_geometry.py','src/field_demo/hand_monitor.py','src/isaac_port/fieldkit/assets/native-hand/surfaces.npz','src/isaac_port/fieldkit/assets/native-hand/surface-provenance.json','src/field_demo/camera_rectification.py','src/field_demo/residual_occupancy.py','src/field_demo/perception_models.py','src/field_demo/sensor_facts.py','src/field_demo/payload_boundary.py','src/isaac_port/fieldkit/payload_load.cpp','src/isaac_port/fieldkit/payload_load_core.hpp','src/isaac_port/fieldkit/read_state.cpp','src/field_demo/random_cycle.py','src/field_demo/random_perception.py',
              'src/isaac_port/fieldkit/assets/fr3v2-factory/collision-models.npz','src/isaac_port/fieldkit/assets/fr3v2-factory/enclosure-proof.json','src/isaac_port/fieldkit/assets/fr3v2-factory/arm-columns.json','src/isaac_port/fieldkit/assets/fr3v2-factory/provenance.json','src/isaac_port/fieldkit/assets/fr3v2-factory/kinematics.yaml','src/isaac_port/fieldkit/assets/fr3v2-factory/franka_arm.xacro','src/isaac_port/fieldkit/assets/fr3v2-factory/fr3v2.urdf.xacro','src/isaac_port/fieldkit/assets/fr3v2-factory/inertials.yaml','src/isaac_port/fieldkit/assets/fr3v2-factory/link0.stl','src/isaac_port/fieldkit/assets/fr3v2-factory/link1.stl','src/isaac_port/fieldkit/assets/fr3v2-factory/link2.stl','src/isaac_port/fieldkit/assets/fr3v2-factory/link3.stl','src/isaac_port/fieldkit/assets/fr3v2-factory/link4.stl','src/isaac_port/fieldkit/assets/fr3v2-factory/link5.stl','src/isaac_port/fieldkit/assets/fr3v2-factory/link6.stl','src/isaac_port/fieldkit/assets/fr3v2-factory/link7.stl',
              'src/field_demo/random_planner.py','src/field_demo/collision.py',
              'src/field_demo/stabilization.py','src/field_demo/stabilized/rgbd_scattered_rings.py',
              'src/field_demo/stabilized/rgbd_geometry.py','src/isaac_port/fieldkit/site.py',
              'src/isaac_port/fieldkit/hand.py','src/isaac_port/fieldkit/site_motion.cpp',
              'src/isaac_port/fieldkit/libfranka_stage.cpp','src/isaac_port/fieldkit/servo_core.hpp',
              'src/isaac_port/fieldkit/dynamic_plan.cpp','src/isaac_port/fieldkit/franka_hand.cpp',
              'src/isaac_port/fieldkit/site_watchdog.hpp','src/isaac_port/fieldkit/robot_mode.hpp',
              'src/field_demo/camera.py','src/field_demo/observations.py','src/field_demo/calibration.py',
              'src/isaac_port/fieldkit/libfranka_stage.hpp')
CAPABILITIES=('optional_bounded_fresh_cap_PIN_preclose_retarget_and_actual_other_empty_park','optional_closed_full_robot_link_palm_world_box_refinement_unchanged_UNKNOWN','optional_full_original_STL_convex_self_refinement_same_uncertainty_and_margin','optional_actual_verified_common_Duo_fixed_link0_pairs_only','optional_measured_RGBD_PIN_cap_circle_plane_no_predicted_XY_fallback','optional_bounded_PIN_RGBD_SDK_prealignment_and_same_strict_predispatch_gate','optional_5mrad_native_tracking_stop_profile_with_real_two_arm_records','bounded_read_only_native_IK_seeds_actual_qd_start','explicit_1M_residual_voxel_budget','candidate_only_KD_queries_identical_occupancy','actual_SDK_Hand_aperture_geometry_and_200ms_drift_guard','full_stopped_SDK_Hand_open_close_sweep','actual_stopped_arm_SDK_wrench_clock_monitor','bounded_same_class_alternate_pickup_search','pre_motion_only_fresh_scene_reselection','official_fr3v2_per_link_SDK_FK_column_geometry','actual_SDK_TCP_factory_native_Hand_mount','content_bound_model_derived_not_measured_records','one_actual_installation_match_record','original_64_sector_CAD_payloads','fixed_base_named_support_plane_contact_only','bounded_support_contact_departure_not_free_space','measured_floor_constrained_RGBD_inference','issued_upward_departure_max70mm','selected_CAD_named_support_floor_contact_only','experimental_stopped_payload_boundaries','RGBD_SDK_part_facts_without_part_PLC','fresh_fixture_after_withdraw','conservative_residual_voxel_occupancy','authentic_SDK_lens_rectification','first_actual_1mm_STOP_bootstrap_without_prior_stop_pass','explicit_low_speed_live_single_pick_and_cycle','bounded_finger_stock_contact','RGBD_complementary_view_single_item_proof','planned_bin_return_and_empty_hand_proof','actual_SDK_Hand_sample_expires_200ms','seed_free_rgbd_rings','fresh_held_G_before_transfer_and_insertion',
              'screened_dynamic_sdk_path','donor_release_after_receiver_native_and_RGBD',
              'fixed_field_camera_calibration','three_part_retention_after_hand_withdrawal',
              'no_simulation_joint_trace_replay')


def file_hash(path):return sha(Path(path).read_bytes())


def checked_file(root,name,expected,dependencies,label):
    root=Path(root).resolve();path=(root/name).resolve()
    if not path.is_relative_to(root) or not path.is_file() or Path(root/name).is_symlink():
        raise ValueError('Profile dependency must be a local package file: '+name)
    if file_hash(path)!=expected:raise ValueError('Profile dependency changed: '+name)
    dependencies[label]=dict(path=str(path),sha256=expected)
    return path


def verify_import(directory):
    directory=Path(directory).resolve();manifest=json.loads((directory/'IMPORT.json').read_text())
    if manifest.get('full_task_pass') is not True or manifest.get('hardware_tested') is not False or manifest.get('packaged_detector_matches') is not True:
        raise ValueError('Selected source must be a completed compatible simulated full-task success')
    dependencies={}
    for name,expected in manifest.get('copied_verified_source',manifest.get('source_hashes',{})).items():
        checked_file(directory,name,expected,dependencies,'source_code_'+name)
    records=manifest.get('copied_verified_records',{})
    for name,expected in records.items():checked_file(directory,name,expected,dependencies,'source_record_'+name)
    for name,expected in manifest.get('derived_files',{}).items():checked_file(directory,name,expected,dependencies,'derived_'+name)
    for name,expected in [('pipeline-results.json',manifest['source_result_sha256']),('code-manifest.json',manifest['source_manifest_sha256'])]:
        if records.get(name)!=expected:raise ValueError('Missing exact successful source evidence: '+name)
    report=json.loads((directory/'pipeline-results.json').read_text())
    code_manifest=json.loads((directory/'code-manifest.json').read_text())
    if report.get('full_task_pass') is not True or report.get('hardware_tested') is not False:
        raise ValueError('Source success evidence changed')
    for name,expected in manifest.get('copied_verified_source',manifest.get('source_hashes',{})).items():
        if code_manifest.get(name)!=expected:raise ValueError('Source code manifest mismatch: '+name)
    return manifest,report,dependencies


def create_profile(directory,*,engine_root=None,output=None):
    """Create explicit policy provenance; no simulator coordinates enter a recipe."""
    directory=Path(directory).resolve();engine_root=Path(engine_root or ROOT).resolve()
    imported,report,_=verify_import(directory)
    if imported['source_run']!='native-roas-table-scatter-speed-v128-closer-transfer-445':
        raise ValueError('This field strategy requires the explicitly selected v128 success')
    trial=json.loads((directory/'trial-config.json').read_text())
    if trial.get('parallel_transfer_approach') is not False or trial.get('path_local_timing') is not True:
        raise ValueError('Unexpected v128 strategy evidence')
    profile=dict(schema_version=1,profile_id=PROFILE_ID,source_import='IMPORT.json',source_import_sha256=file_hash(directory/'IMPORT.json'),
        source_run=imported['source_run'],simulation_wall_elapsed_s=report['wall_verified_elapsed_s'],
        experimental_not_field_ready=True,live_random_cycle_enabled=True,hardware_tested=False,physical_cycle_time_claimed=False,simulation_coordinates_applied=False,simulation_joint_trace_replayed=False,
        runtime_capabilities=list(CAPABILITIES),runtime_files={name:file_hash(engine_root/name) for name in ENGINE_FILES},
        strategy=dict(part_order=['ring1','ring2','pin'],ring_transfer='donor_first_to_distinct_receiver',pin_transfer='direct_same_hand',
            execution_mode='sequential_verified_phases',grasp_selection='fresh_image_accessible_instance_and_measured_grasp_family',
            presentation='actual_field_measured_transfer_poses_replanned_with_actual_SDK',
            receiver_grasp='actual_field_measured_transform_plus_fresh_image_G',
            timing='native_field_limits_and_current_SDK_model_retime_every_path',
            source_hints=dict(prefer_near_handover_branch=trial.get('prefer_near_handover_branch'),
                local_receiver_park=trial.get('local_left_park'),sort_donor_orientations=trial.get('sort_donor_orientations'),
                center_large_receiver_grasp=trial.get('center_large_receiver_grasp'),path_local_timing=trial.get('path_local_timing'),
                wait_for_ring_transport_clear=trial.get('wait_for_ring_transport_clear'))),
        limitations=['43.533 seconds is one simulated source run, not a physical speed setting.',
            'Source world poses, q/dq traces, drive rates, finger forces and simulated camera placement are reference data only.',
            'Near-transfer and local-parking hints require measured field poses; no simulation scene coordinate is converted to hardware.',
            'Current native field runtime remains sequential and replans from measured SDK states under its own conservative limits.'])
    target=Path(output) if output else directory/'field-profile.json'
    if target.exists():raise FileExistsError('Existing selected profile is preserved; choose a new output')
    target.write_text(json.dumps(profile,indent=2)+'\n')
    return profile


def load_profile(path,*,engine_root=None):
    path=Path(path).resolve();engine_root=Path(engine_root or ROOT).resolve()
    profile=json.loads(path.read_text());dependencies={}
    if profile.get('schema_version')!=1 or profile.get('profile_id')!=PROFILE_ID:
        raise ValueError('Explicit compatible v128 field profile required')
    if any(profile.get(name) is not False for name in ('hardware_tested','physical_cycle_time_claimed','simulation_coordinates_applied','simulation_joint_trace_replayed')):
        raise ValueError('Profile cannot claim hardware timing or replay simulated commands')
    if tuple(profile.get('runtime_capabilities',()))!=CAPABILITIES:raise ValueError('Profile runtime contract changed')
    strategy=profile.get('strategy',{})
    expected=dict(part_order=['ring1','ring2','pin'],ring_transfer='donor_first_to_distinct_receiver',pin_transfer='direct_same_hand',execution_mode='sequential_verified_phases')
    if any(strategy.get(key)!=value for key,value in expected.items()):raise ValueError('Profile does not describe the implemented v128-compatible phase strategy')
    if set(profile.get('runtime_files',{}))!=set(ENGINE_FILES):raise ValueError('Complete runtime engine binding required')
    for name,expected in profile['runtime_files'].items():checked_file(engine_root,name,expected,dependencies,'engine_'+name)
    import_path=checked_file(path.parent,profile['source_import'],profile['source_import_sha256'],dependencies,'source_import')
    imported,report,source_dependencies=verify_import(import_path.parent);dependencies.update(source_dependencies)
    if imported['source_run']!=profile['source_run'] or imported['source_run']!='native-roas-table-scatter-speed-v128-closer-transfer-445':
        raise ValueError('Profile points at a different stabilization run')
    if profile['simulation_wall_elapsed_s']!=report.get('wall_verified_elapsed_s'):
        raise ValueError('Reference simulation time differs from checked source report')
    for name in DETECTOR:
        raw=(engine_root/'src/field_demo/stabilized'/name).read_bytes().replace(b'from .rgbd_geometry import',b'from rgbd_geometry import')
        if sha(raw)!=imported['source_hashes'][name]:raise ValueError('Selected source detector does not match the executing port')
    dependencies['profile']=dict(path=str(path),sha256=file_hash(path))
    profile['_path']=str(path);profile['_sha256']=file_hash(path);profile['_dependency_files']=dependencies
    return profile


def bind_recipe(recipe,selection,*,engine_root=None):
    """Select matching phase roles while keeping every pose/force a field input."""
    if not isinstance(selection,dict) or set(selection)!={'path','sha256'}:raise ValueError('Profile selection requires exact path and SHA256')
    root=Path(recipe['_root']);path=(root/selection['path']).resolve()
    if file_hash(path)!=selection['sha256']:raise ValueError('Selected stabilization profile changed')
    profile=load_profile(path,engine_root=engine_root)
    if [entry['part'] for entry in recipe['parts']]!=profile['strategy']['part_order']:
        raise ValueError('V128 phase order must be ring1, ring2, pin')
    for entry in recipe['parts']:
        donor,receiver=entry['pickup_side'],entry['insert_side']
        if entry['part'].startswith('ring') and donor==receiver:raise ValueError('V128 rings require measured donor-to-receiver handover')
        if entry['part']=='pin' and donor!=receiver:raise ValueError('V128 pin uses a direct same-hand insertion')
    return dict(profile_id=profile['profile_id'],path=profile['_path'],sha256=profile['_sha256'],
        source_run=profile['source_run'],simulation_wall_elapsed_s=profile['simulation_wall_elapsed_s'],
        physical_speed_claimed=False,dependency_files=profile['_dependency_files'])


def revalidate_binding(binding,*,engine_root=None):
    profile=load_profile(binding['path'],engine_root=engine_root)
    if profile['_sha256']!=binding['sha256'] or profile['profile_id']!=binding['profile_id']:
        raise ValueError('Active stabilization selection changed; stop and revalidate')
    return profile


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['inspect','import','profile','check','list'])
    p.add_argument('--source-run',type=Path);p.add_argument('--output',type=Path);p.add_argument('--profile',type=Path)
    a=p.parse_args(argv)
    try:
        if a.command=='list':
            result=dict(profiles=[str(path.relative_to(ROOT)) for path in sorted((ROOT/'stabilization').glob('*/field-profile.json'))],hardware_ready=False)
        elif a.command=='check':
            if not a.profile:p.error('check requires --profile')
            profile=load_profile(a.profile);result={key:profile[key] for key in ('profile_id','source_run','simulation_wall_elapsed_s','hardware_tested','physical_cycle_time_claimed')}
            result.update(state='compatible_profile_checked',dependencies=len(profile['_dependency_files']),hardware_ready=False)
        elif a.command=='profile':
            if not a.source_run:p.error('profile requires imported --source-run directory')
            result=create_profile(a.source_run,output=a.output)
        elif a.command=='import':
            if not a.source_run or not a.output:p.error('import requires --source-run and --output')
            result=import_run(a.source_run,a.output)
        else:
            if not a.source_run:p.error('inspect requires --source-run')
            result,_=inspect_run(a.source_run)
        print(json.dumps(result,indent=2));return 0
    except (OSError,ValueError,KeyError,TypeError) as e:
        print(json.dumps(dict(state='stabilization_rejected',reason=str(e),source_modified=False)));return 2


if __name__=='__main__':raise SystemExit(main())
