"""Fresh-process CPU PhysX component. Generated source blocks retain original494 code.

Evaluation-only SDK/body/contact measurements never enter replay commands. This
component is not a full task acceptance test and does not restore solver state.
"""
from pathlib import Path
import os,time,json,hashlib,types,gc
import numpy as np
C_BASE=Path(os.environ['M80_COMPONENT_BASE']);C_H=Path(os.environ['M80_COMPONENT_H'])
C_OUT=Path(os.environ['M80_COMPONENT_OUT']);C_THREADS=int(os.environ['M80_COMPONENT_THREADS'])
C_META=json.loads((C_BASE/'input-source-seals.json').read_text());C_PLAN=C_META['plan']
C_COLD_STARTED=time.perf_counter();C_COLD_CPU=time.process_time()
C_SEALS={}
for C_name,C_seal in C_META['blocks'].items():
    C_path=C_BASE/(C_name+'.py');C_bytes=C_path.read_bytes()
    C_expected=C_seal.get('generated_sha256',C_seal['extracted_sha256'])
    assert hashlib.sha256(C_bytes).hexdigest()==C_expected,C_name
    C_SEALS[C_name]=C_expected
# Exact initial stage/controller preparation comes from the original driver. The
# replacement __file__ preserves original HERE-relative input resolution.
__file__=str(C_H/'run_realtime_recipe.py')
exec(compile((C_BASE/'setup_prefix.py').read_text(),__file__,'exec'),globals())
assert startup_config['fixed_supply_mode'] and startup_config['scattered_control']
assert startup_config['scattered_physics_substeps']==4
assert startup_config['operation_solver_position_iterations']==128 and startup_config['operation_solver_velocity_iterations']==64
assert startup_config['native_shared_motor_drive'] and startup_config['continuous_force_hold']
assert startup_config['native_grip_pad_force_n']==30.
assert not startup_config.get('ring_contact_debug',False)
C_REQUEST_READBACK=carb.settings.get_settings().get_as_int('/persistent/physics/numThreads')
assert C_REQUEST_READBACK==C_THREADS
# Literal original clamp initialization, writeback switch and safety callback.
truth={p:view.create_rigid_body_view(paths['workpiece_'+p])for p in ['ring1','ring2','pin']}
joints={p:[UsdPhysics.PrismaticJoint(pr)for pr in stage.Traverse()if str(pr.GetPath()).startswith('/World/Clamps/'+p+'/')and pr.IsA(UsdPhysics.PrismaticJoint)]for p in truth}
controls=np.zeros((2,3));last_clamp=np.full(3,-1.)
exec(compile((C_BASE/'clamps_and_writeback.py').read_text(),__file__,'exec'),globals())
exec(compile((C_BASE/'contact_callback.py').read_text(),__file__,'exec'),globals())
exec(compile((C_BASE/'operation_settings.py').read_text(),__file__,'exec'),globals())
# Identical native scalar-width motor/preload controller, not a target replay
# through a simplified PD controller.
exec(compile((C_BASE/'native_controller_setup.py').read_text(),__file__,'exec'),globals())
assert shared_motor and loaded_ramp_s==.3 and loaded_initial_speed==.003 and loaded_final_speed==.0075
# Same source function body; the original four-step tensor trace uses a body view
# directly, with no ray camera, geometry worker, or body-pose controller input.
C_BODY_PATHS=json.loads((C_BASE/'evaluation-body-paths.json').read_text())['original_camera_tensor_body_paths']
assert len(C_BODY_PATHS)==34 and all(stage.GetPrimAtPath(p).HasAPI(UsdPhysics.RigidBodyAPI)for p in C_BODY_PATHS)
camera=types.SimpleNamespace(all_bodies=view.create_rigid_body_view(C_BODY_PATHS))
audit=RecipeAudit(cmd,aidx,DT);scheduled_clamps={};left_queue_clock=lambda:0
profile={'control_s':0.,'physics_s':0.,'clamp_flush_s':0.};started=time.perf_counter()
exec(compile((C_BASE/'scattered_step.py').read_text(),__file__,'exec'),globals())
# AST identity is checked again after extraction, including all SDK guard calls.
import ast
C_ORIGINAL_SOURCE=(C_H/'run_realtime_recipe.py').read_text()
assert hashlib.sha256(C_ORIGINAL_SOURCE.encode()).hexdigest()==C_PLAN['source_seals']['code_snapshot/run_realtime_recipe.py']
C_ORIGINAL_AST=ast.parse(C_ORIGINAL_SOURCE)
C_RUNTIME_AST_SEALS={}
for C_name in ['contacts','scattered_step','loaded_preload','set_native_grip_hold','update_motor_speed_limits']:
    C_source=(C_BASE/('setup_prefix.py'if C_name=='update_motor_speed_limits'else'contact_callback.py'if C_name=='contacts'else'native_controller_setup.py'if C_name in ['loaded_preload','set_native_grip_hold']else'scattered_step.py')).read_text()
    C_nodes=[n for n in ast.walk(ast.parse(C_source))if isinstance(n,ast.FunctionDef)and n.name==C_name]
    C_original_nodes=[n for n in ast.walk(C_ORIGINAL_AST)if isinstance(n,ast.FunctionDef)and n.name==C_name]
    assert len(C_nodes)==len(C_original_nodes)==1 and ast.dump(C_nodes[0],include_attributes=False)==ast.dump(C_original_nodes[0],include_attributes=False),C_name
    C_RUNTIME_AST_SEALS[C_name]=hashlib.sha256(ast.dump(C_nodes[0],include_attributes=False).encode()).hexdigest()
C_z=np.load(C_BASE/'source-command-field.npz',allow_pickle=False)
C_START,C_END=C_PLAN['timed']['rows'];C_FIELD=C_z['command'][:C_END].copy();phase=C_z['pin_phase'][:C_END].copy()
assert C_FIELD.dtype.str==C_PLAN['issued_field_dtype'] and list(C_FIELD.shape)==C_PLAN['issued_field_shape']
assert hashlib.sha256(C_FIELD.tobytes()).hexdigest()==C_PLAN['issued_field_sha256']
assert np.array_equal(C_FIELD[0],prepared_initial_command[0])
audit.validate(C_FIELD)
# An additional read-only evaluator subscription counts contacts. The original
# contact callback/subscription is retained unchanged and remains authoritative.
C_CONTACTS={'callbacks':0,'headers':0,'data':0,'small_right_contact_headers':0,'small_right_contact_data':0,'small_right_contact_peak_n':0.,'small_contact_controls':set()}
C_PATH_CACHE={};C_MEASURED_CONTACTS=None
C_CURRENT_ROW=-1
C_small_path=paths['workpiece_ring2']
def C_evaluate_contacts(headers,data):
    C_CONTACTS['callbacks']+=1;C_CONTACTS['headers']+=len(headers);C_CONTACTS['data']+=len(data)
    for h in headers:
        if not h.num_contact_data:continue
        for code in [h.actor0,h.actor1]:
            if code not in C_PATH_CACHE:C_PATH_CACHE[code]=str(PhysicsSchemaTools.intToSdfPath(code))
        p0,p1=C_PATH_CACHE[h.actor0],C_PATH_CACHE[h.actor1]
        if (p0==C_small_path and 'right_NativeHand'in p1)or(p1==C_small_path and 'right_NativeHand'in p0):
            C_CONTACTS['small_right_contact_headers']+=1;C_CONTACTS['small_right_contact_data']+=h.num_contact_data
            C_CONTACTS['small_contact_controls'].add(C_CURRENT_ROW)
            force=sum(float(np.linalg.norm(data[i].impulse))/(DT/active_physics_substeps)for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
            C_CONTACTS['small_right_contact_peak_n']=max(C_CONTACTS['small_right_contact_peak_n'],force)
C_evaluator_subscription=sim.subscribe_contact_report_events(C_evaluate_contacts)
def C_contact_snapshot():
    return {k:(sorted(v)if isinstance(v,set)else v)for k,v in C_CONTACTS.items()}
def C_task_times():
    values={}
    for p in Path('/proc/self/task').iterdir():
        try:
            text=(p/'stat').read_text();tail=text[text.rfind(')')+2:].split()
            values[p.name]={'comm':(p/'comm').read_text().strip(),'utime_ticks':int(tail[11]),'stime_ticks':int(tail[12])}
        except (FileNotFoundError,ProcessLookupError):pass
    return values
C_INITIAL_BODY=camera.all_bodies.get_transforms().copy();C_INITIAL_Q=art.get_dof_positions().copy();C_INITIAL_V=art.get_dof_velocities().copy()
C_COLD={'wall_s':time.perf_counter()-C_COLD_STARTED,'process_cpu_s':time.process_time()-C_COLD_CPU}
C_SUMMARY={'component_only':True,'full_task_pass':False,'hardware_tested':False,'CPU_PhysX_only':True,'GPU_physics':False,'requested_threads':C_THREADS,'setting_readback':C_REQUEST_READBACK,'zero_semantics':'undocumented','source_run':C_PLAN['source_run'],'source_seals':C_PLAN['source_seals'],'source_block_seals':C_SEALS,'issued_field_sha256':C_PLAN['issued_field_sha256'],'cold_setup':C_COLD,'timed_controls':C_END-C_START,'timed_sim_s':(C_END-C_START)*DT,'solver':report.get('operation_solver_iterations'),'native_contact_generation':report.get('native_contact_generation'),'materials':report['contact_material_audit'],'SDK_names':names,'tensor_body_paths':list(camera.all_bodies.prim_paths),'original_callback_retained':True,'original_controller_function_AST_seals':C_RUNTIME_AST_SEALS,'host_AST_seals':C_META['literal_function_AST_sha256'],'AST_comparison_same_runtime_version':True,'original_controller_wall_pacing_retained':True,'camera_planner_and_query_colliders_excluded':True,'static_and_robot_native_collisions_unchanged':True,'evaluator_only_tensor_poses':True,'physics_profile':None,'error':None}
(C_OUT/'running.json').write_text(json.dumps({'pid':os.getpid(),'requested_threads':C_THREADS,'start_epoch_s':time.time(),'source_command_sha256':C_PLAN['issued_field_sha256']},indent=2))
print('THREAD_COMPONENT_PREPARED',C_THREADS,C_COLD,flush=True)
C_WARM_STARTED=time.perf_counter();C_WARM_CPU=time.process_time();C_WARM_THREAD=time.thread_time();C_BEFORE_PROFILE=None;C_START_CLOCK=None
C_gc_was_enabled=gc.isenabled();gc.disable()
try:
    for C_CURRENT_ROW in range(C_END):
        if C_CURRENT_ROW==C_START:
            # Original loose_ring_pick enables force hold immediately before its
            # first recorded close row. No replayed free-body state is imposed.
            set_native_grip_hold('right',True,direction=-1.)
            C_SUMMARY['warmup']={'controls':C_START,'sim_s':C_START*DT,'wall_s':time.perf_counter()-C_WARM_STARTED,'process_cpu_s':time.process_time()-C_WARM_CPU,'calling_thread_cpu_s':time.thread_time()-C_WARM_THREAD,'physics_profile':dict(profile),'contact_counters':C_contact_snapshot()}
            C_BEFORE_PROFILE=dict(profile);C_BEFORE_CONTACTS=C_contact_snapshot();C_BEFORE_TASKS=C_task_times()
            C_START_CLOCK=(time.perf_counter(),time.process_time(),time.thread_time())
        cmd[:]=C_FIELD[C_CURRENT_ROW]
        scattered_step(str(phase[C_CURRENT_ROW]))
        if C_CURRENT_ROW in [C_START-1,C_START,2351,2578,2775,C_END-1]:
            print('THREAD_COMPONENT_ROW',C_THREADS,C_CURRENT_ROW,physical,profile,flush=True)
    C_END_CLOCK=(time.perf_counter(),time.process_time(),time.thread_time());C_AFTER_TASKS=C_task_times()
    assert audit.samples==C_END and np.array_equal(np.asarray([v['q']for v in actuators]),C_FIELD)
    C_SUMMARY['timed']={'wall_s':C_END_CLOCK[0]-C_START_CLOCK[0],'process_cpu_s':C_END_CLOCK[1]-C_START_CLOCK[1],'calling_thread_cpu_s':C_END_CLOCK[2]-C_START_CLOCK[2],'physics_profile':{k:profile[k]-C_BEFORE_PROFILE[k]for k in profile},'contact_counter_deltas':{k:C_CONTACTS[k]-C_BEFORE_CONTACTS[k]for k in C_CONTACTS if k!='small_contact_controls'},'small_contact_control_rows':[v for v in sorted(C_CONTACTS['small_contact_controls'])if C_START<=v<C_END],'task_times_before':C_BEFORE_TASKS,'task_times_after':C_AFTER_TASKS,'clock_ticks_per_s':os.sysconf('SC_CLK_TCK')}
    C_held_rows=[v for v in C_CONTACTS['small_contact_controls']if 2579<=v<C_END]
    C_final_small=truth['ring2'].get_transforms()[0].copy()
    C_initial_small=C_INITIAL_BODY[list(camera.all_bodies.prim_paths).index(paths['workpiece_ring2'])]
    C_SUMMARY['small_lift_evaluation_m']=float(C_final_small[2]-C_initial_small[2])
    C_SUMMARY['contact_rich_prefix_qualified']=bool(len(C_held_rows)>=240 and C_CONTACTS['small_right_contact_data']>0 and C_SUMMARY['small_lift_evaluation_m']>.03)
    C_SUMMARY['closed_contact_controls_after_breakaway']=len(C_held_rows)
    C_SUMMARY['physics_profile']=dict(profile);C_SUMMARY['motor_velocity_audit']=encoder_speed_audit;C_SUMMARY['command_audit']=audit.report();C_SUMMARY['cross_arm_peak_n']=float(peak)
except BaseException:
    C_SUMMARY['error']=traceback.format_exc();C_SUMMARY['last_row']=C_CURRENT_ROW;C_SUMMARY['last_physics_s']=physical;C_SUMMARY['physics_profile']=dict(profile)
finally:
    if C_gc_was_enabled:gc.enable()
    C_SUMMARY['all_contacts']=C_contact_snapshot();C_SUMMARY['setup_report']=report
    np.savez_compressed(C_OUT/'evaluation-only-state-trace.npz',command=np.asarray([v['q']for v in actuators]),phase=np.asarray([v['pin_phase']for v in actuators]),actual_q=np.asarray([v['actual_dof_position']for v in actuators]),actual_v=np.asarray([v['actual_dof_velocity']for v in actuators]),sim=np.asarray([v['phase']for v in actuators]),trace_sim=np.asarray([v['sim']for v in tape]),trace_q=np.asarray([v['q']for v in tape]),trace_links=np.asarray([v['links']for v in tape]),trace_bodies=np.asarray([v['camera_bodies']for v in tape]),trace_parts=np.asarray([v['parts']for v in tape]),initial_body=C_INITIAL_BODY,initial_q=C_INITIAL_Q,initial_v=C_INITIAL_V)
    (C_OUT/'result.json').write_text(json.dumps(C_SUMMARY,indent=2,allow_nan=False)+'\n')
    (C_OUT/'component.done').write_text('error\n'if C_SUMMARY['error']else'finished\n')
    print('THREAD_COMPONENT_RESULT',C_THREADS,C_SUMMARY.get('timed'),C_SUMMARY['error'],flush=True)
