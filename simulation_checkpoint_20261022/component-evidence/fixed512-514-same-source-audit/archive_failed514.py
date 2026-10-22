from pathlib import Path
import json,shutil,hashlib,numpy as np
P=Path('/home/gpuadmin/workspace/jake/mujoco-field-publish-20261022/simulation_checkpoint_20261022/experiments');run=next(Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs').glob('*v514-*'));O=Path(__file__).parent;d=P/'fixed-cache514';j=json.loads((run/'pipeline-results.json').read_text());pin=j['pin_skill_progress'];a=pin['preinsert_visual_alignment'];b=next(e for e in j['planning_diagnostics']if e.get('event')=='small_loaded_receiver_encoder_geometry_bound');sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();z=np.load(run/'executed-actuator-commands.npz')
for key in('command','actual_dof_position','actual_dof_velocity'):assert np.isfinite(z[key]).all()
assert not j['full_task_pass'];assert pin['RGBD_full_depth_check'];assert not j.get('FR3_command_limit_audit')
obs=json.loads((run/'rgb-observations.json').read_text());i=next(i for i,x in enumerate(obs)if x.get('mode')=='initial'and x.get('part')=='ring1');k=max(k for k,x in enumerate(obs[:i])if x.get('mode')=='held_left'and x.get('part')=='ring2')
s=dict(run=run.name,full_task_pass=False,wall_s=j['wall_verified_elapsed_s'],failure=j['error'].splitlines()[-1],canonical_branch=pin['pin_canonical_pickup_branch'],precision_image_count=len(a),original_precision_passed={key:a[-1].get(key)for key in('radial_error_m','radial_limit_m','axis_error_rad','axis_limit_rad','maximum_shaft_axis_radial_error_m','maximum_shaft_axis_radial_limit_m')},precision_sequence=[{key:x.get(key)for key in('iteration','radial_error_m','axis_error_rad')}for x in a],initial_full_axial_insert_executed=any(e['event']=='pin_single_axial_insert'for e in pin['events']),post_insert_depth_RGBD=pin['RGBD_full_depth_check'],blocked_depth_correction_before_issue=True,final_release_withdraw_and_retention_qualified=False,final_complete_command_field_audit_available=False,actual_SDK_motor_audit=j['encoder_motor_speed_audit'],independent_saved_all_arrays_finite=True,actual_issued_rows=len(z['command']),part_attachments=j['part_attachments'],workpiece_teleports_after_initialization=j['workpiece_teleports_after_initialization'],SMALL_binding_capture_age_physics_s=b['capture_age_physics_s'],original_maximum_capture_age_s=.25,camera_observation_indices=dict(required_held_left=k,deferred_next_initial=i),reset_count=len(pin.get('pin_feedback_resets_after_rebranch',[])),saved_route_admissions={key:any(e.get('event')==event for e in j['planning_diagnostics'])for key,event in [('SMALL_donor','saved_SMALL_donor_admitted'),('SMALL_LEFT720','saved_SMALL_LEFT_carry_admitted'),('LARGE_canonical1415','fixed_saved_canonical_carry_admitted')]},caveats=['Original collision gate stopped axial depth correction before motion, after initial insert. No completed PIN or whole-task pass.','Actual motor audit covers issued samples; final complete-field audit is unavailable.','No three-case qualification/default promotion; retain failures.'])
(O/'actual514-failed-audit.json').write_text(json.dumps(s,indent=2)+'\n')
shutil.rmtree(d/'service-sources');shutil.copytree(P/'fixed-cache495/service-sources',d/'service-sources');shutil.copytree(P/'fixed-cache510/canonical-pickup-qualification',d/'canonical-pickup-qualification');shutil.copytree('/tmp/fixed512-defer-next-initial',d/'camera-priority-qualification',ignore=shutil.ignore_patterns('__pycache__'));shutil.copy2(O/'actual514-failed-audit.json',d/'camera-priority-qualification/actual514-failed-audit.json')
status='512NOMINAL and513PLUS fullPASS;514MINUS originaldepth-correction collision stop after initial insertion. Same source cohort is NOT three-case qualified; default checkpoint is not promoted. This is not repeatability or hardware qualification.'
for n in(512,513,514):
 q=P/f'fixed-cache{n}';(q/'camera-priority-qualification/COHORT_STATUS.md').write_text('# Camera-priority cohort outcome\n\n'+status+'\n\nThe original250ms binding image-age and all sensing/retention/collision/actuator gates remain unchanged. Original mandatory held-left camera packet precedes the deferred next-part initial packet in all three actual trials. Full PIN/three-part completion is claimed only for512/513.514 did not release/withdraw the PIN after its blocked correction.\n')
 if n in(512,513):
  t=(q/'README.md').read_text();t=t.replace('동일420개 Python/427개 소스·고정 입력의512 NOMINAL/513 PLUS 결과를 기록하며 MINUS514는 아직 대기 중입니다.','동일420개 Python/427개 소스·고정 입력의512 NOMINAL/513 PLUS는 전체 통과했고 MINUS514는 최초 PIN 삽입 뒤 추가 깊이 보정의 원래 충돌 검사에서 중단됐습니다.')
  (q/'README.md').write_text(t)
 files={str(f.relative_to(q)):sha(f)for folder in('service-sources','canonical-pickup-qualification','camera-priority-qualification')for f in sorted((q/folder).rglob('*'))if f.is_file()};(q/'supplemental-source-evidence-manifest.json').write_text(json.dumps(files,indent=2)+'\n')
(d/'README.md').write_text(f'''# 514: 고정 MINUS 배치 — PIN 추가 깊이 보정의 원래 충돌 검사에서 중단

같은420개 Python/427개 소스·고정 입력으로 두 링을 삽입하고 PIN 최초 삽입까지 진행했으나 **{j['wall_verified_elapsed_s']:.5f}초**에 중단됐습니다. `full_task_pass=false`입니다. PIN 전체 완료·해제·손 철수·최종 유지 확인을 통과한 사례가 아닙니다.

새 camera-priority 순서로 작은 링 필수 바인딩 영상({k}번,0기준)이 다음 큰 링 초기 영상({i}번)보다 먼저 처리됐고 원래 바인딩 영상 나이는{b['capture_age_physics_s']:.6f}물리초로최대0.25초 이내였습니다. PIN canonical catalogue seed가 원래 전체 경로 검사를 통과했고 실제 접근 q6={pin['pin_canonical_pickup_branch']['actual_pre_q'][5]:.6f}rad/q7={pin['pin_canonical_pickup_branch']['actual_pre_q'][6]:.6f}rad였습니다.

PIN 정밀 보정은7회 관측 뒤 원래75µm/3mrad/150µm 기준을50.97µm/.4804mrad로 통과했습니다. 원래 최초 단일 축 삽입 뒤 RGB-D가 추가 깊이를 요구했고 `pin_axial_depth_correction` 계획의16번 구간에서 RIGHT 왼쪽 손가락 tip과 원래 M80 `instance_0554`의 간섭을 검출했습니다. 그 추가 보정은 명령을 내리기 전에 거부됐습니다. 충돌·정밀·속도·힘·센서 기준을 완화하거나 실패를 성공으로 기록하지 않았습니다.

실제 encoder 한도 검사는{len(z['command']):,}개 발행 명령에 대해 통과했습니다. 예외 종료로 최종 전체 명령 필드 한도 검사는 없으며 통과했다고 주장하지 않습니다. attachment·초기화 뒤 teleport는 없습니다. 실제 로봇은 미검증입니다.

같은 소스512 NOMINAL/513 PLUS는 전체 통과했고514 MINUS는 실패했습니다. 세 배치 통과·반복 성공률·일반화·기본 체크포인트 승격을 주장하지 않습니다. 원래 오류·관측·명령·정확한 소스와 `camera-priority-qualification/actual514-failed-audit.json`을 보존합니다. 서비스 소스는495의 현재30개 파일과 byte 단위로 같습니다.
''')
base=json.loads((P/'fixed-cache512/exact-source-manifest.json').read_text())
for n in(512,513,514):
 q=P/f'fixed-cache{n}';m=json.loads((q/'exact-source-manifest.json').read_text());assert m==base
 for rel,want in m.items():assert sha(q/rel)==want
 print(n,'verifiedsame420PY427inputs')
print('514age',b['capture_age_physics_s'],'observationorder',k,i,'depthRGBD',pin['RGBD_full_depth_check'])
