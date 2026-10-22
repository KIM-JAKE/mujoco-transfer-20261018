from pathlib import Path
import json,hashlib,shutil,sys
PUB=Path('/home/gpuadmin/workspace/jake/mujoco-field-publish-20261022');E=PUB/'simulation_checkpoint_20261022/experiments';R=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs');O=Path(__file__).parent;sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for text in sys.argv[1:]:
 n=int(text);run=next(R.glob(f'*v{n}-*'));j=json.loads((run/'pipeline-results.json').read_text());d=E/f'fixed-cache{n}';assert d.exists()
 shutil.rmtree(d/'service-sources');shutil.copytree(E/'fixed-cache495/service-sources',d/'service-sources')
 shutil.copytree(E/'fixed-cache510/canonical-pickup-qualification',d/'canonical-pickup-qualification')
 q=d/'camera-priority-qualification';shutil.copytree('/tmp/fixed512-defer-next-initial',q,ignore=shutil.ignore_patterns('__pycache__'))
 pin=j.get('pin_skill_progress',{});a=json.loads((O/f'actual{n}-audit.json').read_text());a['canonical_branch']=pin.get('pin_canonical_pickup_branch');events=j['planning_diagnostics'];b=next(e for e in events if e.get('event')=='small_loaded_receiver_encoder_geometry_bound');a['original_SMALL_binding_capture_age_physics_s']=b['capture_age_physics_s'];a['original_SMALL_binding_maximum_age_physics_s']=.25
 a['saved_route_admissions']={key:any(e.get('event')==event for e in events)for key,event in [('SMALL_donor','saved_SMALL_donor_admitted'),('SMALL_LEFT720','saved_SMALL_LEFT_carry_admitted'),('LARGE_canonical1415','fixed_saved_canonical_carry_admitted')]}
 obs=json.loads((run/'rgb-observations.json').read_text());i=next(i for i,x in enumerate(obs)if x.get('mode')=='initial'and x.get('part')=='ring1');left=[(k,x)for k,x in enumerate(obs[:i])if x.get('mode')=='held_left'and x.get('part')=='ring2'];k,x=left[-1]
 a['camera_priority_public_observations']={'mandatory_SMAll_held_left_index_0based':k,'deferred_next_ring_initial_index_0based':i,'mandatory_frame_precedes_next_initial':k<i,'rows':[{key:y.get(key) for key in ('work','mode','part','captured_wall_s','ready_wall_s','delivered_wall_s','queue_wait_s','capture_s','processing_s','scene_sim_time_s')}for y in(x,obs[i])],'causal_latency_benchmark':False}
 (O/f'actual{n}-audit.json').write_text(json.dumps(a,indent=2)+'\n');shutil.copy2(O/f'actual{n}-audit.json',q/f'actual{n}-audit.json');shutil.copy2(O/'audit.py',q/'public_result_audit.py')
 qualification_note='512/513 are two completed cases; MINUS514 is pending. No three-case/default promotion is asserted in this per-case archive.'
 (q/'COHORT_STATUS.md').write_text('# Same-source camera-priority cohort\n\n'+qualification_note+'\n\nEvery original sensor/retention/collision/actuator gate remains authoritative. Camera-priority patch is scheduling only; the SAME next-part request is moved after the existing original SMALL receiver binding, with no added frame or dwell. Actual mandatory/deferred frame ordering and original capture-age values are recorded in the per-case audit. The original 250ms physical image-age limit is unchanged.\n\nCanonical PIN branch proof is copied from510 as component history; this case’s actual admission/source/q and full original precision/actuator results are recorded separately. Any saved-route rejection runs original guarded fallback and is not counted as reuse.\n')
 files={str(p.relative_to(d)):sha(p)for folder in('service-sources','canonical-pickup-qualification','camera-priority-qualification')for p in sorted((d/folder).rglob('*'))if p.is_file()}
 (d/'supplemental-source-evidence-manifest.json').write_text(json.dumps(files,indent=2)+'\n')
 source=json.loads((d/'exact-source-manifest.json').read_text());assert len(source)==427 and sum(k.endswith('.py')for k in source)==420
 for rel,want in source.items():assert sha(d/rel)==want
 for rel,want in files.items():assert sha(d/rel)==want
 mode={512:'기준 NOMINAL',513:'PLUS',514:'MINUS'}[n];v=pin['preinsert_visual_alignment'];last=v[-1];c=pin['pin_canonical_pickup_branch'];audit=j['FR3_command_limit_audit'];usage=a['saved_route_admissions']
 (d/'README.md').write_text(f'''# {n}: 고정 {mode} 배치 — camera-priority/canonical PIN 전체 통과

원래 Isaac Sim/PhysX에서 촬영·인식·계산·두 링과 핀의 파지·전달·삽입·고정·손 철수·유지 확인까지 **{j['wall_verified_elapsed_s']:.5f}초**에 전체 통과했습니다. 물리시간{j['simulation_elapsed_s']:.5f}초입니다.20/30초 목표는 미달이며 실제 로봇은 미검증입니다.

원래 작은 링 receiver 바인딩을 완료한 뒤 다음 큰 링 초기 영상 요청을 보냈습니다. 같은 요청의 순서만 바꾸며 새 프레임·대기·속도/힘 변화나 영상 나이 기준 완화는 없습니다. 실제 바인딩의 영상 나이는{b['capture_age_physics_s']:.6f}물리초로 원래최대0.25초 이내입니다. 원래511 영상 나이 중단은 별도 보존합니다.

PIN의 기존 canonical seed 우선순위에서 `{c['selected_source']}`가 원래 전체 경로 검사를 통과했습니다. 실제 접근 q6={c['actual_pre_q'][5]:.6f}rad/q7={c['actual_pre_q'][6]:.6f}rad입니다. 원래 정밀 기준75µm/3mrad/150µm를{len(v)}회 관측 뒤 {last['radial_error_m']*1e6:.2f}µm/{last['axis_error_rad']*1e3:.4f}mrad/shaft{last['maximum_shaft_axis_radial_error_m']*1e6:.2f}µm로 통과했습니다. 재분기 후 feedback reset은 실행되지 않았으므로 그 효과는 검증하지 않았습니다.

저장 경로 실제 승인: SMALL donor={usage['SMALL_donor']}, SMALL LEFT720={usage['SMALL_LEFT720']}, LARGE canonical1415={usage['LARGE_canonical1415']}. 저장 경로 미승인은 원래 전체 검사를 거친 재계획을 사용하며 저장 재사용으로 세지 않습니다. 현재 파지·반대 팔·놓인 부품·실제 encoder/명령·전체 손가락 너비 및 최종 삽입 검사는 유지됩니다.

전체{audit['samples']:,}개 명령 필드와 실제 encoder 한도 검사가 통과했습니다. 속도·가속도·jerk·47.5mm/s 손가락 속도·30N 힘·질량·마찰·원래 충돌/센서/유지 기준을 바꾸지 않았습니다. attachment·초기화 뒤 teleport는 없습니다.

동일420개 Python/427개 소스·고정 입력의512 NOMINAL/513 PLUS 결과를 기록하며 MINUS514는 아직 대기 중입니다. 이 기록은 세 배치 통과·반복 성공률·임의 적층·실기 성공이나 순수 단일수정 속도 개선을 주장하지 않습니다. 이전 실패508/511도 각각 보존합니다.

`pipeline-results.json`, `acceptance-summary.json`, `camera-priority-qualification/actual{n}-audit.json`, 정확한 소스 및 원래 명령·관측을 함께 확인하세요. 외부 서비스 소스는495의 현재30개 파일 패키지와 byte 단위로 같으며 추가 근거는 `supplemental-source-evidence-manifest.json`으로 검증했습니다.
''')
 print(n,'archived',len(source),'source',len(files),'supplemental','priority',k,i,'age',b['capture_age_physics_s'],'usage',usage)
