"""Package accepted physics evidence; never changes a simulator pass criterion."""
from pathlib import Path
import json,sys,hashlib,datetime
from PIL import Image
H=Path(__file__).resolve().parent;PORT=H.parents[1];pin=PORT/'runs'/sys.argv[1];ring=PORT/'runs'/'native-roas-table-bin-quick07';result=json.load(open(pin/'physx-pickup-results.json'));seq=result.get('sequence',{});rings=json.load(open(ring/'physx-sequence-results.json'))
if not seq.get('final_assembly_pass')or not {'ring1','ring2'}.issubset(rings.get('completed_parts',[])):raise SystemExit('No accepted staged assembly evidence; package not marked as passed')
out=PORT/'runs'/'native-roas-table-bin-staged-demo-20261020';out.mkdir(exist_ok=True)
summary=dict(updated_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),state='staged_demo_pass',staged_assembly_pass=True,full_task_pass=False,continuous_cycle_pass=False,hardware_tested=False,vision_end_to_end_pass=False,cycle_time_s=None,initial_mixed_ring_counts=dict(large=5,small=5),ring_source_run=ring.name,pin_source_run=pin.name,pin_collision_approximation=result['pin_collision_approximation'],pin_clamp=dict(center_z_m=seq.get('pin_clamp_center_z_m'),pad_height_m=seq.get('pin_clamp_pad_height_m'),force_per_jaw_n=3,measured_on_hardware=False),finger_torsional_patch_radius_m=.003,final_checks=seq['final_checks'],note='Edited, staged physical trials. Ring trial used the initial finger box approximation. Pin trial uses the original finger mesh, simplified pin collisions and uncalibrated contact/clamp parameters. Rings start installed in the pin trial.')
(out/'report.json').write_text(json.dumps(summary,indent=2));(H/'bin-picking/STATUS.json').write_text(json.dumps(summary,indent=2));(H/'ACTIVE_STATUS.json').write_text(json.dumps(summary,indent=2))
for name in ['FinalGlobal','FinalAssembly']:
 p=pin/(name+'.ppm')
 if p.exists():Image.open(p).save(out/(name+'.png'))
manifest={}
for p in (pin/'code_snapshot').glob('*'):
 if p.is_file():manifest[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
(out/'code-sha256.json').write_text(json.dumps(manifest,indent=2))
text=f'''# 바구니 집기 → 링 2개·핀 1개: 첫 Isaac Sim 시연

두 종류의 링 10개(큰 링 5개·작은 링 5개)를 기존 바구니에 놓고, Franka Hand로 링 두 개와 핀 하나를 집어 삽입하는 단계별 물리 시연이다. 추가 책상·링 공급봉은 사용하지 않는다. 핀은 기존 트레이 위의 제안 홀더(3개 구멍 중 1개 적재)에서 집는다.

## 확인된 결과

링 두 개의 집기·삽입·손을 뗀 유지: `{ring.name}`. 핀 집기·삽입·손을 뗀 유지: `{pin.name}`. 핀 시험은 앞선 링 완료 상태를 불러와 시작한다. **세 부품이 설치된 최종 상태는 통과했으며, 하나의 연속 자동 사이클을 통과한 결과는 아니다.** `staged_assembly_pass=true`, `full_task_pass=false`, `cycle_time_s=null`로 구분한다.

최종 판정은 실제 PhysX 몸체 위치 오차 1.5 mm 이하, 축 기울기 0.05 rad 이하, 손 TCP가 각 부품에서 40 mm 이상 떨어진 상태에서 유지되는지를 사용했다. 상세 수치는 `isaac_port/runs/{out.name}/report.json`에 있다. 버튼 동작과 실기는 이번 시험 범위에 없다.

## 첫 시연의 모델 가정

모든 동역학·역기구학·장애물 질의는 Isaac Sim의 Omni PhysX CPU에서 수행했다. 제어에는 이상적인 시뮬레이션 물체 자세를 사용했다. 인식부터 조립까지 연결한 SAM 시험 결과는 아니다. 바구니는 한 개의 시연 배치이며, 임의 더미에 대한 성공률은 측정하지 않았다.

링 시험의 손가락은 초기 박스 근사 모델이고, 핀 시험은 제공된 Franka 손가락 충돌 메쉬를 쓴다. **핀 충돌 형상은 4개의 동축 원통으로 단순화했다. 끼움부 충돌 지름은 7.6 mm이며, CAD 공칭 8 mm에 대해 반경 0.2 mm의 시연용 여유를 둔다.** 원본 핀의 슬릿·국부 탄성·압입 공차를 검증한 결과로 해석하면 안 된다.

클램프는 실제 치수가 없는 유한 힘 모델이다(핀: 턱당 3 N, 접촉면 높이 6 mm, 중심 높이 1.511 m). 핀 접촉 강성 100 kN/m, 스프링 접점 10 kN/m, 손가락의 비틀림 마찰 접촉 반경 3 mm/최소 2 mm 역시 실측 보정값이 아니다. 원본 설비의 고정 CAD 배치는 유지한다.

## 현재 서버에서 재실행

저장된 두 링 완료 체크포인트 `{ring.name}`와 기존 CAD 자산이 필요하다. 저장소 루트에서 새 실행 이름을 지정한다.

```bash
OPENBLAS_NUM_THREADS=1 keti_m80_mujoco_20261013/.venv/bin/python \\
  isaac_port/native_hand_layout/roas-table/run_cpu_physx.py \\
  run_checkpoint_pin.py candidate native-roas-table-bin-pin-mytrial01
```

실행 결과의 `physx-pickup-results.json`과 `physx-sequence-results.json`을 확인한다. GPU를 노출하지 않는 CPU 실행이며, 다른 사용자의 GPU 예약을 사용하지 않는다. 영상은 기록된 몸체 상태의 간이 PhysX 렌더링과 시간 압축 편집이다. 실제 작업 시간이나 실기 사용 승인을 뜻하지 않는다.
'''
(H/'bin-picking/README.md').write_text(text);print(out)
