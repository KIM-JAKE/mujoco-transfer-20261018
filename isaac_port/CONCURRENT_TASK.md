# 양팔 독립 제어 실험

이 경로는 원본 CAD의 센서 장착 위치를 유지한다. 광축 표시 solid만 제외하며 센서 본체와 tail은 강체다. 왼팔의 핀 파지 방향을 원통 목 둘레로 -60° 바꾸고, 접촉이 없는 이동 구간을 줄인다. 작업물은 PhysX 동역학으로 움직이며 파지 후 pose를 강제하거나 손에 고정 joint를 만들지 않는다.

현재 **실험 경로**다. 오프라인 9초대 계산은 실제 접촉 성공이나 실기 성능을 뜻하지 않는다. 최신 원본 배치 전체 기능·FR3 한도·엄격한 10초 미만 통과 기록은 `reports/concurrent-08/acceptance.json`의 **9.625초**다. 같은 조건의 독립 실행 2회에서 9.625초를 확인했다. 위치·센서·I/O 지연을 포함한 현장 반복 성공률과 안전한 동시 제어는 아직 검증하지 않았다. 근거는 `reports/concurrent-08-repeatability.json`이다. 최신 실험 근거는 `reports/concurrent-experiments.json`을 확인한다. 실기 준비는 [fieldkit/README.md](fieldkit/README.md)를 따른다.

## 경로와 시계

`prepare_shortcut_candidate.py`는 번들 seed에서 IK와 공간 경로를 다시 만든다. 마지막 핀 목 접근은 원래의 측면 접근을 보존한다. 위에서 대각선으로 내려오는 이전 shortcut은 넓은 핀 머리와 충돌해 폐기했다. `audit_pin_pickup.py`는 파지 시작 전 이 충돌을 따로 검사한다.

`prepare_concurrent_timing.py`는 각 팔에 독립된 시간축을 부여한다. 한쪽의 파지·정렬·클램프 대기는 다른 팔의 시간을 멈추지 않는다. 양팔 교차 구간에는 정지 가능한 경계와 상대 팔의 실제 진행도 조건을 둔다. 버튼 접근은 세 부품의 해제가 모두 확인된 후 진행한다. 95% 명령 한도와 실제 PhysX 480Hz q/v/a/jerk 중단 조건을 유지한다.

`audit_concurrent_nominal.py`는 120Hz에서 두 로봇의 hull 충돌을 검사한다. 실제 지연·부품 접촉·설비 간섭·동역학 검사를 대신하지 않는다. 원래 단일 시간축에서 검사한 경로라도 동시 실행 순서가 바뀌면 새로 검사해야 한다.

## 재현

먼저 저장소 루트에서 `python3 isaac_port/bundle_assets.py restore`를 실행한다. 다음 준비 단계에는 NumPy, SciPy, MuJoCo, python-fcl, trimesh 환경을 사용한다. 준비 단계는 Kit 프로세스 밖에서 실행한다. Ruckig와 HiGHS를 Kit 안에서 직접 import하지 않는다.

```bash
python isaac_port/prepare_shortcut_candidate.py --output isaac_port/generated/contact_task/concurrent-reproduction
python isaac_port/prepare_concurrent_timing.py \
  --base isaac_port/generated/contact_task/concurrent-reproduction/planning-base \
  --output isaac_port/generated/contact_task/concurrent-reproduction/fr3-concurrent-profile.npz \
  --aggressive-contact-timing --coordinated \
  --left-ring-clear 1.96 --right-ring-clear 3.4 \
  --left-pin-time-reserve 1.04 --right-return-time-reserve 1.1 \
  --pin-approach-min .45 --pin-fine-min .35 --button-min .2
python isaac_port/audit_pin_pickup.py \
  --recording isaac_port/generated/contact_task/concurrent-reproduction \
  --plan isaac_port/generated/contact_task/concurrent-reproduction/arm-posture-fr3.npz \
  --output /tmp/m80-pin-pickup.json
python isaac_port/audit_concurrent_nominal.py \
  --base isaac_port/generated/contact_task/concurrent-reproduction/planning-base \
  --plan-root isaac_port/generated/contact_task/concurrent-reproduction \
  --output /tmp/m80-cross-arm.json
```

이 명령은 현재 실험 후보를 준비한다. 실행 결과의 `passed` 필드와 함께 검사 범위를 읽는다. 경로 NPZ와 관측 부품 reference는 `reports/shortcut-reproduction.json`에서 최대 1e-12 기준으로 이전 후보와 같은 값을 재현했다.

Isaac Sim에서는 다음 스크립트를 **예약한 개인 GPU 세션 안에서**, 컨테이너 경로로 실행한다. 기본 `/workspace/project` mount와 저장소 이름 `mujoco`를 가정한다. 출력 폴더는 매 실행마다 새 이름을 쓴다.

```bash
python /workspace/project/mujoco/isaac_port/test_concurrent_task.py \
  --output /workspace/project/mujoco/isaac_port/runs/concurrent-reproduction-01 \
  --reference-dir /workspace/project/mujoco/isaac_port/generated/contact_task/concurrent-reproduction \
  --plan-root /workspace/project/mujoco/isaac_port/generated/contact_task/concurrent-reproduction \
  --feedback --precision --posture --fast --clamps --fr3-limits \
  --efficient-timing --physical-sensors --target-10 --cycle-deadline 20 --soft-clamps
```

실행 후 동일한 환경에서 아래 검사를 수행한다. 프로세스 종료 성공만으로 조립 성공을 판단하지 않는다.

```bash
python isaac_port/evaluate_fr3_limits.py isaac_port/runs/concurrent-reproduction-01
python isaac_port/evaluate_contact.py isaac_port/runs/concurrent-reproduction-01
python isaac_port/verify_contact_recording.py isaac_port/runs/concurrent-reproduction-01
python isaac_port/audit_hardware_readiness.py isaac_port/runs/concurrent-reproduction-01
```

`acceptance.json`의 전체 조립·접촉·FR3 한도·엄격한 10초 미만 판정을 함께 확인한다. `hardware-readiness.json`은 별도의 모델 토크·토크 변화율과 현장 미검증 조건을 기록한다. 시뮬레이션에서 통과해도 `hardware_execution_certified`는 false다.

이번 최적화 작업은 각 실행에 `--respect-cutoff`를 사용하고 한국 시간 2026-10-19 낮 12시에 중단하고 준비된 최종본의 GitHub 푸시를 시작한다. 10초 미만을 달성해도 마감까지 계속 단축한다. 마감 이후 같은 기록을 재현할 때는 이 옵션을 생략할 수 있다.

`--soft-clamps`는 링 클램프 각 jaw의 힘 상한을 초기 2N에서 접촉 후 80ms 동안 5N으로 높이는 가정이다. 핀 jaw는 기존 3N을 유지한다. 실물에 비례 압력 제어가 있는지 미확인이다. 링 교차 구간은 임의 지연의 모든 자세 조합에서 안전하지 않으므로 실제 어댑터에는 추가 진행도 인터록 또는 동시 정지가 필요하다. 핀 이송·삽입 교차 조합 3,721개는 충돌 없이 통과했지만 추종 오차 여유를 포함한 연속 증명은 아니다.

`prepare_concurrent_timing.py --strict-clamp-alignment`는 팔이 계획된 착좌 정지점에 도달하고 실제 관절 속도가 0.03rad/s 미만이며 부품 위치/각도 오차가 150µm/0.01rad 안에 있을 때 클램프를 시작한다. 힘·스트로크·성공 기준은 그대로다. 원래 배치의 `contact-strict-clamps-01`은 전체 기능·실제 관절 한도·기록 검사를 **9.764583초**에 통과했다. 가장 빠른 9.566667초 후보와 구분하며, 단일 에피소드다. 근거는 `reports/strict-clamps-01/`에 있다.

`--pin-prealignment`는 핀 지그 진입 전에 정지 가능한 정렬 지점을 추가한다. `--left-pin-lift-time-reserve`는 공급 슬리브에서 들어 올리는 구간에 시간 여유를 둔다. 위치 변경 시험을 위한 선택 사항이며 각 새 동시 경로의 PhysX 검사가 필요하다. 공급 위치 변경은 `test_concurrent_task.py --initial-scene`으로 초기 부품과 선택한 공급 홀더에만 적용한다. M80 삽입 지그는 옮기지 않는다.

`python3 isaac_port/archive_run.py restore isaac_port/reports/strict-clamps-01/measured-evidence.zip`은 약 5MB의 측정·입력·실행 코드 49개 파일을 복원한다. 중복 CAD USD를 생략한 자료로, 모델 검사는 seed 복원이 필요하고 렌더는 원래 입력에서 USD를 다시 만들어야 한다. 모든 조각·내부 파일 SHA를 검사하는 복원과 복원 후 독립 FK/기록 검사를 통과했다.

원시 기록과 실행 당시 구현 snapshot은 두 개의 `reports/concurrent-08/measured-run.zip.part-*`에 들어 있다. `python3 isaac_port/archive_run.py restore isaac_port/reports/concurrent-08/measured-run.zip`은 조각과 50개 내부 파일의 SHA-256을 확인한 뒤 기본 `isaac_port/runs`에 복원한다. 이미 다른 내용이 있으면 덮어쓰지 않는다.

## 최신 9.566667초 기록

같은 공간 경로에서 일부 접촉 clock만 줄인 `contact-trim-01`은 동일 초기 조건 2회에서 전체 PhysX/FR3/기록 검사를 통과했다. [요약·실기 제한](reports/contact-trim-01/summary.json), [1배속 영상](reports/contact-trim-01/m80-9p567-1x.mp4)을 참조한다. 저장소 루트에서 측정 입력과 당시 구현을 복원한다.

```bash
python3 isaac_port/archive_run.py restore isaac_port/reports/contact-trim-01/measured-evidence.zip
```

재실행 시 `--reference-dir`와 `--plan-root`에 복원된 `isaac_port/runs/contact-trim-01`을 모두 지정하고, 기존 공통 옵션에 `--right-arm-damping 1800`을 추가한다. 새 출력 폴더를 사용한다. 대형 USD를 중복 포함하지 않은 약 5MB의 근거이며 형상은 seed에서 재생성한다. 당시 입력·clock을 사용한 측정 재현이며 실기 모션 파일이 아니다.

## 측정된 정지 상태를 사용하는 양팔 대기

`--conservative-ring-handoff`는 왼팔 기준 2.4에서 정지 상태를 확인한 뒤 오른팔의 기준 2.1 이후 이동을 허용하고, 오른팔 링 해제 후 정지 상태를 확인한 뒤 왼팔이 계속 진행하게 한다. 관절 실제 위치·속도가 없으면 기본 실행은 거부한다. `audit_concurrent_nominal.py`만 `nominal_only=True`로 명시하여 정적인 경로 검사를 수행한다. 이 오프라인 결과는 측정된 정지 상태가 아니다.

원래 배치의 `contact-conservative-handoff-01`은 실제 PhysX 조립·버튼·FR3·기록을 13.53125초에 통과했고 10초 미만 조건은 미달이다. 왼팔 damping 800, 오른팔 1800, `--strict-clamp-alignment`, `--left-pin-lift-time-reserve 1.15`를 사용했다. [근거](reports/conservative-handoff-01/summary.json)와 네 구간 20,604개 자세 조합의 표본 검사를 함께 제공한다. 추종 오차 여유나 현장 안전 인증을 포함하지 않는다.

```bash
python3 isaac_port/archive_run.py restore isaac_port/reports/conservative-handoff-01/measured-evidence.zip
```

복원된 run을 `--reference-dir`와 `--plan-root`에 지정하고 공통 native 옵션에 `--arm-damping 800 --right-arm-damping 1800`을 더해 새 출력 폴더에서 실행한다. 다른 시간 정책·초기 위치는 별도 재검증한다.

같은 공간 경로에서 `--conservative-ring-handoff --handoff-right-clear 2.82`를 사용한 후속 시험은 11.947917초에 전체 기능·실제 FR3·기록 검사를 통과했다. 엄격한 10초 미만 조건은 미달이다. 왼팔2.4/오른팔2.82에서 실제 정지 상태를 측정했으며, 후반 자세 조합151×151개 표본도 통과했다. [별도 근거와 복원](reports/conservative-early-handoff-01/summary.json)을 참조한다.
