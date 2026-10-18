# M80 · FR3 DUO · G2 — Isaac Sim 포팅

**현장 연결 준비:** 엔지니어용 [FR3 온보딩](fieldkit/ONBOARDING_FR3.md)부터 읽는다. 코드와 API 개요는 [fieldkit/README.md](fieldkit/README.md)에 있다. 현재 실기 자동 실행은 미검증이다. 이전 14.729초는 센서를 이동한 시험 배치 결과이며 원본 설비 실기 성능을 뜻하지 않는다. 원본 CAD의 센서 광축을 강체 레일로 취급한 오류를 수정하고, 두 팔 독립 스케줄로 10초 미만을 최적화 중이다. 한국 시간 2026-10-19 낮 12시까지 달성 후에도 계속 단축하고, 낮 12시에 검증된 최종본의 GitHub 푸시를 시작한다. `python3 -m isaac_port.fieldkit preflight`로 현장 통합 잔여 항목을 확인한다.

**최신 원본 배치 검사:** `contact-trim-03`에서 **9.466667초**를 독립 2회 확인했다. 세 부품의 실제 PhysX 파지·삽입·유한 힘 클램프·손 해제·버튼 100ms 유지, 95% 명령 생성 상한과 실제 FR3 관절 한도·기록 검사를 통과했다. 의도하지 않은 로봇 접촉 0N. [검증·반복·영상](reports/contact-trim-03/summary.json), [1배속 MP4](reports/contact-trim-03/m80-9p467-1x.mp4). 공간 경로와 원본 센서 장착 위치를 유지하고 접촉 단계의 시각만 조정했다. **실기 시간은 미검증이다.** 기록을 재보간한 연속 C2 곡선은 jerk 최대 4806/5000rad/s³로 공식 상한 이내지만 95% 예산을 넘는다. 별도 실기 모델 검사 입력은 1% 늦춰 연속 95% 검사를 통과시켰다. 모델 토크 변화율 최대 6491/1000Nm/s, 임의 지연 시 양팔 간섭과 실제 주변 장치 응답은 남은 문제다. [실기 온보딩과 SDK 검사](fieldkit/ONBOARDING_FR3.md), [전체 형상 포함 이전 9.625초 재현](CONCURRENT_TASK.md).


**이전 위치 변경 시험 기록: 95% 명령 상한을 적용한 전체 PhysX 작업을 14.729초에 통과했다. 15초 미만 달성.** 이전 22.881초보다 35.6% 줄였다. 두 링·핀의 파지→삽입→클램프→손 해제→버튼 누름과 실제 FR3 관절 위치·속도·가속도·jerk 검사를 통과했다. 최대 실제 가속도 9.592/10rad/s², 의도치 않은 설비 접촉 0N이다. 수정된 시험 설비와 가정한 액추에이터의 단일 에피소드 결과이며 실기 인증이 아니다. 기본 실행은 `run_contact.sh`, 상세 조건은 [CONTACT_TASK.md](CONTACT_TASK.md), 근거는 `runs/contact-under15-07/acceptance.json`과 `fr3-limits.json`이다. 렌더 영상과 GIF는 측정 시각의 1배속이다.


**정정: 아래 7.908초 결과는 FR3 실기 속도·가속도 한도를 초과한다.** 영상은 측정 시각 기준 1배속이며 접촉 조립 검사는 통과했지만, 실기 가능한 8초 공정으로 검증한 결과는 아니다. 측정 기록에서 관절 속도 최대 4.497rad/s(해당 관절 공식 상한 2.62rad/s), 가속도 최대 57.812rad/s²(상한 10rad/s²)를 확인했다. 자세한 감사는 `runs/contact-fast-30/fr3-motion-audit.json`과 [CONTACT_TASK.md](CONTACT_TASK.md)에 있다. 접촉 판정의 `physical_simulation_pass`와 FR3 동작 한도 준수를 구분해야 한다.

MuJoCo 통합 교환 장면을 Isaac Sim 6.1의 **PhysX** 장면으로 변환한다. 기본 실행은 관절·RTX 검사이며, M80 접촉 작업은 `run_contact.sh`로 실행한다. 수정한 시험 배치에서 M80 세 부품 파지·삽입·클램프·버튼 작업을 FR3 운동학 한도와 함께 검증했다. 원본 MuJoCo 파일과 저장된 실험 보고서는 수정하지 않는다.

## 실행

AIDAS에서 아래 명령을 실행한다. 복원된 메인 MuJoCo 환경과 공용 `isaac-run`이 필요하다.

```bash
cd /home/gpuadmin/workspace/jake/mujoco
./isaac_port/run.sh --gpu auto
```

명령은 원본 형상을 검사한 뒤 GPU를 예약하고 독립적인 Docker 작업을 시작한다. 출력된 세션 이름과 `Run output` 폴더를 보관한다. 작업이 종료되면 GPU 예약도 자동 해제된다.

```bash
isaac-status
isaac-logs --session SESSION_NAME --follow
isaac-stop --session SESSION_NAME
```

기본 검사는 120Hz에서 480 physics step(4초)을 실행한다. DUO 좌우 팔과 G2 좌우 팔의 첫 관절에 작은 위치 명령을 보내고, G2의 `base_x` 관절도 구동한다(원본 배치에서는 월드 +Y 방향). 명령값과 실제 PhysX 측정값을 비교하며, 그리퍼 mimic 오차, 베이스 높이·기울기 제약과 유한 상태를 검사한다. 기본 영상은 앞뒤 정지 구간을 포함한 30fps MP4다. 인코더가 없으면 개인 패키지 경로에 `imageio-ffmpeg==0.6.0`을 설치한다.

```bash
# mesh/설정 변경 후 USD를 새로 변환
./isaac_port/run.sh --gpu 0 --fresh
# 영상 없이 수치 검사
./isaac_port/run.sh --gpu auto --no-video
```

## 결과 읽기

각 실행은 `isaac_port/runs/<실행 ID>/`에 새로 저장된다.

| 파일 | 용도 |
| --- | --- |
| `pipeline-status.json` | 전체 작업의 실행 PID, 단계, 종료 상태. `state: passed`가 최종 성공 조건 |
| `conversion.log`, `physics.log` | 실제 변환·시뮬레이션 로그 |
| `scene.usda` | 변환된 asset을 상대 경로로 참조하는 PhysX 장면과 카메라·조명·보정 설정 |
| `status.json` | 관절·베이스 추종 오차, mimic 오차와 검증 범위 |
| `initial-pose-audit.json` | 113개 body의 원본/변환 초기 위치·방향 비교 |
| `articulations.json` | G2 34 DOF, DUO 30 DOF와 매거진 articulation 목록 |
| `joint-trace.npy` | 시간·명령·측정 배열. 열 이름은 `status.json`의 `trace_columns` 참조 |
| `initial.png`, `final.png`, `physx-joint-control.mp4` | 실제 RTX 카메라 결과 |

`scene.usda`는 GUI에서 열 수 있다. 저장된 장면을 연 뒤 같은 초기 자세·명령 검사를 재현하려면 `simulate.py`를 사용한다. 검사 중 적용한 tensor 초기 자세와 명령 궤적은 실행 코드에 있으며, USD 파일만 여는 것으로 전체 검사 궤적이 재생되지는 않는다.

실패 후에는 상태 JSON과 해당 child 로그를 먼저 확인한다. `run.sh`를 다시 실행하면 새 결과 폴더를 만들고, 입력 hash가 일치하는 완료된 USD 변환만 재사용한다. 실패한 물리 실행의 중간 상태를 자동 재개하지 않는다.

## 변환 및 물리 설정

원본: `keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/g2_fr3_side_exchange_20261016/scene_outbound_candidate.xml`.

- `prepare.py`: asset 경로를 컨테이너 경로로 변환한다. 파일이 없는 인라인 mesh 3개는 원본의 정확한 hull 삼각형을 사용한 OBJ로 내보낸다. MuJoCo에서 다시 읽어 형상·질량·관성·관절·충돌 마스크 배열을 원본과 비교한다. 원본 XML과 모든 참조 asset의 SHA-256을 기록한다.
- `convert.py`: NVIDIA의 `MJCFImporter`로 USD를 생성하고 `Physics=physx` variant를 명시적으로 선택한다. mesh instance도 포함해 변환 결과를 검사한다.
- `simulate.py`: 단위 m/kg, z-up, 중력 9.81m/s², CPU PhysX TGS 120Hz, GPU RTX 렌더링을 사용한다. 초기 자세는 원본 `g2_departure_physical/final_state.npz`의 qpos에서 읽고 속도는 0으로 초기화한다. 원본의 전체 동역학 상태를 그대로 재개하는 기능은 아니다.
- G2의 x/y/yaw는 변환기가 D6 하나로 합친다. 이 D6는 reduced-coordinate articulation 밖의 제약으로 유지하며, articulation에는 나머지 34개 관절이 들어간다. 베이스 명령은 D6 drive로 보낸다. 원본의 평면 이동 제약을 유지하도록 z·roll·pitch 축을 명시적으로 잠근다. 제어 검사의 월드 방향은 원본 MuJoCo Jacobian에서 구한다.
- 변환기의 PhysX variant에서 빠진 2개의 그리퍼 mimic 관계를 원본 선형 equality에서 복원한다. 비선형 joint equality는 조용히 근사하지 않고 중단한다.
- 양의 질량을 가진 body는 MuJoCo가 컴파일한 질량·중심·관성·주축을 명시한다. 질량 0인 고정 프레임 7개에는 PhysX 수치 조건을 위한 질량 1e-6kg, 관성 1e-9kg·m²를 부여하고 결과에 기록한다.
- 원본의 `gravcomp=1`은 해당 body의 PhysX 중력 비활성화로 옮긴다. 부분 중력 보상은 별도 force controller가 필요하므로 현재 지원하지 않는다.

## 검증 범위

기본 `run.sh`의 검증 범위는 장면 변환과 관절 제어 smoke test다. 별도 `run_contact.sh`에서는 수정한 M80 시험 배치의 무중단 링/핀 조립·버튼 작업을 검증했다. **매거진 교환, 충돌 필터의 완전한 동등성, Isaac Lab 학습 환경과 SAM3.1 센서 연결은 아직 검증하지 않았다.** PhysX와 MuJoCo의 접촉·마찰·solver가 같다는 가정도 하지 않는다. 기존 다른 세션의 `physical_assembly_pass: false`와 원본 CAD의 조립 간섭 판정은 유지한다.

원본 CAD의 시각용 geometry를 임의로 모두 충돌체로 바꾸지 않는다. 원본에서 별도 FCL CAD 검사로 다루던 설비 간섭은 그 검사와 함께 재검증해야 한다. 공통 Isaac 이미지, 드라이버, 다른 사용자 환경은 이 포팅에서 변경하지 않는다.

공식 API: https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.asset.importer.mjcf/docs/index.html

## AIDAS 실제 검증 결과

`runs/20261018T041408Z-102388/`에서 전체 실행 `passed`. 자세한 요약은 `validation.json`에 있다.

- 원본/준비 MJCF의 형상·물리 배열 비교 통과. 변환된 113개 body의 초기 위치 최대 오차 약 2.18e-7m.
- 네 팔의 각 첫 관절을 실제 PhysX drive로 구동. 최대 추종 오차 DUO 약 0.00434rad, G2 약 0.00115rad.
- G2 `base_x` 실측 이동 약 0.07994m, 최대 추종 오차 약 0.00683m.
- 베이스 높이 최대 오차 약 1.95e-7m, 기울기 약 2.48e-7rad. 그리퍼 mimic 최대 오차 약 1.49e-8rad.
- 실제 RTX 영상 1280×720, 30fps, 6초 생성 및 전체 디코딩 통과. 새 USD 변환과 캐시 재사용 실행을 모두 확인했다.

## M80: 기존 8.060초 조립·버튼 기록 재생

```bash
cd /home/gpuadmin/workspace/jake/mujoco/isaac_port
./run_button_replay.sh           # 빈 GPU 자동 예약
./run_button_replay.sh 1         # GPU 1 지정
```

원본 `cap_button_parallel_alt_20261016`의 기록을 정확한 M80 장면으로 변환한다. 원본 궤적 485개 상태의 관절각·그리퍼·부품 mocap 위치에서 MuJoCo FK를 계산하고, 65개 바디의 월드 자세를 USD 애니메이션으로 저장한다. 모든 원본 상태와 USD 자세의 수치 일치를 검사한 다음 Isaac Sim RTX로 렌더링한다. 원본 파일은 변경하지 않는다.

| 기록 이벤트 | 시간 |
| --- | ---: |
| ring2 해제 | 3.244초 |
| ring1 해제 / 첫 2개 완료 | 5.750초 |
| pin 해제 / 3개 완료 | 7.303초 |
| 버튼 첫 접촉 | 7.912초 |
| 가정한 버튼 이동 완료 | 7.960초 |
| 기록된 녹색 신호 | 8.060초 |

출력은 `runs/button-<실행시각>-<PID>/`에 생성한다. `m80-three-parts-button-isaac.mp4`는 전체 동작·버튼·삽입부를 함께 보여 주며 원래 시간으로 재생한다. 마지막 상태는 확인을 위해 1초 유지한다. `replay.usda`를 Isaac Sim에서 열면 0~8.060초 애니메이션을 다시 볼 수 있다. `status.json`에는 원본 SHA-256, 자세 비교 결과와 영상 디코딩 검사가 있고, `pipeline-status.json`에는 실행 상태와 자식 프로세스 로그 위치가 있다.

이 실행은 **저장 궤적의 재생**이다. 물리는 `none`으로 설정하며 새 PhysX 제어기, 접촉·파지·힘 제어 삽입을 검증하지 않는다. 버튼 2mm 이동과 100ms 유지 및 녹색 신호는 원본의 가정이다. 원본 CAD 조립 간섭 문제도 남아 있으므로 `physical_assembly_pass: false`를 유지한다. `cap_button_under8_20261016`이라는 다른 폴더는 실제로 9.356초이며, 이 재생의 원본과 다르다.

2026-10-18 재생 확인: `runs/button-20261018T043500Z-camera/`에서 성공했다. 31,525개 바디 자세 비교(위치 오차 0m, 회전 최대 약 1.49e-7rad)와 영상 273프레임 전체 디코딩을 통과했다. 영상은 1344×720, 30fps, 9.1초(8.060초 주기 + 마지막 1초 유지 및 프레임 양자화)다. 요약은 `button-replay-validation.json`에 있다. 녹색 점은 기록 이벤트 표시이며 실제 버튼 동역학이 아니다.

## M80 실제 접촉·클램프·버튼 작업

접촉 기반 작업은 별도 진입점을 사용한다. 기존 `run.sh`의 관절 smoke test와 `run_button_replay.sh`의 원본 궤적 재생은 위의 검증 범위를 그대로 유지한다.

```bash
/home/gpuadmin/workspace/jake/mujoco/isaac_port/run_contact.sh
```

이 명령은 공용 실행 도구로 빈 GPU를 예약하고, 컨테이너 안에서 동역학 실행 → 독립 판정 → 측정 상태 렌더링을 수행한 뒤 예약을 해제한다. 필요한 개인 작업공간 패키지는 `mujoco==3.11.0`, `scipy==1.18.1`, `ruckig==0.12.2`다. 공용 이미지를 수정하지 않는다.

입력은 이 서버에서 준비한 `generated/contact_task`와 `generated/cap_button`이다. 입력을 새로 만들 때는 복원된 호스트 메인 `.venv`로 `prepare.py --scene cap_button`, `prepare_contact.py`, `export_precision_cad.py`, `export_precision_cad.py --pin-main`, `prepare_robot_filters.py`, `audit_cad_interference.py`, `audit_pin_brep.py`, `prepare_contact_timing.py --fast`, `plan_right_posture.py --fast`, `prepare_fr3_posture.py`를 실행하고 공용 Isaac 환경에서 `convert.py --scene cap_button`으로 USD를 준비한다. 이 입력 생성 과정과 테스트용 CAD 위치 변경을 검토한 뒤 사용한다.

실제 동적 부품, 힘 제한 클램프, 스프링 버튼, 성공 조건, 원본 CAD 간섭과 실측 전 가정은 [CONTACT_TASK.md](CONTACT_TASK.md)에 정리했다. 새 작업의 최종 판정은 `acceptance.json`의 `physical_simulation_pass`다. 원본 CAD와 실물 조립의 인증 여부는 별도 필드로 계속 `false`를 유지한다.

2026-10-18 최초 보수적 검증: `runs/contact-fr3-09/`에서 `physical_simulation_pass: true`, `fr3_kinematic_limits_pass: true`. 측정 최대 관절 가속도 7.104rad/s², jerk 4940.120rad/s³이며 의도치 않은 접촉은 0N이다. 세 부품의 양측 파지·고정·손 해제 후 유지와 버튼 100ms 누름을 통과했다. 초기 안정화를 뺀 공정 시간은 44.535초다. 보수적인 최초 검증본이며 최단 시간 최적화·실기 인증과 구분한다.

고리의 `purpose=guide`로 렌더에서 빠지던 문제를 수정했다. `runs/contact-fr3-09/render/m80-physx-contact.mp4`는 측정한 PhysX 상태를 1배속·30fps로 보여 준다. 파랑/주황 고리와 초록 핀을 구분하고, 오른쪽은 X-ray 확대 화면이다. `render/status.json`은 모든 영상 프레임의 디코딩·측정 자세 일치와 재생 속도를 확인한다. `recording-audit.json`은 코드·입력 SHA-256, 테이프/상태 기록 일치, 독립 FK와 native 관절 속도 대비 timestep 일치를 검사한다. 원본 CAD 간섭과 실측 전 모델 가정은 계속 남아 있다.

양팔 독립 시간축·접촉 인터록을 사용하는 10초 미만 실험과 재현 방법은 [CONCURRENT_TASK.md](CONCURRENT_TASK.md)에 정리했다. 해당 실험은 별도 PhysX 판정이 필요하며 실기 성능을 인증하지 않는다.
