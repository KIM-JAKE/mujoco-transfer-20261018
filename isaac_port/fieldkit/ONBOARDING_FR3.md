# M80 · 두 대의 Franka Research 3 현장 온보딩

대상은 libfranka를 다루는 로봇 엔지니어다. **저장소 복제 → 오프라인 점검 → 상태 읽기 → 좌표·주변 장치 연결 → 저속 단일 단계 → 양팔 공정** 순서로 진행한다. 서버용 Isaac Sim과 로봇 옆 실시간 제어 PC는 역할이 다르다.

현재 제공 범위는 상태 읽기 실행 파일, 검사 가능한 C++ 관절 단계 실행 API, Python 작업 정책, 시뮬레이션 경로와 검증 기록이다. **실물 두 팔·Robotiq·PLC를 묶는 현장 어댑터는 아직 구현·검증되지 않았다.** 로봇 IP를 넣는 것만으로 전체 조립이 실행되는 배포본은 아니다. 이 문서의 남은 통합 항목을 작업 인계 기준으로 사용한다.

## 먼저 확인할 결과

| 항목 | 현재 근거와 의미 |
|---|---|
| 원본 M80 배치의 조립 | `../reports/concurrent-08/acceptance.json`: PhysX에서 링 2개와 핀 1개 파지·삽입·클램프·해제 후 버튼 유지까지 9.625초 |
| FR3 운동학 | 95% 명령 예산, 실제 시뮬레이션 관절 위치·위치별 속도·가속도·jerk 검사 통과 |
| 실물 같은 시간 | 미확인. 수입 모델 토크 변화율 최대 약 6714Nm/s, 실제 제어기·하중·접촉·통신 응답 미측정 |
| 양팔 지연 | 정상 실행은 통과했지만 링 교차 구간은 임의 지연 시 충돌 가능한 자세 조합이 있음. 추가 인터록 필요 |
| 클램프 | 링 jaw 힘을 접촉 후 80ms 동안 2→5N으로 올린 시뮬레이션 가정. 실물의 비례 압력 제어·지연 확인 필요 |
| 현장 연결 도구 | 공식 libfranka 0.18.1로 빌드·링크 및 오프라인 CTest 2개 통과. 실제 로봇 연결은 미실시 |

시간은 초기 자세 이동·보정·촬영·재계획을 제외한 공정 시간이다. 위치 추정과 경로 생성 소요 시간은 따로 측정한다. 95%는 계획 상한이며 첫 실기 시험 속도를 뜻하지 않는다.

## 1. 가져오기와 오프라인 확인

GitHub 인증은 현장 PC의 GitHub 도구나 자격 증명 관리자로 설정한다. 토큰을 명령 인자나 저장소 URL에 넣지 않는다.

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/KIM-JAKE/mujoco-transfer-20261018.git mujoco
cd mujoco
git rev-parse HEAD
python3 -m isaac_port.fieldkit dry-run
python3 -m unittest isaac_port.fieldkit.test_fieldkit -v
python3 -m isaac_port.fieldkit preflight
```

`dry-run`은 센서 확인과 단계 순서의 계약 검사다. 운동 명령을 보내지 않는다. `preflight` 종료 코드 2와 `hardware_ready: false`는 현재 통합 미완료 상태를 나타낸다. 설정 값을 채워도 이 검사는 실기 인증서를 만들지 않는다.

Isaac 재현용 CAD·모델만 필요하면 `python3 isaac_port/bundle_assets.py restore`를 실행한다. SAM3.1 및 전체 MuJoCo 이관 아카이브 복원은 필수가 아니다. 측정된 9.625초 실행을 복원하려면:

```bash
python3 isaac_port/archive_run.py restore isaac_port/reports/concurrent-08/measured-run.zip
```

실행 당시 코드·입력·실제 관절 및 부품 기록을 SHA-256으로 확인해 복원한다. 기존 파일 내용이 다르면 중단한다. `../CONCURRENT_TASK.md`의 재현 절차로 새 PhysX 실행을 검사한다.

## 2. 제어 PC와 로봇 연결

두 로봇의 모델·일련번호·Desk System Image 버전, 제어 PC OS·커널·NIC, libfranka 버전을 현장 기록에 남긴다. [공식 FR3 호환표](https://franka.de/fr3-compatibility-matrix)를 기준으로 SDK를 선택한다. 제공 관절 단계 API의 운동 한계 검사는 **서버 프로토콜 10**을 요구한다. 다른 버전의 한계로 자동 대체하지 않는다.

로봇과 같은 현장의 유선 제어 PC에서 FCI를 사용한다. 인터넷/VPN을 경유하는 AIDAS 서버 연결을 1kHz 제어 링크로 사용하지 않는다. FCI 활성화와 실시간 커널·사용자 권한·통신 시험은 설치한 System Image의 공식 FCI 문서에 맞춘다. 상태 읽기 성공과 ping 지연만으로 실시간 제어 품질을 판정하지 않는다.

맞는 SDK가 설치된 PC에서 저장소 루트 기준:

```bash
cmake -S isaac_port/fieldkit -B /tmp/m80-field-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/m80-field-build --parallel 2
cd /tmp/m80-field-build
ctest --output-on-failure
```

로봇의 실제 IP를 넣어 각각 읽는다. 서버 접속 IP는 로봇 IP가 아니다.

```bash
/tmp/m80-field-build/m80_read_state LEFT_ROBOT_IP > /tmp/m80-left-state.json
/tmp/m80-field-build/m80_read_state RIGHT_ROBOT_IP > /tmp/m80-right-state.json
```

이 도구는 상태 읽기만 수행한다. 관절 q/dq, 토크, TCP 변환, 설정된 tool/load 질량, 로봇 모드와 서버 프로토콜을 확인한다. 좌우 배선을 실제 로봇과 대조하고 로그를 보관한다. 통신 실패 시 IP·직접 연결·FCI 상태·SDK 호환성을 먼저 확인한다.

## 3. 좌표·TCP·하중 기록

저장소의 CAD 좌표를 실측 보정값으로 사용하지 않는다. 현장 설정은 저장소 밖에 보관한다.

```bash
cp isaac_port/fieldkit/config.example.json /tmp/m80-site.json
python3 -m isaac_port.fieldkit preflight --config /tmp/m80-site.json
```

설정 예제의 null은 실측·구현이 필요한 항목이다. 임의의 단위 행렬이나 true로 채우지 않는다.

| 기록 | 확인 방법과 사용처 |
|---|---|
| cell 프레임 | 설비에 고정한 원점·축·기준점을 사진과 수치로 기록. 설비가 움직이면 재보정 |
| `T_cell_base_left/right` | 각 로봇 base 점을 cell로 옮기는 4×4 변환. 양팔 공통 충돌 검사에 사용 |
| TCP·패드 | 실제 어댑터·2F-85 장착 방향, TCP, 손가락 패드 위치와 폭. 플랜지와 TCP를 구분 |
| tool/load | 그리퍼·어댑터·케이블·파지 부품의 질량, 무게중심, 관성. FR3 설정과 기록을 대조 |
| M80 지그·버튼 | 고정 삽입 목표·접근 방향·버튼 위치와 눌림 신호. 공급 부품 이동과 구분 |
| 공급 위치 | 링 적층·봉, 핀 홀더·슬리브. 홀더가 이동했는지 부품만 이동했는지 구분 |
| `T_cell_camera` | 카메라 점을 cell로 옮기는 변환. hand-eye/외부 보정 ID와 잔차·반복 측정 기록 |

변환 관례는 열벡터 `p_cell = T_cell_camera @ p_camera`다. 좌표 단위 m, 관절 rad, 힘 N, 토크 Nm를 사용한다. FR3 상태 배열의 행렬 저장 순서는 SDK 정의를 따르며, JSON 중첩 행렬로 바꿀 때 전치·축 방향을 알려진 기준점으로 확인한다.

## 4. 카메라와 위치 변경

카메라 모델은 아직 미정이다. 제공 인터페이스는 이미 보정된 metric point cloud 또는 rectified·registered depth의 NPZ와 동명 JSON이다. 카메라 SDK에서 이 형식으로 저장하는 어댑터와 실제 금속 부품 검출 성능 검증이 필요하다. RGB 영상만으로 미터 좌표를 얻었다고 간주하지 않는다.

- NPZ: `points_camera_m`은 Nx3 m. 또는 `depth`와 설정의 intrinsics·depth_scale_m을 사용한다. 선택적인 `labels`는 cloud 길이 또는 depth 해상도와 일치해야 한다.
- 동명 JSON: `camera_frame`, `calibration_id`, `capture_id`, timezone을 포함한 `captured_at_utc`.
- 카메라 설정: 같은 frame·calibration ID, `T_cell_camera`, 세 부품의 실측 `nominal_stock_poses`. depth 입력은 rectified/registered 여부와 명시적인 미터 변환 계수를 포함한다.

저장소 루트의 NumPy·SciPy 환경에서:

```bash
python -m isaac_port.fieldkit.vision /path/to/snapshot.npz \
  --config /path/to/camera-config.json --output /tmp/m80-observations.json
```

기본 실행은 2초보다 오래되거나 미래 시각인 촬영, 다른 보정 ID를 거부한다. `--replay`는 저장 데이터 재현 전용이다. 이 옵션으로 생성한 결과를 실기 실행 승인으로 사용하지 않는다. 현재 CAD 등록은 보정된 공급 위치 주변의 작은 이동을 가정한다. 원형 링의 축 둘레 회전은 관측했다고 주장하지 않고 기준 방향을 유지한다. 적은 가시 면적·반사·깊이 누락·잘못된 보정은 별도 현장 검증 대상이다.

동작 적응은 **공급 접근·파지·들어 올리기만 수정한 뒤 기존 최적 경로에 합류**한다. M80 삽입점과 버튼을 관측한 공급 이동량만큼 옮기지 않는다. 새 경로에는 공급 홀더 형상, 로봇·설비·양팔 충돌, 95% 한도 재타이밍과 PhysX 접촉 검사가 다시 필요하다. 저장 계획을 실행하기 직전 새 관측으로 부품 이동 여부를 재확인한다. 촬영 이후 부품이 움직이면 계획을 폐기한다.

## 5. 그리퍼·클램프·버튼 연결 계약

현장 개발자가 `policy.py`의 `Cell`을 구현한다. `assemble(cell)`은 CaP처럼 호출하는 작업 정책이며 1kHz 서보는 C++ 제어기가 담당한다.

| 신호/함수 | 실제로 연결할 내용 | 다음 단계 조건 |
|---|---|---|
| `execute_parallel(actions)` | 양팔 motion/Robotiq/PLC 명령, 시간 제한, 실제 완료·fault | 양팔 공통 진행도와 충돌 구역 예약을 확인 |
| 파지 확인 | 2F-85 object detection, 위치·전류·힘 등 실제 이용 가능한 신호 | 공중 이동 전에 파지 확인. 시뮬레이션 양측 pad 힘을 실기 신호로 가장하지 않음 |
| 삽입 확인 | 실제 지그 센서·외력·비전 또는 접촉 탐색 | 정렬·착좌 확인 후 클램프 요청 |
| 클램프 확인 | PLC 출력 주소·극성·압력·닫힘 입력·timeout | 닫힘 확인 뒤 손 해제 |
| 손 해제 | 실제 그리퍼 개방과 작업물 유지 | 세 부품 모두 확인 후 버튼 |
| `press_button(...)` | 실물 버튼 신호와 유지 시간 | 시뮬레이션의 1.8mm/100ms와 실물 규격을 대조 |
| `stop()` | 두 로봇과 주변 액추에이터의 현장 정지 절차 | 한 팔 fault 시 다른 팔도 대응. 자동 재시도·계속 진행 금지 |

Robotiq는 Franka Hand API와 다른 장치다. 실제 연결 방식(Modbus TCP/RTU 등), 주소·레지스터·응답 시간을 확인한 드라이버를 사용한다. PLC I/O와 파지 센서가 미정이라 임의 주소는 제공하지 않는다. 콜백에서 Python·카메라 처리·PLC 통신·파일 쓰기 등 블로킹 작업을 하지 않는다. 외부 작업 스레드가 검증된 상태를 원자적 gate로 전달한다.

## 6. 첫 움직임과 단계 API

`libfranka_stage.hpp`의 `run_joint_stage(robot, trajectory, site, gates)`를 현장 애플리케이션에서 호출한다. `m80_joint_stage`가 빌드 대상이다. 실측 좌표·실제 시작 자세로 계획한 `Trajectory`를 사용한다. 시뮬레이션 NPZ의 관절 값을 그대로 전달하지 않는다.

1. 실제 공구·하중·충돌 설정과 현장 정지 기능을 확인한다. 첫 시험은 단일 팔·비접촉 단계로 한다.
2. `FieldContext`에 현장 계획·TCP/I/O·토크 예산 확인, 보정 기록과 commissioning 기록, 기대 tool/load 질량을 넣는다. 초기 `command_fraction` 기본값은 0.1이며 이 값으로 새 궤적을 계획한다.
3. 시작 자세는 로봇의 정지 상태와 일치해야 한다. API는 자동 홈 이동을 하지 않으며 시작 불일치를 거부한다. 접근/복귀 경로도 따로 계획한다.
4. 궤적 검사 후 `AtomicGates.io_ready`를 실제 현장 조건과 연결한다. 중단 요청·I/O 손실 시 두 팔의 정지 처리를 통합한다.
5. 단일 파지·이동·저속 삽입·클램프·해제를 각각 검증한 뒤 양팔 동작을 연결한다. 강체 삽입 오차를 고강성 위치 명령으로 밀어 넣지 않는다. 현장 힘/토크 조건과 복구 절차를 정한다.
6. 실제 지연·파지 실패·클램프 timeout·한 팔 중단 시나리오까지 확인한 뒤 속도를 단계적으로 높인다.

API는 연속 시간의 q/위치별 속도/a/jerk를 보수적으로 검사하고, SDK rate limiter와 100Hz 필터를 유지한다. 실행 중 통신 주기·추종 오차와 desired torque 변화율을 감시한다. 이는 전체 설비의 안전 제어기나 양팔 충돌 회피 구현을 대체하지 않는다. `tau_J_d`는 로봇이 보고하는 desired torque이며 외부 토크 센서 실측값과 구분한다.

FR3 JointPositions에서 q/v/a/jerk는 필수 조건이고 토크·토크 변화율은 권장 조건이다. 로봇이 궤적을 보정할 수 있어 시뮬레이션과 같은 시간이 보장되지 않는다. 이 현장 단계 API는 자체 운용 계약으로 desired torque 변화율 1000Nm/s 초과 시 중단한다. [Franka 공식 제어 조건](https://frankarobotics.github.io/docs/robot_specifications.html)

## 7. 기록과 문제 발생 시

각 시험에 Git commit, SDK/System Image, 로봇 일련번호, 보정 ID, tool/load 설정, 계획 SHA, 촬영 ID, 시작 q, 명령·실제 q/dq, `tau_J_d`, 통신 주기, 그리퍼/PLC 타임스탬프, fault와 정지 이유를 보관한다. 공정 주기는 첫 동작부터 버튼 확인까지 재고 초기 이동·촬영·계획 시간도 별도로 남긴다.

| 증상 | 먼저 확인할 것 |
|---|---|
| CMake Franka/Poco 오류 | OS·SDK 패키지 호환성. 다른 배포판 라이브러리에 임의 symlink를 만들지 않음 |
| 프로토콜 불일치 | Desk 버전과 공식 호환표. 제공 운동 한계는 프로토콜 10용 |
| 시작 자세 거부 | q/q_d, 정지 여부, TCP·좌표 보정, 별도 접근 계획 |
| 관절/통신/토크 변화율 중단 | 원본 로그 보존, 하중·실시간 링크·궤적 재검토. 제한을 해제해 통과시키지 않음 |
| 링·핀 자세 등록 실패 | 단위·보정 ID·노출·반사·깊이·가시 면적·ROI. 실패 시 이전 관측으로 진행하지 않음 |
| 파지·착좌·클램프 미확인 | 실제 신호·극성·timeout 확인. 명령 성공만으로 다음 단계 진행하지 않음 |

실기 통합 완료 기준은 실제 좌표·주변 장치·양팔 정지·충돌 구역·지연 조건을 구현하고 현장에서 검증한 기록이다. `dry-run` 성공과 설정 파일 완성은 그 기록을 대신하지 않는다.
