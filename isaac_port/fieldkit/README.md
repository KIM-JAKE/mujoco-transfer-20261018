# M80 현장 연결 준비

현재 배포본은 **실기 통합 준비본**이다. `git clone` 후 사전 점검과 작업 정책의 dry run을 실행하고 FR3의 상태를 읽는 도구를 빌드할 수 있다. 실제 두 팔·그리퍼·PLC를 연결한 자동 조립은 아직 검증하지 않았다. 시뮬레이션 좌표를 실기 관절 명령으로 바로 재생하지 않는다.

## 1. 가벼운 복제와 오프라인 점검

이 과정에는 GPU, Isaac Sim, SAM3 체크포인트, 기존 MuJoCo 전체 복원이 필요 없다. Python 3.10 이상을 사용한다.

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/KIM-JAKE/mujoco-transfer-20261018.git mujoco
cd mujoco
python3 -m isaac_port.fieldkit preflight
python3 -m isaac_port.fieldkit dry-run
python3 -m unittest isaac_port.fieldkit.test_fieldkit -v
```

`preflight`의 종료 코드 **2는 현장 통합이 남았다는 뜻**이다. 현재 `hardware_ready: false`, `live_motion_enabled: false`가 정상이다. `dry-run`의 성공은 정책 호출 순서와 인터록 계약을 확인하며 물리 성공이나 실행 시간을 뜻하지 않는다.

### Isaac Sim 재현용 입력

전체 이관 아카이브 대신 약 116MB의 `../assets/physx-seed.zip.part-*`에 원본 참조 mesh, 변환 USD, 접촉 형상·기준 궤적을 별도로 넣었다. 다음 명령은 조각과 내부 파일 SHA-256을 확인하고 복원한다. 기존 파일 내용이 다르면 덮어쓰지 않고 중단한다.

```bash
python3 isaac_port/bundle_assets.py restore
```

별도 빈 폴더에 758개 파일을 복원하고 MuJoCo 모델(30 DOF)을 로딩하는 검사를 통과했다. 실행 환경과 실기 보정값을 담은 패키지는 아니다. `run_contact.sh`는 AIDAS의 공용 `isaac-run` 및 개인 패키지 환경을 이용한다. 최신 기록의 15초 판정 실패도 그대로 재현·기록하며, 결과는 `acceptance.json`에서 확인한다.

## 2. Code as Policies 방식의 연결 지점

`policy.py`의 `assemble(cell)`은 양팔 링 파지 → 삽입 → 클램프 확인 → 손 해제 → 핀 조립 → 버튼 순서의 Python 작업 정책이다. `Cell` 인터페이스를 현장 제어기에 연결한다.

| 함수 | 현장 어댑터가 해야 할 일 |
|---|---|
| `execute_parallel(actions)` | 두 팔의 충돌을 검사한 동시 동작 또는 그리퍼·PLC 명령을 실행한다. 각 동작에 시간 제한과 실패 상태를 둔다. |
| `observe()` | 실제 파지·삽입·클램프·해제 피드백과 fault를 반환한다. 명령 전송 성공을 센서 확인으로 대체하지 않는다. |
| `press_button(...)` | 세 부품 해제 확인 후 실제 버튼 신호·변위와 유지 시간을 확인한다. |
| `stop()` | FR3 및 주변 액추에이터를 현장 제어기의 정지 절차로 전환한다. |

1kHz FR3 서보 루프는 libfranka/현장 제어기에서 담당하고 이 Python 정책은 작업 단계와 피드백을 조율한다. 현재 제공 어댑터는 `DryRunCell`이며 **실기용 `Cell` 어댑터는 아직 구현·검증되지 않았다**. 이 상태를 설정 파일의 boolean 하나로 실기 인증으로 바꿀 수 없다.

시뮬레이션에서 가능한 개별 패드 접촉력·정확한 부품 자세는 실기에 바로 존재하는 신호가 아니다. Robotiq의 object detection만으로 삽입 정렬 25µm를 확인했다고 간주하지 않는다. 실제 지그 센서, 로봇 외력, 비전·접촉 탐색 중 무엇으로 각 상태를 판정할지 현장에서 연결해야 한다.

## 3. FR3 읽기 전용 연결 확인

로봇 Desk에서 **두 로봇 각각의 System Image 버전**을 먼저 기록한다. 버전에 맞는 libfranka를 설치한다. 예를 들어 공식 호환표에서 System Image 5.9 이상은 libfranka 0.18 이상, 5.7.2 이상 5.9 미만은 0.15 이상 0.18 미만으로 구분한다. 최신 버전을 일괄 설치하지 않는다. [공식 호환표](https://franka.de/fr3-compatibility-matrix)

이미 올바른 SDK가 설치된 현장 PC에서:

```bash
cmake -S isaac_port/fieldkit -B /tmp/m80-field-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/m80-field-build --parallel 2
/tmp/m80-field-build/m80_read_state LEFT_ROBOT_IP > /tmp/m80-left-state.json
/tmp/m80-field-build/m80_read_state RIGHT_ROBOT_IP > /tmp/m80-right-state.json
```

`LEFT_ROBOT_IP`, `RIGHT_ROBOT_IP`는 실제 로봇 주소로 바꾼다. AIDAS 서버 접속 IP와 로봇 IP는 별개다. 이 도구는 `Robot` 연결과 `readOnce()`만 사용한다. `control()`, error recovery, 충돌 한계·payload 변경을 호출하지 않는다. Realtime 설정 무시는 **상태 읽기에만** 적용한다.

출력은 관절 상태, 토크, TCP 변환, 설정된 tool/load 질량, 로봇 모드, 서버 프로토콜 버전이다. 서버 프로토콜 번호는 System Image 전체 버전 문자열이 아니므로 Desk 기록과 함께 보관한다. 이 테스트만으로 1kHz 제어 네트워크 품질을 입증하지 않는다.

검증 범위: 별도 Ubuntu 20.04 빌드 환경에서 공식 libfranka 0.18.1 패키지로 상태 읽기 도구와 단계 실행 라이브러리의 전체 빌드·링크를 통과했다. Release 빌드에서 궤적 검사와 SDK 링크 테스트 2개도 통과했다. 로봇 상태 읽기와 동작 실행은 현장 로봇에 연결하지 않아 확인하지 못했다.

### libfranka 단계 실행 API

`libfranka_stage.hpp`의 `run_joint_stage()`는 현장 애플리케이션에 연결할 C++ 단계 실행 API다. `m80_joint_stage` 라이브러리로 빌드한다. 실측 좌표와 부하로 별도 계획한 관절 경로를 입력받으며, 시뮬레이션 파일을 자동 재생하는 실행기는 제공하지 않는다.

- `Trajectory`는 q/dq/ddq 경계 조건을 갖는 5차 구간을 만들고, Bernstein 구간 포락으로 위치·위치별 속도·가속도·jerk를 연속 시간에서 검사한다. 명령 상한은 최대 95%, 초기 연결 기본값은 10%다.
- 프로토콜 10(System Image 5.9 계열 이후)의 FR3 한계를 사용한다. 다른 버전은 해당 한계로 다시 계획해야 한다. 실제 시작 관절 상태·부하 질량·오류·현장 I/O를 검사하며 초기 자세로 자동 이동하지 않는다.
- 1kHz 콜백에서는 관절 명령과 원자적 인터록을 처리한다. SDK 속도 제한과 100Hz 필터를 유지하고, 통신 주기·추종 오차·측정된 `tau_J_d` 변화율이 계약을 벗어나면 예외를 통해 동작을 취소한다.
- 양팔 시작 동기화와 상호 정지, PLC·Robotiq 통신, 외력 기반 삽입·복구, Python `Cell` 연결은 현장 어댑터의 구현 항목이다. 현재 라이브러리만으로 자동 조립을 실행할 수 없다.

빌드 후 `ctest --test-dir /tmp/m80-field-build --output-on-failure`로 오프라인 검사를 실행한다. 오래된 CMake에서는 빌드 폴더로 이동해 `ctest --output-on-failure`를 실행한다. 테스트는 로봇 연결이나 운동을 호출하지 않는다.

## 4. 현장 설정

```bash
cp isaac_port/fieldkit/config.example.json /tmp/m80-site.json
# 현장 실측값을 /tmp/m80-site.json에 채운 뒤
python3 -m isaac_port.fieldkit preflight --config /tmp/m80-site.json
```

| 항목 | 필요한 실측·확인 |
|---|---|
| 로봇·SDK | 좌우 IP, System Image, 맞는 libfranka, 관절 한계, 1kHz 통신 품질 |
| 좌표 | 좌우 base→설비 변환, TCP·그리퍼 패드, 지그와 공급 위치 |
| 부하 | 그리퍼·어댑터·케이블 및 부품의 질량·무게중심·관성 |
| 그리퍼 | 실제 2F-85 통신 방식, 위치/속도/힘 설정, object detection, 응답 시간 |
| 지그 | 공압·기계식 클램프의 PLC I/O, 닫힘 확인, 스트로크와 지연 |
| 삽입 | 정렬 관측, 힘·토크 중단 조건, 관측 지연, 허용 공차 |
| 버튼 | 입력 신호, 실제 스트로크, 눌림 확인 시간 |

95%는 공식 관절 명령 한도의 설계 상한이다. 검증된 실기 commissioning 속도를 뜻하지 않는다. 현장 제어기의 제한, 정지 기능과 충돌 검사를 유지하며, 초기 자세 이동·복귀와 고속 공정 시간을 따로 기록한다.

## 5. 현재 성능과 시뮬레이션 가정

최신 원본 배치 `contact-original-sensors-01`은 전체 조립·버튼 기능과 FR3 운동학 검사를 통과했지만 **15.129167초**로 시간 목표를 실패했다. 광축만 제외했으며 센서 위치 이동은 0이다. 근거: `../reports/original-sensors-01/acceptance.json`. 실기 준비 상태는 `../reports/latest-readiness.json`을 참조한다.

- 이전 **14.729초**는 센서 6개를 12–60mm 이동한 시험 배치의 단일 PhysX 에피소드다. 원본 배치 실기 성능으로 사용할 수 없다.
- 원본 CAD의 `E32-T11N-BASE`는 OMRON 투과형 광섬유 센서다. 정밀 BRep에서 본체/짧은 tail과 렌즈 앞에서 반대편 센서까지 이어지는 지름 0.3mm, 길이 754.2mm 광축 표시 solid를 분리했다. 광축을 강체 레일로 처리한 것이 이전 간섭 판정의 원인이었다. 새 모델은 광축만 접촉에서 제외하고 본체·tail·장착 위치를 유지한다. [OMRON 제품](https://industrial.omron.eu/en/products/E32-T11N-2M)
- 14.729초 명령의 1kHz 보간에서는 관절 q/v/a/jerk가 통과했지만, 수입 모델의 중력 제외 역동역학 토크 변화율은 최대 약 **7303Nm/s**로 공식 1000Nm/s를 넘었다. 접촉·마찰·부품 하중을 제외한 모델 경고이며 실측 토크가 아니다. 실제 제어기 응답과 토크 변화율의 확인이 필요하다. [FR3 한계](https://frankarobotics.github.io/docs/robot_specifications.html)
- PhysX 480Hz implicit PD, 이상적인 중력 보상, 120Hz 지연 없는 정확한 부품 자세, 가정한 클램프·공급 슬리브·버튼을 사용한다. 실제 1kHz FCI, 실제 I/O, 공차·마찰·센서 지연에 대한 반복 검증은 남아 있다.

근거: `../reports/hardware-audit/14p729-readiness.json`, `component-identification.json`, `../reports/optimization-goal.json`. 이후 검증 결과가 추가되어도 개별 실행의 `hardware_execution_certified`와 `source_cad_nominal_fit_certified`를 확인한다.

## 마감 규칙

최적화 목표는 전체 동적 조립·버튼 주기 **10.0초 미만**이다. 한국 시간 **2026-10-19 14:00**에 새 최적화와 실행 중인 최적화를 중단한다. 그때의 검증된 결과, 실패 근거, 현장 연결 코드와 이 문서를 정리해 GitHub에 올린다. 미달한 결과를 배속하거나 실기 가능한 10초로 표시하지 않는다.
