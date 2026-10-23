# FR3 CaP 실기 연결 패키지

양팔 FR3의 CaP 공통 API, 링 작업 경계, 실제 libfranka·Franka Hand·PLC·RealSense 연결 코드를 한 설치본으로 제공합니다. **카메라와 로봇 좌표를 현장 설정으로 분리**했으며 Isaac Sim 없이 설치·설정 검사·상태 읽기·카메라 취득을 준비할 수 있습니다.

현재 상태는 **실기 연결 및 commissioning 준비본**입니다. 실제 장비에서 전체 링 조립은 검증하지 않았으며, 현재 실측 장면의 전체 충돌 승인기가 제공되지 않아 자동 조립 구동은 차단됩니다. `motion_enabled` 하나를 바꿔 이 조건을 대체할 수 없습니다. 시뮬레이션에서 성공한 관절 배열을 실기에서 재생하는 패키지가 아닙니다.

## 설치와 첫 점검

```bash
git clone --filter=blob:none --sparse \
  --branch codex/capx-fr3-hardware-package-20261023 --single-branch \
  https://github.com/KIM-JAKE/mujoco-transfer-20261018.git
cd mujoco-transfer-20261018
git sparse-checkout set capx_hardware
cd capx_hardware
python3 -m venv .venv
. .venv/bin/activate
python -m pip install .
capx-fr3 verify-vendor
capx-fr3 init ./site-local
capx-fr3 preflight --config ./site-local/hardware.json
```

마지막 명령의 종료 코드 **2와 미입력 항목 목록은 초기 상태에서 정상**입니다. 이 단계는 장치에 연결하지 않습니다. 설치 의존성은 Python 3.10 이상, NumPy, SciPy이며 카메라를 사용할 PC에만 `python -m pip install '.[camera]'`로 SDK Python 바인딩을 추가합니다. C++ 로봇 도구는 장비 System Image와 호환되는 libfranka를 현장 PC에 설치한 뒤 빌드합니다.

실제 장치 제어 대상은 원본 드라이버의 `fcntl`·POSIX·libfranka 의존성을 지원하는 **호환 Linux 현장 PC**입니다. 현장 OS·CPU·SDK는 아직 확인하지 않았으며, Python 설치 성공이 Windows/macOS의 실제 FR3·Hand 구동 호환성을 뜻하지 않습니다.

`capx-fr3 init`은 기존 디렉터리를 덮어쓰지 않습니다. 새 폴더의 설정·보정 기록·실행 로그는 저장소에 포함하지 마세요. 설치 위치에서 생성한 드라이버 manifest는 소스 해시를 확인하며, 설치본을 이동한 경우 새 site 디렉터리를 생성합니다.

## 현장에서 채울 것

| 파일 | 현장 입력 |
|---|---|
| `site.json` | 좌우 로봇 IP, base→cell 좌표, 실제 tool/load 질량, Hand 설정, PLC 주소·신호 매핑, 승인된 현장 CSV |
| `camera.json` | 실제 시리얼, camera→cell 및 양팔 base 좌표, 측정 근거·시각·오차·대응점, 공급 기준 위치 |
| `hardware.json` | 실제 K, TCP, 지그 좌표, 보정·형상 기록 해시, 장치 도구 해시, 현재 장면 승인기 |
| `ring-task.json` | 링 작업의 소스·현장 설정·관측·경로 식별자와 실제 검증 기록 |

카메라 위치가 달라지면 `camera.json`의 `T_cell_camera`를 다시 측정합니다. 렌즈·해상도·스트림이 달라지면 실제 K도 갱신합니다. 로봇 위치나 지그 위치가 달라지면 해당 base/fixture 변환과 충돌 장면을 갱신합니다. 작업 정책을 고치기 위해 시뮬레이션의 카메라 위치를 맞춰 넣는 방식은 사용하지 않습니다.

측정값을 편집한 뒤 `capx-fr3 pin-files --config site-local/hardware.json`으로 파일 식별자를 갱신합니다. 이 명령은 입력 파일을 승인하지 않으며 매번 motion을 비활성화합니다. 자세한 과정은 [현장 연결 순서](docs/COMMISSIONING.md)를 확인하세요.

현재 카메라 adapter는 **설정한 단일 RealSense의 persistent stream**입니다. 시뮬레이션의 여러 고정 뷰를 실제 다중 카메라로 자동 구현하지 않습니다. 공급대·전달·지그·삽입 후 각 단계에서 실제 장착 카메라의 가시성을 새 사진으로 확인해야 합니다. 보이지 않는 단계는 실제 카메라/장착/보정 adapter를 추가하거나 작업 정책을 다시 검증해야 합니다.

## 실제 장치 코드

기존 검증 코드의 소스를 `capx_vendor/`에 해시를 유지해 포함했습니다. 실행 바이너리는 현장에서 빌드합니다.

```bash
cmake -S "$(capx-fr3 build-source)" -B ./build/fieldkit -DCMAKE_BUILD_TYPE=Release
cmake --build ./build/fieldkit --parallel 2
ctest --test-dir ./build/fieldkit --output-on-failure
capx-fr3 pin-files --config ./site-local/hardware.json --build-dir ./build/fieldkit
```

장치 주소와 설정을 채운 뒤에만 아래의 명시적인 읽기 명령을 실행합니다.

```bash
capx-hardware status --config ./site-local/hardware.json --side left --connect-read-only
capx-hardware camera --config ./site-local/hardware.json \
  --connect-read-only --output ./site-local/capture-001.npz
```

상태 읽기는 로봇·Hand 상태를 조회하며 이동을 명령하지 않습니다. 카메라는 실제 스트림을 시작해 데이터를 저장하며 종료 시 닫습니다. PLC·그리퍼·클램프·이동은 `hardware_v1.backend.HardwareBackend`가 기존 드라이버에 위임하며, 준비 CLI는 이들 구동 명령을 노출하지 않습니다.

## 시뮬레이션에서 가져오는 것과 현장에서 바꾸는 것

가져오는 것은 단계별 정책, 명시적인 양팔 API, 로그·복구·중단 구조, 관측과 실제 발행 명령을 연결하는 계약입니다. 실제 좌표·TCP·추종 편향·카메라·힘·마찰을 같은 값으로 가정하지 않습니다. 실기 요청에는 실제 현재 양팔 상태, Hand 폭, 최신 하드웨어 관측, 관측한 손↔부품 관계, 현장 목표가 들어갑니다.

cuRobo는 설치된 현장 모델에 맞춰 IK/경로 **후보**를 계산하는 연결 지점입니다. 본 배포본에 GPU 서버나 실측 장면 모델은 포함하지 않습니다. 후보는 현장 전체 충돌 검사와 실제 quintic 명령의 FR3 한도 검사를 통과해야 실행할 수 있습니다. 실시간 MPC를 제공한다고 표시하지 않습니다.

시뮬레이션의 480 Hz 배열을 CSV로 이름만 바꿔 보내지 않습니다. 현장 경로는 `time_s,q1..q7,dq1..dq7,ddq1..ddq7`의 22열 stopped-boundary 데이터로 만들고, 기존 C++ 실행기에서 1 kHz quintic 보간 전체를 검사합니다. CSV 파싱 성공은 충돌 승인이나 실기 성공이 아닙니다. 전체 연결은 [구조와 계약](docs/ARCHITECTURE.md)을 참조하세요.

## 검증 범위

`evidence/`에는 패키지 검증 결과, 원본 드라이버 소스 식별자, 보존된 시뮬레이션 결과의 요약을 둡니다. 기존 성공본과 원본 시험 디렉터리는 변경하지 않습니다. 성공 영상·대용량 시뮬레이터 자산·실제 장비 주소·개인 보정값은 이 배포에 넣지 않습니다.

실기 자동 운전을 위해 남은 핵심은 **현장 실측값, 실제 전체 장면 승인기, 해당 장비에서의 commissioning**입니다. 특히 시뮬레이션의 30 N/pad 설정을 현재 실제 Hand wrapper의 5–20 N 설정으로 그대로 쓰거나 조용히 잘라내지 않습니다. 실제 그리퍼 정책을 검증해야 합니다.

이번 버전은 실제 요청·I/O 연결 경계와 설치 도구를 제공합니다. 완성된 실기 자율 링 계획기 및 전체 단계 coordinator를 제공하거나 검증한 버전은 아닙니다. 남은 시뮬레이션 함수별 이식 지점은 [링 이식 표](docs/RING_HARDWARE_PORT_MAP_v1.md)에 정리했습니다.

