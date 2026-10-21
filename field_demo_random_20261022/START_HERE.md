> **현재 추가 기능:** [실제 Hand 폭·후보 재선택](docs/APERTURE_ADAPTIVE.md). 새 설정 작성과 재빌드가 필요하며 실제 SDK 값으로 검사합니다.

> **새 현장 설정:** [모델 기반 starter](docs/MODEL_STARTER.md)에서 링크·Hand·링·핀 충돌 모양을 자동 작성합니다. 실제 설치 일치 기록 한 개와 읽기 전용 SDK 상태를 사용하며 base/카메라/바구니/지그/STOP은 현장 값으로 유지합니다.

# 10월 22일 오전 11시 KST 현장 시험용 — RGB-D + Franka SDK

현재 실행과 설정은 [RGB-D + SDK 온보딩](integration/RGBD_SDK_ONBOARDING.md)을 먼저 읽는다. 실제 카메라 인식·현재 SDK 상태 기반 경로·저속 단품 집기와 전체 공정 실행 경로를 구현했다. 실제 로봇 성공이나 랜덤 바구니 전체 완주를 검증한 버전은 아니다. [구현 범위·하중·실패 처리](integration/PAYLOAD_EXPERIMENTAL.md)와 최신 검사 결과를 함께 확인한다.

원본 봉인 패키지는 보존했다. 아래 v128 자료는 이전 시뮬 참조/설치 배경이며 현장 실행 상태는 위 최신 문서를 기준으로 한다. GPU/cuRobo 최적화 최신 시뮬은 H 작업에 있으며 이 패키지의 실기 경로 생성기는 native libfranka 모델과 보수적 외피 검사다. 시뮬 CSV를 로봇에서 재생하지 않는다.

---

# 10월 22일 실기 준비 — v128 성공본 연결과 카메라 설치

현장 제어 PC에 이 폴더 전체를 복사합니다. 기존 안정화 작업과 이전 `field-demo-20261022`, `field-demo-random-20261022` 패키지는 수정하지 않았습니다. v119 52.30초 성공 자료도 그대로 보존합니다.

새 실행 경로는 **RGB-D 무작위 링 검출 → 현재 SDK 상태로 집기 경로 생성·검사 → 파지 후 재관측 → 필요한 양손 전달 → 재관측으로 삽입 목표 보정 → 안착·클램프 → 개방·철수 → 세 부품 유지 확인**입니다. 큰 링·작은 링은 초기 물체 위치를 입력하지 않습니다. 핀은 별도 홀더의 실측 위치를 탐색 범위로 사용합니다.

실물은 아직 연결하지 않았습니다. 내일 좌표 보정·손/부품 치수·현장 충돌 영역·PLC 신호를 측정하고 단품 동작을 확인해야 합니다. 실행 코드를 준비한 상태이며, 현장 무조정 완주나 30초 미만을 검증한 상태는 아닙니다.

## 먼저 할 일

```bash
chmod +x field-demo scripts/*.sh
./field-demo verify
./field-demo doctor --output reports/arrival.json
```

Ubuntu 22.04 / x86_64 / Python 3.10, 컴파일러와 make가 있는 PC의 오프라인 설치:

```bash
scripts/install.sh --with-realsense --offline --wheelhouse vendor/wheelhouse/cp310-linux-x86_64
scripts/build-offline-ubuntu22.sh
scripts/offline-check.sh
./field-demo init-random --output site/random
```

다른 환경은 [설치 안내](docs/INSTALL.md)를 따릅니다. 미확인 현장값은 `null`로 생성되며, 값을 측정하기 전에는 실행되지 않습니다.

## 내일 열어 둘 문서

**[현장 실행 순서와 명령](docs/RANDOM_ONSITE.md)**을 위에서부터 따릅니다. 먼저 **[이미지로 보는 카메라 설치 가이드](docs/CAMERA_INSTALLATION.html)** ([Markdown](docs/CAMERA_INSTALLATION.md))에서 바구니·전달·지그·핀 홀더 시야를 확인합니다. **[v128 연결 방법과 적용 범위](docs/V128_CONNECTION.md)**에는 선택한 성공본과 현장 실행기의 관계를 정리했습니다. 기존 고정 경로 도구는 [기존 연결·단품 절차](docs/ONSITE_SEQUENCE.md), 문제 대응은 [중단·문제 해결](docs/TROUBLESHOOTING.md)에 있습니다.

장비 없이 인식만 확인하는 명령:

```bash
./field-demo random-vision detect --replay \
  --snapshot stabilization/replay-frames/observation-001-initial.npz \
  --config src/field_demo/stabilized/replay-camera.json \
  --output reports/random-recognition-replay.json
```

이 명령은 보관된 시뮬 RGB-D를 검출기에 넣습니다. 로봇에 연결하거나 움직이지 않습니다. 전체 실행기는 `random-cycle --help`로 확인합니다.

실제 검출기와 전체 단계 순서를 함께 확인하는 합성 영상 재생도 포함했습니다. 로봇·손·PLC와 경로 계획은 가상 장치입니다.

```bash
./field-demo random-cycle --recipe replay/synthetic-contract/recipe-v128.json \
  --dry-run --replay-manifest replay/synthetic-contract/manifest.json \
  --output-dir reports/synthetic-replay
```

이 예제의 24개 영상은 시험용 합성 영상입니다. 저쪽 성공 run의 전체 영상 기록이나 실제 조립 시험으로 해석하지 않습니다.

## 저쪽 안정화 결과 연결

해시를 확인한 **v128 43.533초 성공본**을 `stabilization/v128`에 추가하고, 기존 `stabilization/v119`를 보존했습니다. 인식 코어가 같은 것을 확인했고, 새 `init-random` 설정은 v128 프로필과 실행 엔진을 해시로 연결합니다. 프로필이 바뀌거나 실행 코드가 달라지면 재검증해야 합니다.

```bash
./field-demo stabilization check --profile stabilization/v128/field-profile.json
./field-demo random-vision detect --replay \
  --snapshot stabilization/v128/replay-frames/observation-001-initial.npz \
  --config stabilization/v128/replay-camera.json \
  --output reports/v128-recognition-replay.json
```

43.533초는 seed445의 시뮬 벽시계 시간입니다. 실기에서는 실측 좌표·현재 영상·실제 SDK로 경로를 다시 만들며, 기존 실기 실행기의 저속·순차 제약을 유지합니다. 시뮬의 관절 궤적·전달 좌표·힘·속도를 그대로 실행하지 않습니다. 근거리 전달 분기 탐색·시간 배분·동시 작업 최적화 전체를 실기에 포팅했다는 뜻도 아닙니다. 새 성공 자료는 `stabilization inspect/import`로 별도 폴더에 읽어 올 수 있고, 자동으로 현재 프로필을 바꾸지 않습니다.

검증 범위는 [기능과 남은 현장 확인](docs/CAPABILITIES.md), 시험 결과는 `reports/release-validation.json`에 기록합니다.

추가 실험 기능: [자기 Hand RGB-D 표면 연결](docs/KNOWN_HAND_SURFACE.md). 기본 비활성, 실제 in-bin 파지 통과 미확인입니다.
