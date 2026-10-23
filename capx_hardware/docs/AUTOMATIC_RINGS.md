# 자동 SMALL → LARGE 실행기

`capx-rings`는 실제 현장용 링 정책을 포함합니다. 기존 workspace의
`field_demo_random_20261022`에서 RGB-D 인식, 파지 후보/IK 선택, SMALL 양팔 전달,
현재 측정 G를 이용한 운반/삽입, PLC 클램프, 실제 Hand 놓기, 철수와 새 관측을
이식했습니다. 단계별 빈 callback을 현장에서 작성해야 하는 패키지가 아닙니다.

기존 배포 commit `86c6e906242b4b6839bbb42cce89736989684fcc`, 모든 시뮬 성공/실패,
현장 설정과 관측 원본은 보존합니다. 새 실행기는 새 소스 버전입니다. 기존 v090/v100
시뮬 성공률을 이 현장 정책의 검증 결과로 승계하지 않습니다. 기존 실기 정책의 더
엄격한 수치 제한, 10% native 속도/가속도 범위와 5–20N Hand 제한을 유지합니다.
30N 시뮬 파지를 clip하거나 같은 물리 정책이라고 부르지 않습니다.

## 설치와 장치 무동작 확인

```sh
# 현장 checkout에서 같은 브랜치를 갱신한 뒤
python -m pip install './capx_hardware[autoring,camera]'

# 기존 site-local 안에 새 파일만 추가. 기존 파일이 있으면 중단합니다.
capx-rings init ./site-local --hardware hardware.json

# 정책/프레임/장치/실측 증거를 검토한 뒤 현재 파일의 해시만 고정
capx-rings pin-files --config ./site-local/autoring.json
capx-rings preflight --config ./site-local/autoring.json

# run 기본값 역시 정적 검사만 합니다. 카메라/SDK/PLC에 연결하지 않습니다.
capx-rings run --config ./site-local/autoring.json

# 실제 production 상태흐름 + 명시적 모의 센서/로봇/Hand/PLC
capx-rings self-test --output ./NEW-offline-check
```

`self-test`는 원래 production `run` 함수를 실행하며 SMALL 8단계, LARGE 8단계,
새 두 링 final을 검사합니다. 결과는 `offline_contract`,
`accepted_ring_pair:false`, `accepted_full3:false`입니다. 모의 센서와 planner는
실기 인식/경로/접촉의 증거가 아닙니다. 기존 현장 설정 폴더에서 `init`이 기존
이름과 충돌하면 새 폴더를 사용하고 hardware.json을 상대 경로로 참조하세요.

실제 저장된 RGB-D를 원래 detector로 재생할 수도 있습니다:

```sh
capx-rings replay --recipe ./EXPLICIT-replay.recipe.json \
  --manifest ./recorded-frames.json --output ./NEW-recorded-replay
```

recipe는 `replay_only:true`, camera는 명시적 replay 보정이어야 합니다. 이 경로도
실제 구동이나 승인으로 변환되지 않습니다. 실제 live recipe를 replay로 몰래
바꾸거나 과거 영상을 새 관측으로 재시각화하지 않습니다.

## 현장에서 채울 파일

새 `rings.recipe.json`, `rings.camera.json`, `rings.planner.json`,
`rings.payload.json`은 원래 현장 스키마의 NULL/미검증 템플릿입니다.

- 기존 `hardware.json`/`site.json`: 실제 IP, 원본 native binary/Hand, PLC 주소·극성,
  tool/TCP, 전체 장면/케이블 측정, stop/commissioning 증거. 기존 값을 덮어쓰지 않습니다.
- recipe: SMALL `ring2` 다음 LARGE `ring1`, 실제 pickup/insert arm, 측정 파지 families,
  고정 world PART goal, 측정 접근/철수 corridor, 원래 허용범위 안의 gate·예산.
- camera: 측정 camera serial/K/해상도/fps/T, 양팔 공통 cell 좌표, basket ROI/깊이/잔여
  occupancy 설정. `camera_profile`은 backend K/size/fps와 정확히 같아야 합니다.
  camera/base/TCP 기준점과 실제 접촉 datum을 혼합하지 않습니다.
- planner: 실측 whole robot/Hand/tool/cable enclosure, 설치물/자유 부품/미관측 공간,
  현재 양팔·개구폭과 payload 검사. 비어 있는 설비를 CLEAR로 취급하지 않습니다.
- payload: 별도 tool과 부품의 실측 질량/COM/관성, stopped setLoad/readback binary.
  Hand 질량을 두 번 더하지 않습니다. PIN 동작/질량 입력은 두 링 실행에 필요 없습니다.
- `autoring.json`: 현재 소스와 위 파일들의 해시. pin-files는 verified나 motion_enabled를
  바꾸지 않습니다. 실제 측정·승인 대신 해시만 입력해서 통과할 수 없습니다.

기존 schema1 hardware camera 설정과 schema2 ring camera 설정은 목적이 다릅니다.
측정 serial/T/base/K가 일치하는지 preflight가 검사합니다. 새로 보정하라는 뜻이
아니며, 기존 측정값과 그 증거를 이어서 명시적으로 이식해야 합니다.

## 실제 연결과 구동 차단

현재의 전체 장면 승인기가 없는 현장은 **실행 코드가 포함되어 있어도 구동이
차단**됩니다. `run --execute --output NEW_DIRECTORY`만 live 경로이며, 이 옵션도
완전한 preflight와 기존 `motion_enabled`/commissioning/보정/장면 조건을 대체하지
못합니다. 이번 배포 검증에서는 이 live 경로로 장치를 구동하지 않았습니다.

연결은 다음과 같습니다:

1. 실제 RealSense persistent stream → 원래 signed annular detector/원래 CAD gate.
2. 실제 양팔 SDK q/q_d/실측 TCP + Hand 폭 → 원래 native measured-model IK/C2 계획.
3. 원래 1kHz quintic 모델/보수 연속 geometry/FR3 검사 → 새 immutable CSV.
4. 현재 actual/command 양팔·개구폭·fresh raw/G·고정 PART goal·source/site/CSV를
   `capx-hardware-autoring-admission/v1` 요청으로 전체 현장 승인기에 전달.
5. exact 요청 digest, 모든 coverage bool, 2초 이내 승인과 상태/폭 불변 확인 후,
   원래 `Site.motion`의 dynamic 재검사·cell lock·PLC·TCP·payload·native watchdog 실행.
6. Hand 전체 aperture/클램프 지원 전이/stopped payload 변경도 각각 새 승인을 요구.
   STOP은 기존 독립 경로를 유지합니다. 실패 뒤 자동 opening/unclamp/home/resume 없음.

기존 `capx-ring-task`의 taught-CSV 요청 API도 그대로 유지합니다. 자동 정책은 새 CSV가
매번 생성되므로 원래 dynamic-field 실행 경로를 사용하며,
`field_path_reviewed:false`를 유지합니다. 자동 경로를 사람이 가르친 경로로 위장해
기존 등록 검사를 통과시키지 않습니다.

승인기는 `capx-hardware-autoring-approval/v1`, domain hardware, `approved:true`,
exact `request_sha256`와 다음 **모든 exact true**가 있어야 합니다:
`full_world`, `both_arms`, `hands`, `payload`, `installed_and_free_parts`,
`measured_frames`, `actual_command_bridge`, `original_FR3_limits`,
`continuous_interpolant`, `continuous_aperture`, `supported_release`.
이 배포는 그런 승인기나 현장 검증 결과를 만들어 넣지 않습니다.

## cuRobo 후보의 위치

기본값은 포함된 실제 native measured-model planner입니다. 기존 cuRobo를 현장의
실측 모델로 연결하려면 `rings.planner.json`의 optional `cuRobo_proposal`에 실제
client executable/model-record 해시와 <=2초 timeout을 명시합니다. 기본 NULL은
GPU 요청을 하지 않습니다. native 후보가 거부됐을 때 한 번만 후보를 요청합니다.

client는 stdin JSON `capx-current-curobo-proposal/v1`을 받으며 현재 SDK/Hand,
실측 모델 해시, 원래 필수 TCP 목표들이 포함됩니다. reply는
`capx-current-curobo-proposal-reply/v1`, exact request/model digest,
각 원래 목표에 대응하는 `segments`(현재 시작과 원래 끝을 포함한 2..33 SE3), optional
native 7-joint `ik_seed`입니다. 전체 새 targets는 원래 native 제한 32개 이하여야 합니다.

GPU 응답은 새 native IK/C2/geometry 검사를 다시 통과해야 합니다. 원래 필수 목표를
생략하거나 old simulation q/goal/gauge를 실기 목표로 사용할 수 없습니다. 기존
시뮬용 cuRobo daemon을 실측 모델 재구축 없이 실기 서비스라고 표시하지 않습니다.
실측 장면/모델/서비스가 아직 없다면 이 옵션을 NULL로 두고 native 계획기를 씁니다.

## 기록·실패·검증 범위

각 새 output에 `native-ring-pair-session-final.json`, run/action token,
16 real phase 기록(또는 거부 전 partial), source/config SHA, 기존 세부 driver/raw/CSV
기록과 마지막 LARGE 이후 NEW BOTH 영상이 남습니다. ring pair는 full3로 기록되지
않고 component나 설치된 SMALL을 실제 SMALL 수행으로 대체하지 않습니다.
실기 완료를 기록하는 경우에도 반복 안정성/하드웨어 qualification을 자동 선언하지
않습니다. 기존 원본과 실패 분모를 덮어쓰지 않습니다.

코드 provenance는 `evidence/autoring-upstream-port.json`, 설치 소스/자산 seal은
`capx_autoring/engine-manifest.json`입니다. package relocation과 실제 설치본에서
비구동 tests를 실행합니다. SDK 전체 build와 실제 기구/접촉 안정성은 현장 검증입니다.
