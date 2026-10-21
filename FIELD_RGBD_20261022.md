# FR3 Duo · native Franka Hand · RGB-D 현장 시험

현장 시험 기준 시각: **2026-10-22 오전 11:00 한국 시간**.

`field_demo_random_20261022/`는 Isaac Sim 없이 설치·실행할 수 있는 현장 패키지입니다. 혼합 링 10개에서 큰 링·작은 링·홀더의 핀을 관측하고 파지·전달·삽입 스킬을 조합합니다. 실행 중 LLM agent를 호출하지 않습니다. 입력은 RealSense RGB-D, 보정값, 실제 로봇/Hand SDK 상태입니다.

**현장 보정과 저속 단일 파지부터 시험하는 실험 코드입니다. 랜덤 바구니의 전체 실기 성공·20초 완료를 검증한 배포본은 아닙니다.** 실제 하중·자기충돌·다른 팔·고정 장애물·가려진 적재물·정지/통신 조건 검사를 유지합니다.

## 내려받기와 설치

과거 MuJoCo 아카이브와 SAM 체크포인트를 복원하지 않아도 됩니다. 새 설치는 현장 폴더만 내려받을 수 있습니다.

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone --filter=blob:none --sparse \
  https://github.com/KIM-JAKE/mujoco-transfer-20261018.git fr3-field
cd fr3-field
git sparse-checkout set field_demo_random_20261022
cd field_demo_random_20261022
```

Ubuntu 22.04 / x86_64 / Python 3.10 현장 PC에는 오프라인 wheel·SDK 소스와 설치 도구가 포함돼 있습니다. 컴파일러·make 등 호스트 조건과 다른 OS의 설치 방법은 [설치 안내](field_demo_random_20261022/docs/INSTALL.md)를 확인합니다.

```bash
scripts/install.sh --with-realsense --offline --wheelhouse vendor/wheelhouse/cp310-linux-x86_64
scripts/build-offline-ubuntu22.sh
./field-demo verify
```

[시작 안내](field_demo_random_20261022/START_HERE.md) → [모델 기반 설정](field_demo_random_20261022/docs/MODEL_STARTER.md) → [현장 실행 순서](field_demo_random_20261022/docs/RANDOM_ONSITE.md)를 따릅니다.

## 현장 설정

공식 FR3-v2 CAD와 실제 SDK 링크 자세로 팔 형상을 자동 작성하고, 실제 `F_T_EE`를 반영해 Hand·양면 파지 후보·링 64개 sector·핀·명목 CAD 하중을 설정합니다. 현재 SDK에서 읽은 Hand 폭과 오차 구간으로 손 형상을 구성하고, 손을 열고 닫는 전체 구간을 검사합니다. 손 동작 중에도 정지한 팔의 실제 SDK 힘·자세·시각을 감시합니다. 공장 Hand 장착 방향, 추가 어댑터 유무, 실제 모델/부품 일치, 케이블 여유를 **실제로 확인한 설치 기록 한 개**가 필요합니다. 모델에서 계산한 값은 `model_derived / verified:false`로 유지하며 실측값으로 위장하지 않습니다.

```bash
mkdir -p site
./field-demo model-preset template --output site/installation.json
# 실제 장비와 대조해 installation.json 및 증거 파일 작성
./build/m80_read_state LEFT_IP  > site/left-sdk-state.json
./build/m80_read_state RIGHT_IP > site/right-sdk-state.json
./field-demo init-random --output site/random \
  --left-ip LEFT_IP --right-ip RIGHT_IP \
  --model-installation site/installation.json \
  --left-state site/left-sdk-state.json --right-state site/right-sdk-state.json
```

양팔 base 좌표, 카메라 보정/깊이 오차, 바구니·지그·핀 홀더 좌표, 작업공간과 고정 장애물, 관측/반환 자세, STOP과 실제 클램프 설정은 현장 값이 필요합니다. 클램프는 실제 PLC 어댑터 또는 명시적인 작업자 조작을 사용합니다. 가상의 파지·삽입 PLC 신호를 요구하지 않습니다.

## 첫 실행

카메라 인식·읽기 전용 SDK 확인 후, 실제 여유 방향에서 각 팔의 **1mm 이동·정지 시험**을 먼저 수행합니다. 아래 `+z`는 로봇 BASE 좌표이며, 현장에서 움직여도 되는 방향을 선택합니다. 자동 복구/재시작을 하지 않습니다.

```bash
./field-demo doctor --config site/random/site.json --probe
./field-demo commissioning-test --config site/random/site.json --side right --axis z --direction 1 --execute
./field-demo commissioning-test --config site/random/site.json --side left --axis z --direction 1 --execute
./field-demo random-cycle --recipe site/random/random-cycle.json --single-pick ring2 --execute
```

실제 현장 입력과 정지 기록이 빠졌거나 관측·충돌·하중 검사가 실패하면 진행하지 않습니다. 단일 파지는 성공 시 물체를 든 채 정지합니다. 이후 전달·삽입·전체 사이클 순서와 중단/반환 절차는 현장 실행 안내에 있습니다.

## 검증 범위

- **현장 패키지:** 275 Python·27 fieldkit·11 native 검사 통과. 배포 파일 452개와 연결 의존성 91개를 별도 경로에서 검증했습니다. SDK 소스 빌드·모델/TCP/하중 연결 검사는 실기 조립 성공과 다릅니다.
- **단일 상부 시뮬 카메라:** 실제 PhysX 빈 손 주차 1.408초 후 RGB-D 한 장에서 크기·중심·축을 추정했습니다. 큰 링 접근·40/70mm 이탈 기하 구성요소가 통과했고, 손–바구니 최소 8.24mm, 링–측벽 3.03mm였습니다. 가려진 공간은 삭제하지 않았습니다. 기록은 `reports/one-overhead-camera-278/`에 있습니다.
- **GPU 연구 경로:** cuRobo 제안 0.256초·왕복 0.286초인 운반 질의 한 건이 원래 충돌 검사와 FR3 제한 검사, 추가 자기충돌 검사를 통과했습니다. 예상 이동시간 1.794초이며 이 정적 질의 자체는 로봇을 움직이지 않았습니다. LEFT IK도 원래 LEFT FK로 검증된 후보 10개를 약40ms에 받았습니다. LEFT 운반 질의는 178ms 제안과 0.892초 예상 이동시간으로 자체·물체·양팔·고정 충돌/실행 명령 검사를 통과했습니다. 이 정적 질의는 이전 설치 부품을 포함하지 않았고, 새 전체 실행에는 아래 보호 검사를 적용합니다.
- **전체 물리 작업:** 최신 무작위 시험은 파지·전달·삽입·철수 문제를 수정하며 진행 중입니다. 한 시험에서 핀/큰 링 유지와 작은 링 삽입 깊이를 확인했으나 마지막 철수 충돌로 전체 실패를 기록했습니다. 임의 80mm 하강을 실제 삽입 전 위치로 되돌리는 수정은 289개 실행 명령 검사에 통과했고 실제 전체 재시험 중입니다.

시뮬 연구 소스·실패 기록은 [simulation_checkpoint_20261022](simulation_checkpoint_20261022/README.md)에 있습니다. 이 폴더만으로 전체 Isaac 장면이 복원된다는 뜻은 아닙니다. 현장 실행기는 접근할 수 없는 후보를 제외하고 같은 크기의 다른 링 후보를 제한된 횟수로 선택합니다. 손이 영상에 나타나는 경우 자체 형상을 적재물로 오인하는 문제는 현재 보완 중이며, 가려진 공간은 그대로 장애물로 유지합니다. 자세한 변경과 한계는 [폭에 따른 손 형상 안내](field_demo_random_20261022/docs/APERTURE_ADAPTIVE.md)에 있습니다. 현장 실행기는 측정값과 현재 영상·실제 SDK로 경로를 다시 만들며, 과거 시뮬 관절 명령을 그대로 재생하지 않습니다. 기존 봉인 배포본은 보존했습니다.

최신 후속 시험에서는 세 삽입 스킬을 실행한 뒤 다음 접근 중 기존 큰 링을 건드려 최종 유지가 실패했습니다. 조립이 끝난 부품의 실제 RGB-D 관측 자세를 양팔 공통 장애물로 등록하는 수정을 추가했습니다. 기록된 충돌 전 접근을 차단하고 정상 해제 자세를 통과하는 정적 회귀 검사를 완료했으며, 전체 재시험 중입니다. 이 결과도 전체 성공으로 표기하지 않습니다.
