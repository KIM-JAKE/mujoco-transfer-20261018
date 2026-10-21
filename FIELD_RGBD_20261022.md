# FR3 Duo · native Franka Hand · RGB-D 현장 시험

현장 시험 기준 시각: **2026-10-22 오전 11:00 한국 시간**.

`field_demo_random_20261022/`는 Isaac Sim 없이 설치·실행할 수 있는 현장 패키지입니다. 혼합 링 10개에서 큰 링·작은 링·홀더의 핀을 관측하고 파지·전달·삽입 스킬을 조합합니다. 실행 중 LLM agent를 호출하지 않습니다. 입력은 RealSense RGB-D, 보정값, 실제 로봇/Hand SDK 상태입니다.

**현장 보정과 저속 단일 파지부터 시험하는 실험 코드입니다. 랜덤 바구니의 전체 실기 성공·20초 완료를 검증한 배포본은 아닙니다.** 실제 하중·자기충돌·다른 팔·고정 장애물·가려진 적재물·정지/통신 조건 검사를 유지합니다.

최신 Isaac 시험404는 실제 상대 팔 자세와 실행 명령 전체를 검사하는 수정 후, 기존 배치446에서 링 두 개·핀 하나의 삽입·유지·최종 철수를 통과했습니다. 촬영·인식·계산 포함590.63초, 물리 시간67.67초이며 다른 배치의 반복 성공·실기·20초 달성은 아직 미검증입니다. [정확한 코드와 검증 범위](simulation_checkpoint_20261022/experiments/full-pass404/README.md).

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

## 현재 검증 범위와 남은 문제

- 새 배포본은 **596개 파일·117개 연결 의존성**을 다른 경로에서 검증했습니다. 핀 피드백 추가분과 관련 사이클 Python 검사 21개, native 재빌드와 C++ 검사 12개를 통과했습니다. 장비 연결은 하지 않았습니다. 새 후보 순서 변경은 native C++ 소스를 바꾸지 않았습니다.
- 고정 장애물에 대해서도 원본 링크·palm 전체 convex hull을 사용하는 선택 검사를 추가했습니다. 기존 케이블·추종·시간·충돌 여유와 0.5mm를 모두 적용하고, 동일한 전체 장애물 상자를 통과해야 합니다. UNKNOWN을 지우거나 손가락·부품 검사를 완화하지 않습니다. 기본값은 꺼짐이며 관련 검사 59개와 596파일 복사본 검증을 통과했습니다. 기록된 후보는 여전히 전체 접근을 통과하지 못했습니다.
- 자기 충돌 모형의 오검출을 줄이는 원본 STL convex hull 검사도 선택 기능으로 추가했습니다. 링크와 고정 palm 전체를 둘러싸는 형상을 사용하며 기존 케이블·추종·시간·충돌 여유에 부품마다 0.5mm를 더합니다. 손가락·부품·상대 팔·바구니 UNKNOWN 검사는 유지합니다. 기본값은 꺼짐이며 관련 검사 39개와 복사본 검증을 통과했습니다. 실제 기록의 두 자기 충돌 오검출을 해소했지만 현장 전체 경로를 통과한 것은 아닙니다.
- 공식 듀오의 같은 프레임에 고정된 베이스 부품을 팔끼리 충돌로 오인하는 경우를 처리하는 선택 기능을 추가했습니다. 실제 공통 마운트·모델·베이스 상대좌표를 확인한 기록이 있어야 켤 수 있으며 기본값은 꺼짐입니다. 움직이는 팔·손·부품·미관측 공간 검사는 유지됩니다. 관련 검사를 포함해 31개 검사를 통과했으나, 실제 기록을 대신 만들거나 현장 경로를 통과 처리하지 않았습니다.
- 파지 후보를 링별로 공평하게 시도합니다. 기존 코드는 앞쪽 링들의 여러 IK 시도에 탐색 횟수를 써서 뒤쪽의 잡을 수 있는 링까지 도달하지 못할 수 있었습니다. 5개 링·4개 IK 시드·16회 예산에서 새 코드는 다섯 번째 링을 5번째 시도에 검사합니다. 관련 검사와 기존 사이클·단일 파지 검사 22개를 통과했고, 미관측 공간·충돌·정지 기준은 유지했습니다. 실제 현장 성공을 검증한 것은 아닙니다.
- [선택적 핀 정렬 피드백](field_demo_random_20261022/docs/PIN_PRECISION.md)을 추가했습니다. 실제 SDK TCP와 같은 상태의 FK, 새 RGB-D로 중심·축을 보정하고 명령 직전에도 정밀 기준을 다시 확인합니다. 기본 비활성이며 현장 ICP의 정밀도·수렴은 미검증입니다. 기록된 시뮬의 보정식 재생 통과를 실기 성공으로 취급하지 않습니다.
- [핀 원형 끝면 인식](field_demo_random_20261022/docs/PIN_CAP_RGBD.md)을 선택할 수 있습니다. 실제 RGB·Depth 픽셀에서 원과 축을 맞추며 가림이 심하면 이전 좌표로 대신하지 않고 거부합니다. 원본 시뮬 이미지 4장에서 기존 픽셀 추정과 일치했고 새 품질·기본 인식 회귀 검사 7개를 통과했습니다. 기본 CAD ICP는 유지합니다. 시뮬의 여러 가상 시야 재생은 실제 고정 카메라의 시야·금속 깊이 정확도를 검증한 결과가 아닙니다.
- 실제 시뮬 RGB-D 데이터에서 4mm 바구니 지도가 기본 용량을 넘는 문제를 수정했습니다. 선택 물체의 CAD 검색을 필요한 점에만 적용해 같은 분류 결과를 유지하면서 해당 계산을 약 3.15배 줄였습니다.
- [5mrad 저속 시운전 프로파일](field_demo_random_20261022/docs/LOW_SPEED_COMMISSIONING.md)은 선택 사항입니다. 경로 검사·native SDK 실행·Hand 감시에 같은 오차 상한을 사용하며, 실제 양팔 이동·정지 기록을 요구합니다. 기본 20mrad 설정과 기존 배포본은 보존했습니다. 실제 추종 성능을 이미 측정했다는 뜻은 아닙니다.
- 읽기 전용 IK 시드 대안을 추가했습니다. 실행 경로의 시작은 계속 실제 SDK에서 읽은 `q_d`이며, 시드를 실제 관절 상태로 대입하지 않습니다.
- 오프라인 분석 도구의 과거 손가락 관절 인덱스 혼동을 수정해 다시 검사했습니다. 실제 SDK의 팔별 Hand 폭 입력은 영향을 받지 않습니다. 수정한 폭에서도 기본 48후보는 44개 형상·4개 IK 거부로 끝났고, 첫 차단 원인은 폭과 무관한 고정 팔 링크였습니다. 연속 각도 후보 16개도 아직 끝 자세를 통과하지 못했습니다. 과거 분석은 원본을 보존하고 [보정 기록](simulation_checkpoint_20261022/component-evidence/field-correct-width596/README.md)을 덧붙였습니다.
- **현장 파지 전체 흐름은 아직 통과하지 못했습니다.** 새 수직 RGB-D와 공정한 후보 순서에서도 큰 링 네 개를 모두 시도했지만 24개 후보가 UNKNOWN/팔·손 및 IK 조건에서 거부됐습니다. 2mrad 민감도·25도 파지 추가의 제한된 오프라인 분석도 실행 가능한 경로를 확보하지 못했습니다. 실행 설정은 바꾸지 않았습니다.  기록 RGB-D에서 물체를 찾았지만, 24개 후보의 팔·손과 미관측 공간 검사에서 모두 중단됐습니다. 가상 상부·전체 카메라 두 뷰를 합쳐도 남는 UNKNOWN 셀과 충돌합니다. 두 뷰 도구는 오프라인 구성요소이며 실제 다중 카메라 지도 유지 기능이나 실기 성공을 뜻하지 않습니다. 이 시험에서는 접근·Hand 명령을 보내지 않았습니다.
- **후속 Isaac 물리 시험 374:** 같은 배치 446에서 세 부품 삽입·유지·최종 철수를 다시 통과했습니다. 전체 **504.04초**, 물리 시간 48.98초입니다. 계산을 줄이는 FK·질의 갱신 변경을 포함하지만 병렬 시험이므로 전체 속도 개선을 입증하지 않습니다. 447·448 새 배치는 여전히 실패했고 실기·20초 완료는 미검증입니다. [정확한 코드와 범위](simulation_checkpoint_20261022/experiments/full-pass374/README.md).
- **후속 Isaac 물리 시험 357:** 같은 랜덤 배치 446에서 세 부품 삽입·유지·손 철수를 다시 통과했습니다. 전체 **470.50초**, 물리 시간 59.07초이며 작은 링은 두 번 정리했습니다. 큰 링 운반은 같은 경로의 불필요한 정지를 없애 실제 1.87초로 줄였고 기존 FR3·그리퍼 제한을 지켰습니다. 다른 배치·실기·20초 달성은 미검증입니다. [정확한 코드와 범위](simulation_checkpoint_20261022/experiments/full-pass357/README.md).
- **이전 Isaac 물리 시험 348:** 랜덤 배치 446에서 링 두 개·핀 하나 삽입, 고정 유지, 최종 손 철수까지 전체 판정을 통과했습니다. 촬영·인식·계산·동작 포함 **656.96초**, 물리 시간 85.35초이며 작은 링은 세 번 정리한 뒤 잡았습니다. 다른 랜덤 배치 449는 파지 전 중단됐습니다. [성공 기록 영상](simulation_checkpoint_20261022/media/all3-pass348-recorded-excerpts-1x.mp4)은 동작 1배속·대기 CUT·전체 원본 시간을 표시합니다. 실기·반복 일반화·20초 완료는 미검증입니다.
- **이전 Isaac 물리 시험 331:** 랜덤 배치에서 링 두 개와 핀 하나를 삽입하고 마지막 부품 위치·자세 및 유지 기준을 통과했습니다. 전체 판정은 빈 오른손의 최종 철수 거리 154.6mm가 기준 160mm에 못 미쳐 미통과입니다. 촬영·인식·계산·동작을 포함해 801.38초가 걸렸고, 수정 후 반복 성공과 실제 장비 동작은 아직 검증 중입니다. [실제 기록 영상](simulation_checkpoint_20261022/media/all3-inserted331-recorded-excerpts-1x.mp4)은 동작 1배속·대기 구간 CUT·전체 판정 미통과를 표시합니다.
- **Isaac 물리 시험:** 304에서는 링 두 개 삽입 후 핀 정렬이 실패했습니다. 수정한 308은 핀 단독 삽입·해제·철수에 성공했습니다. 합친 312는 큰 링 정렬에서 중단됐습니다. 319에서는 한쪽 테두리를 잡아 바구니 밖까지 약 132mm 들어 올리고 다시 내려놓았지만, 해당 파지의 일반 접촉 인증은 FALSE이며 조립/실기 적용이 검증된 것은 아닙니다. 348의 한 배치 전체 물리 성공과 달리, 실기·반복 일반화·20초 완료는 검증하지 못했습니다.

[현장 시작 안내](field_demo_random_20261022/START_HERE.md)와 [현재 기록](field_demo_random_20261022/reports/recorded-closure-release-check.json)에 실제 완료 범위가 있습니다. 카메라·설비·로봇 보정 후 읽기 전용 확인 → 1mm 이동·STOP → 저속 단일 파지 순서로 시작합니다. 현재 충돌/UNKNOWN 사유는 현장 영상과 좌표·접근 경로를 맞추기 위한 진단 자료로 남겨 두었습니다.

연구 소스·구성요소 영상·실패 기록은 [Isaac checkpoint](simulation_checkpoint_20261022/README.md)에 있습니다. checkpoint만으로 전체 Isaac 장면이 복원되는 것은 아닙니다. [짧은 지도 유지](field_demo_random_20261022/docs/STATIC_SCENE_LEASE.md), [알려진 손 표면](field_demo_random_20261022/docs/KNOWN_HAND_SURFACE.md), [현재 Hand 폭](field_demo_random_20261022/docs/APERTURE_ADAPTIVE.md) 검사는 유지합니다. 실행 중 agent 호출이나 시뮬 내부 물체 좌표를 쓰지 않으며, 실제 RGB-D·현재 SDK 상태로 다시 계획합니다.
