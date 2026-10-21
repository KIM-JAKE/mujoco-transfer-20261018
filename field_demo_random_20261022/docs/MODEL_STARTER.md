# FR3-v2 + native Franka Hand: 모델 기반 현장 시작 설정

링크 반경 16개와 Hand·링·핀 모양을 현장에서 직접 재지 않아도 됩니다. `model-preset`이 고정된 공식 FR3-v2 CAD, native Hand CAD, 원본 M80 부품 CAD에서 보수적인 충돌 모양을 만듭니다. 실제 로봇의 읽기 전용 SDK 상태에 있는 `F_T_EE`를 사용하므로 Desk TCP를 임의로 103.4mm라고 가정하지 않습니다.

이 기능은 **현장용 설정 작성 도구**입니다. 공장 모델과 실제 설치의 일치를 한 번 확인한 기록이 필요합니다. 만들어진 값은 계속 `kind: model_derived`, `verified: false`로 남습니다. 현장 동작 성공, 실제 마찰·파지 안정성, 실제 측정 관성이라는 뜻이 아닙니다.

## 1. 설치 일치 기록 한 개

```bash
./field-demo model-preset template --output site/installation.json
```

`site/installation.json`의 `expected`를 참고하여 `facts`를 실제 장비와 대조해 작성합니다. 확인할 것은 FR3-v2 모델, 기본 native Franka Hand, 공장 플랜지 장착 방향 −45°, 추가 어댑터 유무, 케이블을 감싸는 여유 두께 하나, M80 링·핀 CAD 일치와 명목 CAD 하중 사용입니다. 실제 확인자·시각과 사진/메모 파일을 기록한 뒤에만 `verified: true`로 바꿉니다. 케이블 여유를 모르면 0으로 채우지 않습니다. 기본 모델에 포함되지 않는 카메라 브래킷/어댑터/툴이 붙어 있으면 이 기본 프리셋을 그대로 사용하지 않습니다.

부품 하중은 큰 링 102.8278g, 작은 링 71.7855g, 핀 12.7874g의 기존 명목 CAD 모델을 사용합니다. 실제로 저울·관성을 측정했다는 기록을 만들지 않습니다. 파지 후보도 실제 TCP 원점을 부품 중심에 고정하지 않고, 공식 Hand의 명목 접촉 기준점을 실제 `F_T_EE`를 통해 변환합니다. 앞뒤 면마다 접근 방향을 따로 계산합니다. Hand는 별도 물체 하중에 더하지 않으며 SDK에 실제 설정된 tool mass/COM/inertia를 복사합니다.

## 2. 읽기 전용 SDK 상태 저장 후 생성

빌드가 끝난 현장 PC에서 양팔을 정지시키고, 빈 Hand 상태로 실행합니다. 이 명령은 이동·파지·자동 복구를 하지 않습니다.

```bash
mkdir -p site
./build/m80_read_state LEFT_IP  > site/left-sdk-state.json
./build/m80_read_state RIGHT_IP > site/right-sdk-state.json
./field-demo init-random --output site/random \
  --left-ip LEFT_IP --right-ip RIGHT_IP \
  --model-installation site/installation.json \
  --left-state site/left-sdk-state.json --right-state site/right-sdk-state.json
```

이미 빈 템플릿을 만든 경우에는 새 `init-random` 대신 다음을 사용합니다. 기존 `model-preset.json`은 덮어쓰지 않습니다.

```bash
./field-demo model-preset author --site site/random \
  --installation site/installation.json \
  --left-state site/left-sdk-state.json --right-state site/right-sdk-state.json
```

SDK의 tool 질량이 0이거나 기존 물체 하중이 남아 있으면 거부합니다. tool 하중을 추측해서 채우거나 실제 로봇 설정을 자동 변경하지 않습니다. Desk TCP 또는 설치 기록을 변경한 후에는 새 현장 디렉터리에 다시 생성합니다.

## 3. 자동으로 채워지는 것

- 공식 CAD 각 링크를 삼각형 단위로 분할해 감싸는 478개 column capsule. 실제 SDK의 joint1…7 자세로 변환하며, 0 길이 관절 구간을 200mm짜리 공으로 간주하지 않습니다.
- 공장 Hand 장착 방향과 **실제 SDK TCP**에서 변환한 palm·finger 모양. 0–80mm 전체 개방 범위를 보수적으로 감싸므로 실제보다 큰 간섭 판정이 생길 수 있습니다.
- 원본 큰/작은 링의 64개 sector씩과 핀 4개 CAD 조각을 감싸는 모양. 링 구멍을 통짜 원기둥으로 채우지 않습니다.
- 명목 CAD 물체 하중과 SDK에 설정된 tool/빈 물체 하중을 분리한 payload profile.
- 외측을 닫아 잡는 native Hand 앞뒤 면 × yaw 후보 자세 8개와 저속 그리퍼 동작 값. **파지 검증된 자세가 아니라 후보**이며 실제 RGB-D 관측·충돌 검사·파지 후 확인을 통과해야 합니다. 내부 확장 파지를 SDK가 지원한다고 가정하지 않습니다.
- commissioning용 알고리즘 예산. 속도/힘 제한, 실제 SDK wrench·heartbeat·STOP, residual UNKNOWN 장애물, 양팔 충돌 검사를 해제하지 않습니다.

같은 강체 내부와 바로 연결된 관절 사이만 기구 연결로 처리합니다. link5–link7처럼 한 관절을 건너뛴 같은 팔의 충돌은 계속 검사합니다. 전체 실행 곡선에서 실제 FK 모양과 추종 오차 한도를 포함하며, swept AABB가 분리된 쌍만 빠르게 제외합니다.

## 4. 현장에서 여전히 넣어야 하는 값

`docs/RANDOM_ONSITE.md`의 순서를 따릅니다. 실제 로봇 IP, 양팔 base 좌표, 카메라 보정·깊이 오차, 바구니 내부와 공급/지그 목표 좌표, 작업공간과 고정 장애물, 확인 가능한 촬영/반환 자세, 실제 clamp 방식은 현장 값이어야 합니다. 모델 프리셋이 사진 한 장에서 이런 좌표를 만들어내지 않습니다.

작업공간을 한 번 측정한 JSON(`T_cell_box`, `half_extents_m`, 실제 `measurement`)이 있으면 위 author/init 명령에 `--workspace FILE`을 추가할 수 있습니다. 모든 phase corridor가 그 같은 작업공간을 사용합니다. 전체 장애물·양팔·payload 검사는 그대로 남으므로 phase마다 상자를 손으로 반복 입력할 필요가 줄어듭니다. 이것이 작업공간 안의 장애물을 생략한다는 뜻은 아닙니다.

`random-motion.json`의 `robot_mounting_supports`는 고정된 로봇 바닥과 실제 장착 상판 사이의 지지 접촉용입니다. 실제 obstacle 이름과 상판 평면, 바닥 케이블이 평면 위에 있음을 한 번 기록합니다. 원본 base CAD 전체가 평면 위에 있는지 검사한 경우에만 고정 base capsule의 둥근 껍질과 그 상판의 겹침을 허용합니다. 이동 링크·Hand·다른 장애물에는 예외를 적용하지 않습니다.

## 5. 첫 현장 테스트

먼저 카메라 인식과 읽기 전용 SDK 확인을 실행합니다. 이어 실제 보정/하중/주변 여유를 채운 뒤에 다음 실제 1mm STOP 시험으로 증거를 만듭니다. 이전 STOP 성공 기록을 가짜로 만들 필요가 없습니다.

```bash
./field-demo commissioning-test --config site/random/site.json --side right --axis z --direction 1 --execute
./field-demo commissioning-test --config site/random/site.json --side left  --axis z --direction 1 --execute
```

위의 `+z`는 **로봇 base 좌표계**입니다. 현장에서 1mm 이동해도 되는 실제 방향으로 선택합니다. 양팔 STOP 기록을 확인한 뒤 `./field-demo random-cycle --recipe site/random/random-cycle.json --single-pick ring2 --execute`로 한 개만 천천히 들어 올려 촬영·파지 확인부터 진행합니다. 이 명령은 정상 종료할 때 물체를 든 채 정지합니다. 실제 입력이 빠져 있거나 관측/충돌 검사가 실패하면 진행하지 않습니다.

실제 로봇에서 파지·전체 조립 성공은 아직 검증되지 않았습니다. CAD 모델·합성 SDK 회귀 테스트를 실기 성공으로 표기하지 않습니다.
