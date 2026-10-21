> 링크 반경·Hand 모양·부품 sector를 직접 재기 전에 [MODEL_STARTER.md](MODEL_STARTER.md)의 자동 작성 절차를 사용하세요. 모델 파생 값은 실제 측정값으로 위장하지 않습니다.

# RGB-D 무작위 바구니 → 링 2개·핀 1개: 현장 온보딩

패키지 루트에서 실행한다. 현재 실기 경로는 **실제 RGB-D + Franka SDK**를 사용하고 양팔은 순차로 움직인다. `--execute`가 실제 동작을 허용하는 명시적 옵션이다. 실제 로봇의 랜덤 집기·조립 완주와 작업시간은 아직 검증하지 않았다. 인계 목표는 2026-10-22 오전 11시 KST다. [이번 연동 범위](../integration/PAYLOAD_EXPERIMENTAL.md)와 [상세 RGB-D/SDK 안내](../integration/RGBD_SDK_ONBOARDING.md)를 함께 본다.

## 1. 설치·읽기 전용 확인

[START_HERE](../START_HERE.md)의 호스트 설치·소스 빌드·오프라인 검사를 마친다. 새 설정은 실제 값을 넣기 전까지 null 상태다.

```bash
./field-demo verify
./field-demo init-random --output site/random
./field-demo camera list
./field-demo doctor --config site/random/site.json --output reports/site-check.json
./field-demo stabilization check --profile stabilization/v128/field-profile.json
```

`site/random/site.json`에 실제 서로 다른 양팔 IP를 넣는다. SDK 대상은 FR3 프로토콜 10이다. native Franka Hand를 선택하며 TCP/tool/load는 실제 Desk/SDK 설정과 일치시킨다. 상태 조회는 다음과 같다.

```bash
./field-demo run status --config site/random/site.json
```

이 단계는 로봇을 움직이지 않는다. 실제 SDK 상태를 읽는 성공과 실제 파지/조립 성공은 별개다. 보존한 v128 시뮬 성공·43.533초 기록은 참조 자료이며 시뮬 좌표·CSV를 실기에 재생하지 않는다. 최신 H의 GPU/cuRobo 시뮬 정책 전체가 이 패키지에 들어온 것도 아니다. 이 패키지는 현재 실제 SDK 모델로 경로를 생성한다.

## 2. 실제 측정 입력

모든 좌표는 같은 셀 좌표계, 길이는 m를 사용한다.

| 파일 | 채울 실제 값 |
|---|---|
| `site.json` | 양팔 IP·기저 변환·tool/load, native Hand 폭/힘, 실제 셀 정지 모드, 보정/시운전 기록 |
| `random-camera.json` | RealSense 일련번호·외부 보정, 바구니/관측 영역, 핀 홀더 탐색 범위, 지그 관측 영역, 실제 Depth 오차 상한 |
| `random-motion.json` | 고정 바구니·테이블·설비·지그, 양팔/Hand/케이블의 보수적 형상, 단계별 통로, 실제 외력/정지 증빙, 잔여 RGB-D 점유 설정 |
| `random-cycle.json` | 부품 순서/팔, 모델 파생 또는 실제 교시 파지 후보·접근/철수·전달 자세, 지그 목표, 클램프 방식, 선택적 적재물 접촉/관측/반환 설정 |
| `payload-profile.json` | 각 팔의 별도 Hand/tool과 빈 load, 각 부품 명목 CAD 또는 실제 측정 질량·COM·관성·좌표계·출처 증빙 |

`measurement`의 실제 측정 경로는 확인자·시각·비어 있지 않은 증빙 파일을 가리킨다. `MODEL_STARTER.md`의 자동 작성 경로는 별도로 `kind=model_derived, verified=false`를 유지하며 한 개의 실제 설치 일치 기록과 고정된 모델 출처·수치 해시로 승인 범위를 구분한다. 모델 숫자를 실측이라고 표시하지 않는다. boolean 변경이나 가짜 기록은 물리 측정을 대신하지 않는다. 하중 프로파일을 완성하면 정확한 SHA-256을 `random-cycle.json`의 `payload_profile_sha256`에 넣는다. Hand는 `m_ee`이고 물체는 `m_load`이므로 중복 합산하지 않는다. 프로파일의 tool/빈 load와 실제 SDK 초기 tool/load가 다르면 자동 덮어쓰지 않고 거부한다.

부품별 파지·착좌·유지용 PLC 센서는 필요 없다. `part_evidence_source=RGBD_SDK`, `abort_hand_policy=preserve_completed_grasp`를 사용한다. 실제 셀 PLC가 없으면 `cell_io_mode=SDK_stop`을 사용하며 실제 native 외력·추종·heartbeat 검사는 유지한다. 셀 PLC가 있으면 실제 주소/극성/준비·고장·정지를 연결해 `actual_PLC`를 선택할 수 있다.

클램프는 실제 구동이 필요하다. `clamp_control.mode=actual_PLC`이면 실제 명령과 읽기 확인을 연결한다. `operator_manual`이면 사람이 실제 클램프를 고정하고 `CLAMP ring1` 같은 요청된 문구를 입력한다. 수동 확인을 자동 클램프나 압력 측정으로 표시하지 않는다.

## 3. 카메라·인식부터 확인

역할은 `basket`, `held_right`, `held_left`, `pin_holder`, `fixture`다. 한 고정 카메라가 모두 볼 수 있으면 같은 일련번호·같은 실측 외부 보정을 공유한다. 같은 카메라가 역할마다 다른 위치라고 선언할 수는 없다. 실제 손목 카메라의 동적 외부 보정은 이 실행기에 구현하지 않았다.

```bash
./field-demo random-vision observe --config site/random/random-camera.json \
  --output reports/random-live.json --session-dir reports/random-camera --frames 3
```

새 RGB-D 픽셀과 실제 SDK XYZ로 크기·중심·축·앞뒤를 구한다. SAM이나 실행 중 LLM agent는 호출하지 않는다. 물체 ID는 해당 이미지 안에서만 유효하다. 물체 초기 좌표·시뮬 actor 위치를 넣지 않는다. 핀은 실측 홀더 ROI에서 현재 영상을 본다. 지그 목표도 실측 고정 ROI/삽입 목표일 뿐 현재 부품 위치를 대신하지 않는다.

`residual_occupancy`는 미인식/가림/무효 깊이를 UNKNOWN 장애물로 남긴다. 실제 SDK 렌즈 보정은 관측을 정렬하되 Depth 보간이나 구멍 채우기를 하지 않는다. 명시적 `depth_uncertainty_m` 상한이 필요하다. 실패한 검출을 빈 바구니로 처리하거나 오래된 영상의 시각을 바꾸지 않는다. 재실행 시 새 빈 `session-dir`을 쓴다. `--frames 0`은 연속 관측이며 실패하면 이전 타깃을 제거한다.

## 4. 첫 1mm 이동·STOP 시험

이번 패키지는 선택적 [5mrad 시운전 프로파일](LOW_SPEED_COMMISSIONING.md)도 제공합니다. 기본 20mrad를 바꾸지 않으며, 별도 프로파일의 같은 임계값을 native 이동·경로 검사·정지 Hand 모니터에 적용합니다. 실제 양팔의 첫 시험/telemetry/파일 해시가 있어야 ready 레시피를 만듭니다. 더 작은 임계값은 더 이른 중단이며 이미 정확도를 검증했다는 뜻이 아닙니다.

실제 작업 공간, tool/load, 단일 팔 시험 영역, 보정값을 먼저 확인한다. 다음 경로는 **이전 `stop_tested` 통과를 요구하지 않고** 실제 시험 기록을 만든다.

```bash
./field-demo commissioning-test --config site/random/site.json --side right --axis z --direction 1 --execute
./field-demo commissioning-test --config site/random/site.json --side left --axis z --direction 1 --execute
```

축과 방향은 **로봇 BASE 좌표** 기준이다. 위 z+는 예시이므로 실제 1mm 여유가 있는 방향을 고른다. native 10% 이하 경로에서 실제 CSV 이동이 관측된 뒤 명시적 STOP을 보내고, native 수락/SDK 반환과 실제 `idle/dq`를 대조한다. 손·클램프는 움직이지 않는다. 실제 PASS 기록만 정지 증빙과 `commissioning_record`/`stop_tested`에 사용한다. 실패 기록을 통과로 고치지 않는다. 이 소프트웨어 정지 시험은 현장 비상정지·안전 등급 인증의 대체물이 아니다.

Hand 단독 명령은 `./field-demo hand --help`를 참고한다. 현재 실기 파지는 큰 실제 개방 폭에서 **닫아서 잡는 외경 파지**다. 시뮬에서 사용한 내경 확장 파지는 SDK 힘 제어 지원을 검증하지 않아 실행하지 않는다. 실제 폭·접촉·들고 유지하기를 저속으로 확인한다.

## 5. 단품 집기 → 전체 조합

```bash
./field-demo random-cycle --recipe site/random/random-cycle.json --validate
./field-demo random-cycle --recipe site/random/random-cycle.json \
  --single-pick ring1 --execute --output-dir reports/pick-large
./field-demo random-cycle --recipe site/random/random-cycle.json \
  --single-pick ring2 --execute --output-dir reports/pick-small
```

설정 검사는 장치 동작 없이 실제 입력의 누락/불일치를 검사한다. `--single-pick`은 해당 링만 집고 들어 재관측한 뒤 **팔은 정지하고 손은 잡은 채** 끝난다. 자동 전달·삽입·개방은 하지 않는다. 결과의 실제 Hand 상태와 `single_item_inventory_proof`를 확인한다. 이 증거가 null이면 양면 광학 단일 개수 검증을 수행했다고 해석하지 않는다. 물체를 잡은 채 새 공정을 시작하면 초기 빈 손/빈 하중 검사에서 거부된다.

엄격한 점유로 접근이 막히는 느슨한 링 재고는 선택적 `bounded_loose_stock_contact`를 사용할 수 있다. 기본은 비활성이다. 두 종류 링·최대 10개라는 실제 재고 조건, 실측 바구니 내부, 실제 손가락 이름/캡슐, native 실행 파일과 실제 STOP 시험 해시를 설정한다. 지정된 손가락 전체가 내부에 있는 샘플만 적재물 접촉을 허용하며 벽·손바닥·팔·반대 팔·운반 부품은 예외가 아니다. UNKNOWN을 지우거나 빈 공간으로 바꾸지 않는다. 5N/1Nm SDK 외력 중단과 250ms heartbeat를 유지한다.

현재 `stock_departure`는 별도의 명시적 첫 이탈 기술이다. 같은 재고/정지 증빙과 실측 위 방향으로 최대 70mm만 허용하고, 선택한 영상/SDK 부품과 지정 손가락만 바구니 내부 적재물에 접촉할 수 있다. 발행 FK의 아래 방향/측방 운반/회전을 검사하며 실제 외력·추종 중단을 유지한다. 이후 양면 단일 개수/G 검사를 통과해야 나머지 운반을 시작한다. 이 기술을 충돌 없는 이동이나 물리적 단조 운동 인증으로 표시하지 않는다.

이 모드는 `inventory_inspection` 설정도 필요하다. 실제 SDK `F_T_EE`와 실제 설치와 일치하는 `T_flange_hand`로 공식 Hand 표면을 배치하고, 실제 로봇을 검사된 자세로 회전해 같은 카메라에서 링 양면을 본다. 넓은 ROI의 모든 깊이를 단일 CAD와 대조해 추가 물체·숨은 적층·가림·불안정한 G를 거부한다. 선택한 부품만 잘라서 개수를 확인했다고 하지 않는다.

검증 실패 시 측정한 불확실 적재물 외피를 포함해 전체 충돌 검사를 통과한 경로로 기존 바구니 위에 반환한다. 실제 반환 영역을 확인한 뒤 개방하고, 서로 다른 두 실제 뷰에서 빈 손을 확인한 후 빈 하중으로 바꾼다. 미확인 걸림 가능성은 빈 손 증명 전까지 운반 형상에 남긴다. 경로·시야가 없으면 반환을 강행하지 않고 파지를 유지하며 중단한다. 자동 재시도나 실패 무시를 구현한 것은 아니다. 첫 들기 전 숨은 추가 부품의 질량/COM은 알 수 없으므로 초기 선택 클래스 하중을 다중 부품 하중 검증으로 해석하지 않는다.

단품의 접근·들기·유지·반환/놓기와 전달/삽입을 확인한 뒤 전체를 실행한다.

```bash
./field-demo random-cycle --recipe site/random/random-cycle.json \
  --execute --output-dir reports/random-live-cycle
```

공정은 새 인식 → 경로 선택 → 이동 직전 재촬영 → 실제 파지/새 G → 들기 → 필요 시 양손 전달 → 새 G로 삽입 목표 계산 → 새 영상 안착 → 실제/수동 클램프 → SDK 개방 → 2회 이상 새 영상의 최소 0.3초 유지 → 철수 → 설치한 모든 부품 재관측 순서다. 받는 손은 주는 손이 아직 잡은 동안 실제 SDK와 새 영상으로 확인하고, 주는 손 개방 이후 받는 손을 다시 촬영한다. 부품별 PLC 파지 센서는 사용하지 않는다.

카메라·SDK 오류, 물체 이탈, 파일 변경, 막힌 경로는 중단 사유다. 일반 중단은 팔 heartbeat를 중단하고 완료된 Hand 파지 명령을 무조건 취소하지 않는다. 실제 손 상태는 별도 SDK 읽기로 확인하며 200ms 이상 오래되면 사용할 수 없다. 파지 명령 보존과 물체 유지 보장은 다르다. 자동 재개는 없다. 실제 작업시간과 안정성은 현장 시험 결과로 기록한다. 현재 native 실기 상한은 10%이며 시뮬의 95% 속도 설정이 아니다.

## 6. 기록과 참조

`reports/PAYLOAD_INTEGRATION_STATUS.json`은 구현/검사/미검증 범위를 구분한다. 저장 RGB-D 재생, 가짜 SDK/Hand/클램프 경계의 상태머신 검사, 실제 SDK 렌즈 함수 검사는 각각 다른 증거다. 실제 로봇의 파지·조립 성공으로 합치지 않는다.

`stabilization inspect/import`는 완료한 시뮬 run의 원본 manifest와 스냅샷을 확인해 새 참고 폴더에 복사한다. 검출기가 달라지면 별도 포팅/회귀 검사가 필요하며 경로·속도 개선이 실기에 자동 적용되지 않는다. 기존 봉인 패키지는 수정하지 않았다.

실험 옵션: [STATIC_SCENE_LEASE.md](STATIC_SCENE_LEASE.md)는 엄격한 빈 손 접근 뒤 원래 바구니 지도를 정지 파지 검사에만 잠시 유지합니다. 실제 Hand 명령 전에 폐기하며, 원래 촬영 시각·실제 SDK 근거를 유지합니다. 기본은 비활성이고 전체 현장 성공은 미검증입니다.

## 이번 기록 기반 연동 결과

4mm 잔여 점유의 기본 예산은 1,000,000 voxel입니다. 선택 표면의 CAD 검색을 후보 픽셀로 제한했고, 동일 기록에서 점유 상태와 직렬화 결과가 바이트 단위로 같은 것을 확인했습니다. 5mrad 레시피의 최대24회 파지/IK 대안 탐색은 읽기 전용 SDK 모델에만 다른 초기값을 줍니다. 실제 실행 시작은 현재 측정한 q_d이며, 기존 속도·전체 형상·반대팔·UNKNOWN·추종 검사를 모두 유지합니다.

[현재 기록 검사](RECORDED_PIPELINE_STATUS.md)는 두 가상 뷰 융합 후에도 전체 끝 자세가 막혔습니다. 실기 무작위 집기나 접근/파지 완료를 검증한 결과가 아니며, 현장 시험용 도구와 확인된 차단 원인을 인계합니다.

핀의 지그 아래 정렬에는 선택적 [정밀 피드백](PIN_PRECISION.md)을 사용할 수 있습니다. 새 SDK desired-FK 출력이 필요하므로 소스를 재빌드합니다. 기본 비활성이고 실제 ICP 정밀도/수렴은 미검증이며, 실제 명령 직전에도75µm/3mrad/150µm 기준을 다시 검사합니다.

핀 카메라는 선택적으로 [직접 끝면 원·축 측정](PIN_CAP_RGBD.md)을 사용할 수 있습니다. 기본은 ICP입니다. 새 모드는 실제 원주/plane의 품질이 부족하면 거부하며 이전 XY를 사용하지 않습니다. 기록된 여러 가상 뷰에서만 확인했고 실제 고정 카메라/금속 깊이는 현장 확인이 필요합니다.

후보 탐색은 [PICKUP_SEARCH.md](PICKUP_SEARCH.md)를 따릅니다. 초기값 하나로 영상에 보이는 같은 크기 링과 파지 자세 후보를 차례로 검사한 다음, 남은 예산으로 다른 초기값을 시도합니다. 모든 후보가 UNKNOWN/충돌로 막히면 이동하지 않고 중단합니다. 계획 횟수와 촬영/신선도 제한을 풀지 않습니다.


공식 FR3 Duo 공통 강체 마운트를 사용하는 경우, 기본적으로 엄격하게 검사되는 고정 베이스 쌍에만 별도 설치 증빙을 연결할 수 있습니다. [공통 Duo 마운트 설정](COMMON_DUO_MOUNT.md)을 따르세요. 실제 설치 확인 전에는 기본값 꺼짐을 유지합니다.


같은 팔의 보수적 capsule 때문에 막히는 경우에만 [전체 원본 링크 hull 선택 검사](CONVEX_SELF.md)를 사용할 수 있습니다. 기본 20mrad, 실제 설치 증빙, 케이블 여유, SDK/STOP 및 UNKNOWN 조건은 그대로이며 이 기능만으로 실기 가능 판정을 하지 않습니다.


같은 팔의 보수적 capsule 때문에 막히는 경우에만 [전체 원본 링크 hull 선택 검사](CONVEX_SELF.md)를 사용할 수 있습니다. 기본 20mrad, 실제 설치 증빙, 케이블 여유, SDK/STOP 및 UNKNOWN 조건은 그대로이며 이 기능만으로 실기 가능 판정을 하지 않습니다.


설비/UNKNOWN 상자에서 보수적인 로봇 capsule이 막히는 경우 [닫힌 전체 hull 선택 검사](CONVEX_WORLD.md)를 검토할 수 있습니다. 상자·UNKNOWN·손가락 접촉·실기 한도는 그대로이며, 실제 SDK 시작 상태와 증빙으로 실행 직전 다시 검사합니다.
