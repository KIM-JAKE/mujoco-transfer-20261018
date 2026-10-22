# 고정 공급대: 실제 RGB-D로 링 크기·중심·앞뒤 관측

이 명령은 **카메라만 열고 관측값을 출력**한다. 로봇·Hand·PLC에 연결하거나 동작 명령을 보내지 않는다. 실행 중 agent나 SAM을 호출하지 않는다. 실제 RGB·Depth의 원형/평면, 알려진 두 링 CAD 치수와 단면 차이로 큰/작은 링, 중심, 부호 있는 축을 계산한다. 링 축 주위 회전각은 관측할 수 없어 정규화한 기준축을 출력한다.

기존 현장 `random-vision`과 별도로 `fixed-native-vision`을 명시해 선택한다. 432에서 세 부품 물리 작업을 통과한 원본 detector를 패키지 import만 바꾸어 포함한다. 이 관측 명령은 새 영상에서 동일한 인식 계산을 수행하지만 실물 파지·조립 성공을 인증하지 않는다.

## 현장 카메라 설정

기존 `site/random/random-camera.json` 형식을 사용한다. `config/fixed-native-camera.example.json`은 아직 채우지 않은 템플릿이다. 다음은 실제 값으로 입력한다.

- `camera_serial`, `camera_frame`, `cell_frame`, `calibration_id`와 실측 증거
- `T_cell_camera`: 카메라 광학 좌표→설비/셀 좌표, 단위 m
- `basket.floor_origin_cell_m`, `basket.basket_to_cell_R`, `basket.outer_bounds_cell_m`: 지금은 바구니 대신 기존 공급 선반의 실측 바닥/관측 영역
- 양팔 `T_cell_base`: 실제 로봇 base→셀 좌표
- 손에 든 링을 관측할 경우 `fixed_native.T_EE_hand.left/right`: 실제 SDK EE→시뮬에서 사용한 nativeHand TCP의 실측/확인된 변환. 알 수 없는 값을 단위행렬로 채우지 않는다.

RGB intrinsics는 RealSense SDK가 현재 스트림에서 읽는다. 왜곡 없는 카메라라는 가정으로 깊이를 임의 역투영하지 않고, RGB에 정렬된 Depth의 SDK H×W×3 점군을 사용한다. `T_cell_camera`와 선반/지그/로봇 좌표 보정은 카메라 설치를 바꿀 때 다시 측정한다.

기존 설치 의존성 `numpy`, `scipy`와 별도 `pyrealsense2`가 필요하다. GPU나 Isaac Sim은 필요하지 않다. 카메라 시작 후 노출 안정화를 한 번 수행하고 같은 스트림을 재사용한다.

```bash
./field-demo fixed-native-vision \
  --camera-config site/random/random-camera.json \
  --part ring2 --mode initial --frames 100 \
  --enable-live-camera --output site/random/fixed-small-latest.json
```

`ring2`는 작은 링, `ring1`은 큰 링, `pin`은 핀이다. 초기 링 인식에는 링 위치·자세를 입력하지 않는다. 핀을 관측할 때만 기존 실측 `pin_holder.T_cell_pin_seed` 주변을 찾는다. 공개한 시뮬 좌표를 현장 측정값으로 사용하지 않는다.

출력의 `poses[].center_m`, `axis`, `T_cell_part`, `measured_classification_diameter_m`, `front_back_from_RGBD`를 확인한다. 출력은 셀 좌표이며 위치 단위는 m이다. 실패하면 이전 목표를 남기지 않고 `observation_rejected` 상태로 교체한다.

## 실제 SDK 상태로 손에 든 링의 관측 영역 설정

```bash
./field-demo fixed-native-vision \
  --camera-config site/random/random-camera.json \
  --part ring2 --mode held --frames 100 --enable-live-camera \
  --sdk-state-json site/random/current-right-sdk.json \
  --grasp-json site/random/last-small-RGBD-grasp.json \
  --output site/random/held-small-latest.json
```

SDK 상태 파일은 별도 읽기 전용 상태 수집기가 갱신한다. 이 명령은 로봇 IP로 접속하지 않는다. 파일은 `captured_at_utc`와 `state.read_only:true`, 실제 libfranka의 `O_T_EE_column_major` 16개 값을 포함한다. live 관측은 250ms보다 오래된 상태를 거부한다. 실제 SDK 관절/FK, 실측 base와 EE→Hand 변환으로 Hand 자세를 계산한다.

이전 파지 파일은 `{ "source":"RGBD_measured_hand_to_part", "part":"ring2", "capture_id":"...", "measured_grasp":[[...],[...],[...],[...]] }` 형식이다. `measured_grasp`는 앞선 실제 RGB-D와 당시 SDK Hand 자세로 측정한 Hand→부품 변환이다. 이 값은 새 영상의 관측 영역에만 쓰며, 새 중심·축은 현재 RGB-D에서 다시 계산한다. 부품 actor 좌표·시뮬 object pose는 입력하지 않는다.

## 검증 범위

실제432 저장 영상4개(작은/큰 링 초기와 손에 든 상태)에서 기존 결과와 중심이 동일하고 signed axis가 수치 정밀도 내에서 일치했다. CPU 단일 재생 측정은 초기 약0.60초, 손에 든 링 약0.14초였다. 실제 카메라·로봇 성능 측정은 아니다. 렌더러 물체 좌표를 추가한 packet은 거부했다.

실물 금속의 반사·깊이 결손, 카메라/로봇 보정 오차, detector의 이상적 깊이 문턱값은 현장 영상에서 먼저 확인한다. 이 camera-only 명령은 원본 RealSenseStream의 획득 시간 제한과 중복 프레임 거부를 사용한다. SDK 하드웨어 시각의 완전한 노출 나이는 이 새 명령에서 인증하지 않으므로, 기존 실제 로봇 실행기의 동기화·신선도 검사를 대신하지 않는다. 관측 결과만으로 저장된 시뮬 궤적을 실기에 바로 재생하지 않는다.
