# 빈 손 접근 뒤, 촬영한 바구니 지도를 짧게 유지하는 실험 기능

실제 SDK·카메라로 완성 작업을 검증한 기능은 아닙니다. 현장 프로토타입에서 시험할 수 있는 opt-in입니다. 기본 설치와 IP/보정/실제 하중/STOP 검사는 [MODEL_STARTER.md](MODEL_STARTER.md)와 [RANDOM_ONSITE.md](RANDOM_ONSITE.md)를 그대로 따릅니다.

문제는 손이 바구니 안에 들어온 다음 새 장면 전체를 만들면 자기 손바닥과 그 뒤의 가림 영역이 장애물로 보이는 것입니다. 이 모드는 **손이 가리지 않을 때 촬영한 원래 바구니 지도**를 유지하고, 접근 후 새 영상은 목표 링이 같은 위치에 있는지 확인하는 데 씁니다. 원래 촬영 시각을 바꾸거나 오래된 영상을 새 영상으로 표시하지 않습니다.

`random-motion.json`의 `scene`에 아래 설정을 추가하면 링 pickup에 적용됩니다.

```json
"static_scene_lease": {"maximum_scene_age_s": 12}
```

12초는 작업자가 선택하는 정적 재고 가정의 예시입니다. 값은 양수·최대 15초이고, 실제 경로 시간과 새 촬영/검사 시간을 포함해 원래 노출 시각부터 계산합니다. 15초 동안 부품이 움직이지 않는다는 센서 인증이 아닙니다. 경로가 시간 예산 안에 끝나지 않으면 팔을 보내기 전에 다른 후보를 고릅니다.

동작 순서는 다음과 같습니다.

1. 빈 Hand를 먼저 열고, 바구니 전체와 링들을 새 RGB-D로 촬영합니다. 부품 위치·크기는 픽셀에서 추정합니다.
2. 빈 손의 접근 전체를 `strict_empty_approach`로 검사합니다. 목표 링을 포함해 손가락의 stock-contact 예외를 적용하지 않습니다. 원래 전체 팔/손/반대 팔/기존 삽입물/장애물/UNKNOWN 검사와 SDK 5N/1Nm·tracking·heartbeat 제한을 유지합니다.
3. 실제 native SDK 실행 완료, 원래 CSV와 driver 해시, 빠짐 없는 실제 SDK telemetry를 확인한 뒤에만 짧은 지도 lease를 만듭니다. 시뮬 엔코더 파일을 SDK telemetry로 바꾸지 않습니다.
4. 팔이 멈춘 상태에서 새 RGB-D로 목표를 확인합니다. 새 목표 영상의 유효기간은 기존대로 최대 2초입니다. 크기·위치·자세가 바뀌거나 새로 보이는 재고가 기존 지도와 모순되면 lease를 거부합니다.
5. 원래 지도와 실제 현재 SDK FK/폭으로 정지 Hand의 전체 파지 sweep을 검사합니다. 목표 링의 현재 관측과 원래 지도 ID는 위치·자세로 연결합니다. 지도는 **이 정지 pickup closure 검사에만** 쓸 수 있고 팔 이동·운반에 사용할 수 없습니다.
6. 실제 Hand 명령을 시작하기 **전에** lease를 버립니다. 이후에는 실제 파지 확인, 새 held RGB-D, 설정된 적재물 하중, 제한된 수직 이탈, 단일 부품·유지 확인을 거칩니다. 접촉 후 예전 바구니 지도로 운반하지 않습니다.

다른 팔 이동, Hand 명령, STOP/장치 오류, 만료, 목표 변화도 lease를 무효화합니다. 실제 접촉을 허용하는 접근 경로에는 이 모드를 적용하지 않습니다. SDK wrench만으로 모든 작은 접촉이 없었다고 증명하지 않으며, 결과에 `physical_no_contact_certified=false`를 남깁니다. 사람이 접근 중 바구니를 만지거나 외부 진동으로 재고가 움직일 수 있는 환경에서는 이 정적 가정을 사용하면 안 됩니다.

목표 영상은 기본적으로 같은 실제 basket 카메라를 씁니다. 가림 때문에 필요하면 실제로 고정·보정한 두 번째 카메라 설정을 recipe와 motion의 `camera_configs.contact_target`에 추가할 수 있습니다. 같은 시리얼 카메라에 서로 다른 고정 외부좌표를 부여하면 거부합니다. 카메라를 순간이동시키는 가정은 지원하지 않습니다.

첫 현장 시험은 기존 명령으로 큰 링 하나를 저속으로 집고, 정지해 잡고 있는 상태까지 진행합니다.

```bash
./field-demo random-cycle --recipe site/random/random-cycle.json --validate
./field-demo random-cycle --recipe site/random/random-cycle.json \
  --single-pick ring1 --execute --output-dir reports/pick-large
```

기록된 Isaac 예제에서는 초기 overhead 영상 뒤 빈 손 free descent를 마친 후 새 target 영상의 큰 링 중심 차이가 0.209mm, 축 차이가 0.0000166rad였습니다. 원래 지도 나이는 7.826초였고 촬영 시각은 유지했습니다. 두 개의 실제 기록된 **가상 시점**이며 한 대의 실제 카메라 검증은 아닙니다. 실제 native SDK telemetry가 없어 이 예제로 hardware lease를 만들지 않았습니다. 증거와 한계는 `reports/static-scene-lease/recorded-v280-target-pair.json`에 있습니다. 전체 현장 접근·파지 sweep 통과나 실제 하드웨어 성공을 주장하지 않습니다.

## 현재 lease 폐기 방식

lease는 별도 validity 파일과 nonce에 연결됩니다. 실제 Hand 명령 전에 파일을 원자적으로 비활성화하므로, 예전 관측/lease 사본이나 정지 파지 proof도 다시 사용할 수 없습니다. 최신 SDK 자세·wrench·폭과 원래 geometry/관측 근거를 확인한 한 번의 closure admission만 이 폐기 경계를 지나 명령을 보냅니다. 이미 허가된 닫기는 한 번 완료할 수 있고, admission의 두 번째 사용은 거부합니다. 폐기 파일에 문제가 생겨도 STOP은 실행됩니다.

로컬 fake SDK 통합 테스트에서 이 전체 순서와 Hand 파서/heartbeat/명령 전달을 확인했습니다. 실제 장치 연결은 없었으며 factory 전체 형상·물리 파지 검증을 대신하지 않습니다. 기록: `reports/static-scene-lease/COMMAND_INTEGRATION_STATUS.json`.
