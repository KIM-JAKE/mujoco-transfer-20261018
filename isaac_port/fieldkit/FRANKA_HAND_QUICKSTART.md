# 현장: FR3 + Franka Hand + CaP

이번 버전은 **Franka Hand의 실제 libfranka 통신과 순차 CaP 어댑터를 구현**했다. IP를 넣어 바로 할 수 있는 것은 상태 읽기와 명시적인 손가락 테스트다. 팔 이동은 좌표·부하·작업 공간·PLC 정지 조건을 먼저 설정해야 한다. **링 2개·핀 삽입·버튼 전체 자동 조립은 실물에서 검증하지 않았다.** 원래 9초 Robotiq 경로를 Franka Hand 실기에 재생하는 버전이 아니다.

Franka Hand는 로봇 컨트롤러 IP를 사용한다. 별도의 Robotiq Modbus 주소가 필요 없다. 카메라 위치 추정·자동 접촉 탐색·힘 제어 삽입은 이번 실기 어댑터에 없다. 위치가 달라지면 아래 경로를 다시 기록하고 검토한다.

## 1. 받기·빌드

현장 제어 PC에서 실행한다. GPU·Isaac·SAM·대용량 아카이브 복원은 필요 없다.

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/KIM-JAKE/mujoco-transfer-20261018.git mujoco
cd mujoco
# 이미 복제한 경우 위 두 줄 대신: git pull --ff-only
cmake -S isaac_port/fieldkit -B /tmp/m80-field-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/m80-field-build --parallel 2
cmake -E chdir /tmp/m80-field-build ctest --output-on-failure
python3 -m unittest isaac_port.fieldkit.test_hand isaac_port.fieldkit.test_devices isaac_port.fieldkit.test_fieldkit -v
```

Python 3.10+, C++17, Eigen3, 현장 로봇 System Image와 호환되는 libfranka가 필요하다. 이 실행기는 **libfranka 0.18 이상 / FR3 프로토콜 10**을 대상으로 빌드·검사했다. SDK 버전은 [FR3 호환표](https://franka.de/fr3-compatibility-matrix)와 현장 Desk 버전으로 확인한다. 실기 팔 제어는 PREEMPT_RT·실시간 권한과 FCI 연결이 필요하다. 해당 설정은 기존 [온보딩](ONBOARDING_FR3.md)을 따른다.

## 2. 손만 먼저 확인

아래 IP는 예시다. 현장 로봇 IP로 바꾼다. 서버의 외부 접속 IP와 다르다.

```bash
LEFT_IP=192.168.10.2
RIGHT_IP=192.168.20.2
python3 -m isaac_port.fieldkit.hand status --ip "$LEFT_IP"
python3 -m isaac_port.fieldkit.hand status --ip "$RIGHT_IP"
```

`width_m`, `max_width_m`, `is_grasped`, 온도가 나온다. 이 명령은 손가락과 팔을 움직이지 않는다. 손가락 주변을 비우고 필요하면 **명시적으로 homing**한다. Homing은 손가락을 열고 닫으므로 물건을 물고 있을 때 실행하지 않는다.

```bash
python3 -m isaac_port.fieldkit.hand homing --ip "$LEFT_IP" --execute --clear-hand-area
python3 -m isaac_port.fieldkit.hand move --ip "$LEFT_IP" --width-mm 40 --speed-mm-s 5 --execute --clear-hand-area
# 대상물을 실제 파지 위치에 둔 뒤, 실측 폭으로 변경한다. 40mm는 예시다.
python3 -m isaac_port.fieldkit.hand grasp --ip "$LEFT_IP" --width-mm 40 --speed-mm-s 5 --force-n 5 --epsilon-mm 0.5 --execute --clear-hand-area
python3 -m isaac_port.fieldkit.hand stop --ip "$LEFT_IP"
```

이 독립 손 도구는 **PLC를 감시하지 않는다**. 사람이 손가락 시험을 수행할 때 사용한다. 손만 구동하며, CaP 조립은 PLC 감시가 있는 `site` 도구로 실행한다. 개구 폭은 양쪽 손가락 사이 전체 거리다. 단위는 CLI에서 mm·mm/s·N, 내부 설정에서 m·m/s·N이다. 첫 시험 상한은 속도 20mm/s, 힘 20N, 편차 각 2mm이며 이 숫자는 물체 파지 적합성 인증이 아니다. 파지 성공은 명령 ACK와 SDK `is_grasped`를 함께 요구한다. Homing으로 측정한 최대 폭을 넘는 요청은 거부한다.

## 3. 현장 설정

```bash
python3 -m isaac_port.fieldkit.teach init --left-ip "$LEFT_IP" --right-ip "$RIGHT_IP" --output /tmp/m80-site/site.json
```

`/tmp/m80-site/site.json`을 편집한다. 원본은 `site.franka.example.json`이다. 재부팅 후에도 보관하려면 `/tmp` 대신 현장 작업 폴더를 사용한다.

- `robots`: 각 IP, 실측 `T_cell_base` 4×4 변환(열벡터 `p_cell = T_cell_base @ p_base`), Desk에 설정된 실제 tool/load 질량. 무게중심·관성과 TCP도 Desk/현장 SDK에서 실제 장착 상태에 맞춰야 한다. 이 도구는 임의 값으로 설정을 덮어쓰지 않는다.
- `calibration_record`, `commissioning_record`: 좌표/TCP/부하 측정과 정지 시험 기록 파일. 비어 있으면 이동하지 않는다. `commissioning` 항목들은 실제 확인 후에만 true로 바꾼다.
- `plc.endpoint`: Modbus TCP PLC 또는 검증된 게이트웨이 IP/port/unit. Profinet·Ethernet/IP는 별도 어댑터가 필요하다.
- `plc.signals`: **0 기반** 실제 주소, discrete/coil/input/holding 종류와 active 극성. 기본 주소 null은 의도적으로 미설정이다. `cell_ready`, `cell_fault`, 전용 `cell_stop`, 각 부품의 grasped/seated/clamped/retained/clamp_command, `button_pressed`를 연결한다. register는 mask와 active 값을 지정한다. 값이 없으면 정지한다.
- `gripper_actions`: 각 부품의 `open`, `grasp`, `release` 폭을 **실측**한다. native Hand 설정에는 Robotiq의 position 0..255를 넣지 않는다. open/release 폭은 grasp 허용 폭보다 넓어야 한다.
- `button_verified_travel_m`: 버튼 신호가 실제 1.8mm 이상 눌림에 대응하는지 측정한다. 눌림 유지 100ms와 복귀 신호를 확인한다.

`grasped` PLC 신호는 현재 주기의 물체 존재/파지 감시 입력이다. Franka SDK 파지 응답도 추가로 확인한다. `seated`·`retained`는 지그 센서 등 실제 피드백이어야 한다. SDK의 손가락 폭만으로 삽입 완료나 클램프 유지 성공을 만들지 않는다. 현장에 해당 입력이 없으면 관측 어댑터를 추가해야 하며 true 상수로 대체하지 않는다.

```bash
python3 -m isaac_port.fieldkit.site validate --config /tmp/m80-site/site.json --log-dir /tmp/m80-site/logs
python3 -m isaac_port.fieldkit.site status --config /tmp/m80-site/site.json --log-dir /tmp/m80-site/logs
# 작업 공간/정지/부하/PLC 확인 후: 7번 관절 0.01rad, 3초 이동
python3 -m isaac_port.fieldkit.site jog --side left --joint 7 --delta 0.01 --duration 3 --config /tmp/m80-site/site.json --log-dir /tmp/m80-site/logs --execute
# cell 좌표 z 방향 1mm의 endpoint IK 시험: 관절 보간이며 직선 TCP 이동 보장은 아님
python3 -m isaac_port.fieldkit.site cartesian-jog --side left --axis z --distance-mm 1 --duration 3 --config /tmp/m80-site/site.json --log-dir /tmp/m80-site/logs --execute
```

`validate`는 설정 검사이며 로봇/PLC 연결 성공이나 충돌 검증을 뜻하지 않는다. `status`는 실제 장치를 읽는다. 팔 이동은 명령 한도의 **10%**에서 연속 관절 위치·위치별 속도·가속도·jerk 검사를 한다. 단일 jog는 0.05rad 이하, Cartesian endpoint 변경은 5mm 이하, 2~30초다. 실제 SDK 모델 토크/변화율을 사전 검사하며 실행 중 추종·통신 주기·관절 토크 변화율과 외력 5N/외부 모멘트 1Nm를 감시한다. 기본 삽입 컨트롤러가 아니므로 접촉으로 중단되면 무조건 재시도하거나 한도를 풀지 않는다.

## 4. Franka Hand 경로를 실물에서 기록

Franka Hand가 짧아 이전 Robotiq 관절 경로는 사용할 수 없다. 형상 검사에서만 원본 배치의 일부 삽입 자세 여유가 약 **0.77~1.53mm**였으며, 이는 실물 공차·연속 충돌·파지 유지·물리 삽입 성공을 보증하지 않는다.

가이드 모드 등 현장 절차로 경유 자세를 만든 후 **가이드 모드를 종료하고 정지 상태에서** 각각 기록한다. 아래 capture는 읽기만 하며 자동으로 자세를 만들거나 초기 위치로 이동하지 않는다. 중간 회피 자세를 충분히 기록한다. 두 점 사이 보간 경로 전체를 검사해야 한다. 다른 팔과 모든 설치된 링·지그도 검사 대상이다.

```bash
python3 -m isaac_port.fieldkit.teach capture --ip "$LEFT_IP" --output /tmp/m80-site/poses/ring2-start.json
# 현장에서 다음 정지 자세로 이동 후
python3 -m isaac_port.fieldkit.teach capture --ip "$LEFT_IP" --output /tmp/m80-site/poses/ring2-clearance.json
# 현장에서 파지 직전 정지 자세로 이동 후
python3 -m isaac_port.fieldkit.teach capture --ip "$LEFT_IP" --output /tmp/m80-site/poses/ring2-grasp.json
python3 -m isaac_port.fieldkit.teach compile --config /tmp/m80-site/site.json --stage left.grasp.ring2 --poses /tmp/m80-site/poses/ring2-start.json /tmp/m80-site/poses/ring2-clearance.json /tmp/m80-site/poses/ring2-grasp.json --seconds-per-segment 10 --path-reviewed
```

`--path-reviewed`는 작업자의 **전체 경로 검토 기록**이다. 이 도구가 연속 형상 충돌을 검사했다는 뜻이 아니다. 컴파일은 각 기록점에서 멈추는 5차 C2 관절 곡선을 만들고 10% 운동학 한계를 검사한다. 단계 전체는 최대 60초다. 같은 로봇 IP·동일 TCP·설정 질량인지 확인하고 CSV SHA-256을 등록한다. 경로를 수정하면 전체 조립 승인(`assembly_commissioned`)을 false로 되돌린다.

필요한 12개 단계:

| 팔 | 단계 |
|---|---|
| 오른팔 | `right.grasp.ring1`, `right.insert.ring1`, `right.release.ring1` |
| 왼팔 | `left.grasp.ring2`, `left.insert.ring2`, `left.release.ring2` |
| 왼팔 | `left.grasp.pin`, `left.insert.pin`, `left.release.pin` |
| 오른팔 | `right.approach.button`, `right.approach.button_press`, `right.approach.button_release` |

`grasp`는 열린 손으로 접근한 뒤 파지, `insert`는 부품을 유지하며 안착, `release`는 클램프 확인 후 **같은 위치를 유지하며 손만 개방**한다. release CSV는 동일 q의 정지 기록 두 개로 만든다. 다음 grasp/approach 경로에 직전 지그에서 빠져나오는 회피 경유점을 포함한다. 양팔 정책 호출은 순차로 실행한다. 이미 설치된 다른 부품과 양팔 공유 작업 공간을 검토한다.

개별 경로 시험은 아래처럼 실행한다. **손가락 파지나 클램프 명령을 자동 실행하지 않고 팔 CSV만 실행**하므로, 해당 경로의 손가락·물체·클램프 상태를 먼저 맞춘다.

```bash
python3 -m isaac_port.fieldkit.site trajectory --side left --stage left.grasp.ring2 --config /tmp/m80-site/site.json --log-dir /tmp/m80-site/logs --execute
```

시작 q_d는 저장 경로와 1e-6rad 이내, 실제 q는 0.005rad 이내여야 한다. 자동 초기 자세 이동은 없다. 일치하지 않으면 실행을 거부한다. 처음 시험의 시작 자세를 재기록하거나, 현장 검토를 거친 별도 접근 경로를 준비한다. 두 단계가 같은 팔에서 이어질 때 앞 단계 끝과 다음 단계 시작이 맞아야 한다.

## 5. CaP 전체 실행

전체 순서는 `policy.assemble()` → `site.ConfiguredCell` → libfranka 팔 실행기 / native Hand / PLC다. `execute_parallel` 요청도 현장 버전에서는 **순차 실행**하여 미검증 동시 경로를 실행하지 않는다.

12단계와 입출력·버튼 스트로크를 각각 실물에서 확인한 뒤에만 `assembly_commissioned: true`로 바꾼다. 새 주기 시작에는 부품 피드백과 버튼이 해제되어 있어야 한다. 물체를 이미 설치한 상태에서 다시 누르면 자동 초기화하지 않고 중단한다.

```bash
python3 -m isaac_port.fieldkit.site assemble --config /tmp/m80-site/site.json --log-dir /tmp/m80-site/logs --execute
```

각 부품은 파지 확인 → 안착 확인 → 클램프 닫힘 확인 → 손 개방 확인 → 유지 확인 순서다. 세 부품의 유지가 확인되어야 버튼을 누른다. 실패 시 손을 임의로 열거나 클램프를 해제하지 않고 공통 중단을 요청한다. 로그에는 설정 해시·단계 결과·SDK 상태와 관절/토크/외력 CSV가 남는다.

Python이 READY를 보내지 못하면 native 팔·손 실행기의 250ms watchdog이 중단을 요청한다. PLC fault, EOF, 시간 초과도 중단 처리한다. 이는 소프트웨어 중단이며 인증된 비상정지 회로나 하드웨어 정지 시간을 대신하지 않는다. 실제 양팔·손·PLC 정지 동작을 현장에서 검증한다.

## 검증 범위

공식 libfranka 0.18.1 기반 전체 C++ 빌드와 오프라인 CTest, Python 로컬 가상 장치 테스트를 사용한다. 실제 SDK 파지 실패·시간 초과·PLC 이탈과 CaP의 fresh feedback 처리를 검사한다. **실제 로봇 연결·파지력·자동 삽입·전체 사이클 시간은 현장 미검증**이다. `hardware_ready:false`는 이 상태를 명시한다.
