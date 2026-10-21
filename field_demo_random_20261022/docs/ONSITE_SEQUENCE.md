# 10월 22일 현장 순서

먼저 `START_HERE.md`의 설치·진단을 끝냅니다. 내일 첫 실물 연결이므로 실제 로봇/손/센서/클램프를 확인하고 부품별로 진행합니다. 아래 예시의 IP와 파일은 현장 값으로 바꿉니다.

## A. 연결 정보 기록 — 운동 없음

로봇 Desk/현장 담당자로부터 **각 팔 IP, System Image/Desk 버전, FCI 상태**를 확인합니다. 서버 접속 IP가 아닙니다. 자동 IP 스캔이나 임의 네트워크 변경은 하지 않습니다.

```bash
./field-demo init --output site/demo
```

`site/demo/site.json`의 `robots.left.ip`, `robots.right.ip`를 실제 값으로 입력합니다. 설치 위치의 실행기 절대 경로는 init이 채웁니다. 나머지 실측값은 계속 미설정으로 남습니다.
이미 START_HERE 순서에서 `site/demo`를 생성했다면 init을 반복하지 않고 그 파일을 사용합니다. 기존 실측값은 덮어쓰지 않습니다.

```bash
./field-demo doctor --config site/demo/site.json --probe --output reports/first-connection.json
```

이 명령은 팔/손 상태만 읽습니다. 관절 상태·TCP·부하·프로토콜·손 폭이 나오는지 확인합니다. 버전이 맞지 않으면 전체 운동 실행을 시도하지 말고 진단 결과에 남깁니다.

## B. 현장 설정

`site/demo/site.json`을 실제로 측정해서 채웁니다.

| 설정 | 확보할 내용 |
|---|---|
| `T_cell_base` | 각 팔 base→공통 cell 변환. `p_cell=T_cell_base@p_base` |
| tool/load | 실제 장착 TCP와 손/공구·물체 하중. Desk 설정과 읽힌 값 대조 |
| `gripper_actions` | 각 부품의 실제 open/grasp/release 폭, 시험한 파지력·속도 |
| PLC | endpoint, 실제 정지/준비/이상 입력, 부품별 파지·안착·클램프·유지 신호 및 클램프 출력 |
| 기록 | `calibration.md`에 좌표/TCP/하중 측정, `commissioning.md`에 정지·경로 시험 기록 |

없는 신호를 상수 true로 채우지 않습니다. 실제 지그가 PLC/센서 방식이 아니라면 이 실행기의 전체 자동 모드는 그대로 사용할 수 없습니다. 관측 어댑터가 필요한 상태를 보고서에 남기고, 현재 지원되는 개별 시험 범위를 구분합니다.

전체 버튼 기능은 기본 프로필에 없습니다. 버튼 모드가 필요할 때만 별도로 신호·경로·스트로크를 검증합니다.

## C. 정지·손·작은 팔 이동

현장 하드웨어 정지 절차를 확인하고 시험 기록을 남깁니다. 손 동작은 물체/손가락 주변 상태를 확인한 후 명시적으로 실행합니다. 다음 예의 IP는 그대로 쓰지 않습니다.

```bash
./field-demo hand status --ip ACTUAL_ROBOT_IP --binary build/m80_franka_hand
./field-demo hand homing --ip ACTUAL_ROBOT_IP --binary build/m80_franka_hand --execute --clear-hand-area
```

standalone `hand`는 PLC를 감시하지 않는 수동 시험 도구입니다. 실제 파지 폭을 측정한 뒤 `hand grasp --help`로 입력 항목을 확인합니다. homing은 손가락을 열고 닫으므로 물체를 잡은 채 실행하지 않습니다.

좌표·하중·작업 공간·정지 조건을 실물에서 확인한 항목만 `commissioning`에 true로 기록한 뒤 작은 이동부터 시험합니다.

```bash
./field-demo run jog --config site/demo/site.json --log-dir logs/first-motion --side left --joint 7 --delta 0.01 --duration 3 --execute
```

별도 터미널에서 실행 중인 세션에 중단을 요청할 수 있습니다.

```bash
./field-demo run stop --config site/demo/site.json --execute
```

이 응답은 중단 요청 접수이며 물리적 정지 확인을 대신하지 않습니다. 실제 하드웨어 정지 수단은 별도입니다. Ctrl-C도 실행을 중단하며, 물체를 자동으로 놓지 않습니다.

## D. 부품별 정지 자세와 경로 기록

기본 순차 레시피는 큰 링 오른팔, 작은 링·핀 왼팔입니다. 양손 전달이 필요한 경우 아래 H 절의 독립 시험을 먼저 합니다. 기본 레시피의 팔 배정을 조용히 바꾸지 않습니다.

현장 가이드 모드 등으로 **각 경유 자세를 만든 뒤 가이드 모드를 종료하고 정지한 상태**에서 기록합니다.

```bash
./field-demo teach capture --ip ACTUAL_LEFT_IP --binary build/m80_read_state --output site/demo/poses/ring2-start.json
# 현장에서 다음 정지 자세로 이동한 뒤
./field-demo teach capture --ip ACTUAL_LEFT_IP --binary build/m80_read_state --output site/demo/poses/ring2-clearance.json
# 다시 실제 파지 직전 정지 자세로 이동한 뒤
./field-demo teach capture --ip ACTUAL_LEFT_IP --binary build/m80_read_state --output site/demo/poses/ring2-grasp.json
```

실제 다른 팔·손·바구니·지그·설치된 부품을 포함해 **보간 경로 전체**를 검토한 뒤 컴파일합니다.

```bash
./field-demo teach compile --config site/demo/site.json --stage left.grasp.ring2 --poses site/demo/poses/ring2-start.json site/demo/poses/ring2-clearance.json site/demo/poses/ring2-grasp.json --seconds-per-segment 10 --path-reviewed
```

이 명령은 관절 곡선을 만들고 10% 운동학 한계를 검사합니다. 충돌을 자동으로 검증했다는 뜻이 아닙니다. `--path-reviewed`는 실제 작업자의 전체 경로 검토 기록입니다.

필요한 기본 단계:

| 부품 | 단계 |
|---|---|
| 큰 링 | `right.grasp.ring1`, `right.insert.ring1`, `right.release.ring1` |
| 작은 링 | `left.grasp.ring2`, `left.insert.ring2`, `left.release.ring2` |
| 핀 | `left.grasp.pin`, `left.insert.pin`, `left.release.pin` |

`release` CSV는 같은 자세를 유지하는 정지 기록 두 개입니다. 실제로는 클램프 확인 후 손만 엽니다. **손 철수 경로는 다음 grasp 또는 별도 approach 단계에 포함**합니다. 마지막 핀 release 후에도 별도 철수 경로를 검토·실행하고 유지 상태를 확인합니다. 전체 시연 종료 조건은 손 개방만으로 대신하지 않습니다.

CSV 단독 시험은 팔만 움직이며 손가락과 클램프를 자동 조작하지 않습니다.

```bash
./field-demo run trajectory --config site/demo/site.json --log-dir logs/path-check --stage left.grasp.ring2 --execute
```

시작 자세가 다르면 실행을 거부합니다. 자동으로 초기 자세로 돌아가지 않습니다. 최초 기록한 시작 위치와 `q_d`가 맞는지 확인하고, 필요한 접근 경로를 별도로 기록합니다.

## E. 부품 하나씩 실제 시험

각 부품의 3단계와 실제 피드백을 설정한 뒤 먼저 오프라인 설정 검사를 합니다.

```bash
./field-demo run validate --config site/demo/site.json --part ring2
./field-demo run part --config site/demo/site.json --log-dir logs/ring2-001 --part ring2 --execute
```

파지→안착→클램프→손 개방→유지를 실제 피드백으로 확인합니다. `part`는 초기 개별 commissioning 도구이므로 전체 `assembly_commissioned`는 요구하지 않지만, 해당 팔의 현장 확인·경로 검토·피드백은 모두 요구합니다.

각 부품 시험 뒤에는 부품과 로봇 상태를 현장 절차로 정리합니다. 자동 리셋이나 무조건 재시도는 없습니다. 핀은 실제 끼움과 공차·파지 상태를 따로 확인합니다. 현재 실행기는 자동 접촉 탐색/힘 제어 압입 컨트롤러가 아닙니다.

## F. 카메라 기반 집기

[CAMERA_MOTION.md](CAMERA_MOTION.md)를 따릅니다. 원시 점군 수집→실측 대응점 보정→실제 초기 위치 주변 인식→실측 손/부품 관계→읽기 전용 모델 계획→실제 경로 검토→등록 순서입니다.

정해진 위치 근처의 CAD 추정과 무작위 바구니 전체 최초 검출을 구분합니다. 시뮬의 seed/정답 좌표를 실물 입력으로 쓰지 않습니다.

## G. 세 부품 연결 — 최종 권장 경로

[CYCLE.md](CYCLE.md)의 실측 레시피를 작성합니다. 집는 팔과 삽입하는 팔이 다르면 검증한 양손 전달 레시피를 연결합니다. `cycle`은 각 부품의 손 철수와 실제 TCP 거리, 마지막 0.5초 동안 세 부품 유지까지 확인합니다.

```bash
./field-demo cycle --config site/demo/site.json --recipe site/demo/cycle.json --validate
./field-demo cycle --config site/demo/site.json --recipe site/demo/cycle.json --log-dir logs/cycle-001 --execute
```

현장 측정·개별 실행·전체 경로 검토가 끝난 레시피만 실행합니다. 이것이 무작위 더미를 자동으로 발견하는 인식 기능을 추가하는 것은 아닙니다.

### 기존 고정 팔 레시피를 사용할 경우

9개 기본 단계의 개별 시험, 같은 팔의 연속 시작/끝 자세, 설치된 부품과 양팔 간섭, 마지막 손 철수까지 확인한 뒤에만 `assembly_commissioned`를 true로 기록합니다.

```bash
./field-demo run validate --config site/demo/site.json
./field-demo run assemble --config site/demo/site.json --log-dir logs/assembly-001 --execute
```

명령 성공은 순차 레시피의 센서 확인 결과입니다. 외부 카메라로 임의 배치를 자동 발견했다는 의미는 아닙니다. 최종 손 철수는 별도로 기록한 경로를 실행합니다. 시연 완료 시간은 실제 촬영/실행 로그로 측정합니다. 20/30초 목표를 위해 첫 현장 시험의 한계를 임의로 올리지 않습니다.

## H. 양손 전달이 필요할 때

`./field-demo handover --help`를 사용합니다. 전달 경로/받는 손 접근/주는 손 철수 경로, 실제 손 폭·힘, **서로 다른 실제 donor/receiver 파지 신호**, 전체 경로 검토 기록을 준비합니다.

```bash
./field-demo handover --config site/demo/site.json --recipe site/demo/handover-ring2.json --validate
./field-demo handover --config site/demo/site.json --recipe site/demo/handover-ring2.json --log-dir logs/handover-ring2-001 --execute
```

받는 손의 SDK 파지와 독립 PLC 파지 입력이 확인되기 전에는 주는 손을 열지 않습니다. 실패 시 양손을 임의로 개방하지 않습니다. 이는 **전달 단독 시험**입니다. 전달 직후 `run part --part ring2`를 실행하면 처음부터 손을 열기 때문에 사용하지 않습니다. 파지를 유지한 후속 삽입은 [CYCLE.md](CYCLE.md)의 레시피로 연결합니다. 기존 `run assemble`의 고정 팔 배정에는 자동으로 결합되지 않습니다.
