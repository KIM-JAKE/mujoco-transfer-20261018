# 양손 전달을 포함한 세 부품 순차 실행

이 실행기는 **현장 경로와 실측 피드백**을 연결합니다. 무작위 더미의 최초 검출이나 경로 자동 충돌 검증 기능을 제공하지 않습니다. 내일 실제 파지·끼움·시작 자세와 연속 경로를 먼저 확인해야 합니다.

## 1. 팔 배정과 경로

`init`이 만든 `site/demo/cycle.json`에 세 부품을 기록합니다. `parts`는 큰 링 `ring1`, 작은 링 `ring2`, 핀 `pin`을 각각 한 번 포함해야 합니다.

각 부품 항목:

| 필드 | 의미 |
|---|---|
| `pickup_side` / `insert_side` | 실제 집는 팔 / 삽입하는 팔, `left` 또는 `right` |
| `pickup_stage` | 해당 팔의 `*.grasp.<부품>` 경로 |
| `insert_stage` | 삽입 팔의 `*.insert.<부품>` 경로 |
| `release_stage` | 같은 자세를 유지하는 `*.release.<부품>` 경로 |
| `withdraw_stage` | 손을 연 상태로 지그에서 빠져나가는 `*.approach.<이름>` 경로 |
| `T_cell_part` | **조립 완료 위치**의 실제 부품 변환. 바구니 초기 위치가 아님 |
| `min_tcp_clearance_m` | 철수 후 실제 TCP와 부품 중심 사이 요구 거리. 최소 0.04m |
| `handover_recipe` | 팔이 다르면 필요한 별도 전달 레시피. cycle 파일 기준 상대 경로 |

`T_cell_part`와 손 거리 검사는 실제 TCP의 위치 검사입니다. 손가락·팔 전체의 충돌 여유나 보간 경로 검토를 대신하지 않습니다.

상위 `feedback_timeout_s`(0.1~5초), `feedback_dwell_s`(0.1~1초), `release_width_tolerance_m`(0 초과~0.002m)를 실제 신호와 손 응답에 맞게 시험합니다. 신호가 안 들어오면 시간을 무제한 늘리거나 성공으로 대체하지 않습니다.

예를 들어 오른팔로 두 링을 집고 왼팔로 삽입한다면 각 링은 `pickup_side:right`, `insert_side:left`이고 전달 레시피가 필요합니다. 핀은 실제 도달·파지·삽입이 확인된 팔을 지정합니다. 기본 `run part`의 고정 팔 배정을 바꾸는 대신 이 명시적인 cycle을 사용합니다.

## 2. 전달 단독 검증

`site/demo/handover.json`을 링별 별도 파일로 복사하고 다음을 채웁니다.

- donor/receiver 팔, 부품 종류.
- donor 운반/receiver 접근/donor 철수의 검토한 `*.approach.*` 경로.
- 실제 receiver 개방/파지와 donor 개방 폭·속도·힘.
- **서로 다른** 실제 `donor_grasped_signal`, `receiver_grasped_signal`. PLC 설정에 실제 주소를 추가합니다.
- 실제 시험한 피드백 대기·유지 시간과 폭 확인 오차.

전체 `cycle`에 쓰는 일반 `ring1_grasped`/`ring2_grasped`는 **어느 손이든 해당 부품을 유지 중**인 사실을 나타내는 실제 관측이어야 합니다. donor 센서만 연결하면 손을 넘기는 순간 이 신호가 꺼져 중단될 수 있습니다. 없으면 적절한 실제 관측 어댑터가 필요합니다. 상수 true로 우회하지 않습니다.

전달 단독 명령은 donor가 이미 해당 링을 실제로 잡고 있는 상태에서 시작합니다. 개별 집기와 초기 자세를 먼저 현장에서 맞춥니다. receiver는 비어 있어야 합니다.

## 3. 검토 파일과 해시 준비

모든 경로를 기록하고 실제 전체 보간 경로를 검토합니다. 자동 도구는 해시를 준비할 뿐 물리 검토를 대신하지 않습니다.

전달부터 준비합니다.

```bash
./field-demo review prepare --config site/demo/site.json --recipe site/demo/handover-ring2.json --output site/demo/handover-ring2-review.json
```

이 명령이 만든 검토 파일은 reviewer/시각이 비어 있고 확인 플래그는 false입니다. 실제로 경로 전체·손 개폐·다른 팔·바구니·지그·부품을 검토한 사람이 이름/UTC 시각을 적고 해당 사실을 확인한 뒤 봉인합니다.

```bash
./field-demo review seal --recipe site/demo/handover-ring2.json --review site/demo/handover-ring2-review.json
./field-demo handover --config site/demo/site.json --recipe site/demo/handover-ring2.json --validate
./field-demo handover --config site/demo/site.json --recipe site/demo/handover-ring2.json --log-dir logs/handover-ring2-001 --execute
```

각 링의 전달을 개별 확인한 뒤 cycle 파일의 해당 `handover_recipe` 경로를 지정합니다. 전체 cycle의 설정·경로·전달 파일 해시도 준비합니다.

```bash
./field-demo review prepare --config site/demo/site.json --recipe site/demo/cycle.json --output site/demo/cycle-review.json
# 실제 전체 순서·연속 시작/끝 자세·설치된 부품·손 철수를 검토한 뒤 검토 파일 작성
./field-demo review seal --recipe site/demo/cycle.json --review site/demo/cycle-review.json
```

현장 설정이나 경로를 바꾸면 이전 검토는 맞지 않으므로 다시 준비·검토해야 합니다. 특히 카메라 집기 경로를 등록하면 설정이 바뀌므로 **카메라 경로 등록을 끝낸 다음 전체 cycle 검토를 준비**합니다.

## 4. 카메라 집기와 함께 사용할 때

카메라의 `part_robot`은 해당 cycle의 **pickup_side**와 같아야 합니다. 같은 링을 왼팔로 삽입하더라도 오른팔로 집으면 카메라 목표는 오른팔 base로 생성합니다.

기본 고정 팔 배정과 다른 집기 경로를 등록할 때는 명시적인 cycle 파일을 넘깁니다.

```bash
./field-demo motion register --candidate site/demo/trajectories/ring2-candidate.json --config site/demo/site.json --stage right.grasp.ring2 --review site/demo/path-review-ring2.json --observation site/demo/live-ring2.json --cycle-recipe site/demo/cycle.json
```

집기 종료 자세, donor 운반, receiver 접근·파지, donor 철수, 삽입 경로가 실제로 연결되는지 확인해야 합니다. 새 잡기 자세와 옛 삽입 경로가 맞지 않으면 SDK 시작 자세 검사에서 거부합니다. 억지로 건너뛰지 않고 해당 경로를 다시 기록·검토합니다.

## 5. 전체 실행

```bash
./field-demo cycle --dry-run
./field-demo cycle --config site/demo/site.json --recipe site/demo/cycle.json --validate
./field-demo cycle --config site/demo/site.json --recipe site/demo/cycle.json --log-dir logs/cycle-001 --execute
```

- `--dry-run`은 실행 단계 계약만 표시하며 로봇·센서 연결이 없습니다.
- `--validate`는 설정/해시/네이티브 경로 한계 검사이며 로봇/PLC 상태를 읽지 않습니다.
- `--execute`는 모든 미설정 항목과 실제 관측이 갖춰진 상태에서만 진행합니다. 단일 단계 실패 시 중단하며 손을 임의로 열거나 재시도하지 않습니다.

실행은 부품별 파지→필요한 전달→삽입→안착 확인→클램프 확인→손 개방→유지 확인→손 철수의 순서입니다. 전달 후에는 받은 손을 다시 열고 집는 동작 없이 삽입으로 이어갑니다. 앞서 설치한 부품의 안착·클램프·유지는 이후 단계에서도 감시합니다. 마지막에는 세 부품의 유지 상태와 실제 손 철수를 확인합니다.

중단은 Ctrl-C 또는 같은 설정으로 `run stop --execute`입니다. 물리적 정지는 현장 장치에서 확인합니다.
