# PIN 파지 직전 RGB-D 보정과 빈 반대팔 주차

이 기능은 **기본 꺼짐**이다. 실제 RGB-D/SDK, 기존 전체 로봇·부품·UNKNOWN·실제 Hand 폭·STOP·힘 검사와 native 10% 시운전 상한을 그대로 사용한다. 실기에서 파지 또는 전체 조립을 검증한 기능은 아니다.

Isaac의 별도418 시험은 24mm로 벌린 실제 native Hand로 접근하여, 새 끝면 영상 확인 → 파지 →70mm 들어올리기 → 빈 LEFT 주차 → 정밀 삽입 → 손 놓기 → 빈 RIGHT 후퇴를 실제 PhysX로 수행했다. 마지막0.542초 물리/0.668초 관측 구간에서 원래1.5mm/.05rad/160mm 조건을 통과했다. 이는 **한 PIN 시뮬 구성 요소**의 결과이며 여기 field SDK 경로의 실기 성공이나30초 이내 전체 작업 근거가 아니다.

## 현장 설정

기존 START_HERE의 실제 좌표·tool/load·STOP 시험을 먼저 설정한다. `random-camera.json` 또는 실제 `camera_configs.pin_holder` 설정에서 `pin_pose_backend`를 `measured_cap_circle`로 선택한다. 실제 고정 카메라가 PIN 끝면을 봐야 하며 카메라를 자동 이동하지 않는다. 가려진 끝면을 예측 XY나 ICP로 대신하여 통과시키지 않는다.

`random-cycle.json`의 PIN 항목에는 다음이 기본 비활성으로 생성된다.

```json
"pin_pickup_feedback": {
  "enabled": false,
  "max_corrections": 3,
  "gain": 0.5,
  "other_empty_park": {
    "enabled": false,
    "phase": "pin_empty_park",
    "target_offset_base_m": [0.20, -0.12, 0.45],
    "measurement": {"verified": false, "by": null, "at_utc": null, "evidence_file": null}
  }
}
```

상위 `enabled`만 켜면 파지 직전 보정을 사용한다. 모든 PIN family의 실제 `open.width_m`는 **24mm 이상**이어야 한다. 기존 model starter는80mm를 생성하므로 자동으로24mm로 축소하지 않는다. 선택한 파지 family/Hand action은 실제 부품·TCP에 맞춰야 한다. 현재 기본13mm grasp와 upright family는 미검증 후보이며418의 비스듬한 파지 family를 자동으로 복제하지 않는다. field CAD의 원래8mm PIN shaft를 사용하며 시뮬7.6mm 근사를 실기 치수로 쓰지 않는다.

각 보정은 새 RGB-D 끝면 원·축과 실제 SDK actual/desired TCP로 계산한다. 위치0.3mm/회전0.01rad 안에 들어와야 닫을 수 있다. 최대3회, 회당2mm/0.05rad 안의 반응만 허용하며 보정 gain 기본0.5다. 축 주위의 관측할 수 없는 yaw는 이전 영상의 표현을 유지한다. 원래 계획기에서 모든 보정 경로를 만들고, 실제 Hand aperture sweep 직전에 새 영상/SDK로 같은 오차를 다시 확인한다. 실패하면 손을 닫지 않는다.

## 빈 반대팔 주차

`other_empty_park.enabled`를 추가로 켜려면 실제 셀에서 해당 한 점과 통로를 확인한 기록을 연결한다. `target_offset_base_m`는 **반대팔의 실제 보정된 base 좌표계**에서의 목표 TCP 위치다. 예시 수치를 그대로 실측값으로 표시하지 않는다. 손 방향은 현재 실제 SDK 방향을 유지한다. `random-motion.json`의 전용 `pin_empty_park` phase에도 실제 통로·5N/1Nm STOP 시험 기록이 필요하다. 기존 실제 workspace 기록을 통로 근거로 재사용할 수 있다.

이동 전 반대 Hand의 SDK가 `is_grasped=false`, 실제 폭78mm 이상이고, 현재 expected-held/load 소유권이 비어 있어야 한다. RIGHT가 든 PIN의 실제 RGB-D 기반 형상·자동 setLoad는 계속 유지한다. 계획/실행은 원래 전체 검사와 재관측을 거친다. 실제 주차에 도착한 뒤 PIN을 다시 관측하고 **현재 실제 SDK에서 원래 preinsert 계획이 가능한지** 확인한다. 가상의 미래 SDK 상태를 만들거나, 전체 삽입/고정까지 미리 증명했다고 표시하지 않는다. 막히면 PIN을 유지하고 중단한다.

## 실행과 범위

```bash
./field-demo random-cycle --recipe site/random/random-cycle.json --validate
./field-demo random-cycle --recipe site/random/random-cycle.json --single-pick pin --execute
```

`--single-pick pin`은 실제 SDK 파지·기존 새 held 영상·하중 전환·lift 후 확인을 통과한 PIN을 들고 멈춘다. 반대팔 주차/삽입은 전체 cycle에서만 실행한다. 현장에서 먼저 이 단품 모드로 실제 파지를 확인한다. 기존 파지 직후 held 영상이 가려지면 **lift 전에 거부**할 수 있다. 이 field 기능은 SIM의 SDK만으로 임시 lift를 허용하는 분기를 포팅하지 않았다. 현재 field는 fresh held-G/load 증명을 유지한다. 실제 고정 held-view를 조정하거나 관측 조건을 해결해야 하며 가려진 상태에서 성공을 꾸미지 않는다.

전체 실행은 기존 `random-cycle --execute`다. 반대팔이 실제 다른 부품을 잡고 있으면 주차하지 않는다. STOP/카메라 오류 시 현재 완료된 grasp를 유지하는 원래 정책을 사용한다. UNKNOWN 검사에서 접근이 막히는 기존 field 한계는 이 기능으로 제거하지 않는다.

검증: 새11개 mocked RGB-D/SDK 경계·실행순서 시험, 기존11개 cycle/10개 PIN precision 회귀. 실제 로봇/RealSense 명령은 보내지 않았다.

재생 가능한418 원본 RGB-D 한 장도 포함했다. `PYTHONPATH=src python3 tools/replay_pin_pickup418.py`로 공개 field 끝면 검출기와 보정 수식을 재생한다. 관측 원·축이 원본 RGB-D 추정값과 일치했고 CPU 측정 약0.25초였다. 고정 실기 카메라 가시성·전체 경로·실기 정확도를 증명하는 시험은 아니다.
