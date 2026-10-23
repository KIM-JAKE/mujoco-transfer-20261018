# 실기 코드 경계

| 구성 요소 | 역할 |
|---|---|
| `fr3_capx` | 인식/좌표 도구 18개, 좌우 각각 제어 도구 13개, 단계 관리·로그·스킬 저장 |
| `capx_ring_tasks` | 실제 측정 관측·양팔 상태·손↔부품 관계·현장 CSV를 묶는 immutable 링 요청 |
| `hardware_v1` | 요청 등록·설정/소스/상태 결합, 실제 상태 읽기·Hand·PLC·카메라·현장 stage 연결 |
| `capx_vendor.fieldkit` | 기존 libfranka 상태 읽기·1 kHz stage·Franka Hand·Modbus/PLC 구현 |
| `capx_vendor.field_demo` | 기존 실제 RealSense 취득·보정·rectification 구현 |
| `capx_deploy` | 현장 설정 생성·소스 검증·바이너리 경로/해시 기록 |

링 정책 → 실제 RGB-D와 SDK 상태 → 현재 측정 G를 이용한 현장 목표 → 현장 planner/cuRobo 후보 → 전체 하드웨어 장면 검사 → exact CSV/요청/상태 승인 → 원래 libfranka stage 순서로 연결합니다.

공통 API는 임의 생성 Python을 실행하지 않습니다. 모델 클라이언트 등록은 모델 추론 검증을 뜻하지 않습니다. 현재 관측·경로·구동 준비 코드를 배포하며, GPU 서버와 실제 현장 장면을 묶어 제공하지 않습니다.

## 승인과 요청

task layer가 전달한 binding 자체는 실기 안전 증거가 아닙니다. backend는 현재 설정의 stage/CSV/side 및 전체 요청 해시를 확인하고, 실제 SDK와 Hand를 다시 읽어 현재 전체 장면 승인기에 보냅니다. 승인 결과가 요청에 결합돼야 하며, 승인 이후 상태가 달라지거나 CSV가 바뀌면 실행하지 않습니다. 기존 현장 제어기도 실제 시작 상태·한도·인터록을 확인합니다.

부품을 잡았다는 명령 응답과 실제 파지 확인, 삽입 명령과 실제 놓기·철수 후 완료 확인을 구분합니다. 코드를 설치하거나 CSV를 등록하는 것만으로 모터가 움직이지 않습니다. `capx-fr3`/`capx-hardware` 준비 CLI와 별도로 `capx-rings` 자동 두 링 진입점을 제공합니다. 실제 RGB-D/SDK/native planner 상태흐름과 새 BOTH final은 [자동 실행기](AUTOMATIC_RINGS.md)를 따릅니다. 실제 구동을 위해서는 현장 승인기와 장치별 검증을 마무리해야 합니다.
