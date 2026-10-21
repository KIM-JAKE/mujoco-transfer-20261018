# 현장에서 막혔을 때

| 증상 | 먼저 확인할 것 |
|---|---|
| `doctor` 종료 2 | JSON의 installation_checks, site.missing_values. 단순 미설정과 실제 연결 실패를 구분 |
| CMake/Eigen/SDK 없음 | Ubuntu22 x86_64는 offline build helper. 다른 PC는 맞는 설치 SDK prefix 사용 |
| SDK 버전/프로토콜 불일치 | Desk System Image와 설치 libfranka 확인. 전체 실행은 FR3 protocol 10 대상 |
| 실행 권한/실시간 환경 오류 | 현장 RT PC·FCI·rtprio/memlock 설정 확인. 진단 도구는 권한을 임의 변경하지 않음 |
| PLC 주소/신호 없음 | 실제 I/O 표 확인. 상수 성공으로 채우지 않음. 이 자동 레시피의 미지원 조건으로 기록 |
| 카메라 global time/노출 메타데이터 없음 | RealSense SDK·센서 지원과 USB 상태 확인. 신선도 검사를 끄지 않음 |
| 관측이 계속 stale | 지속 observe 사용, CPU 부하·추정 시간·UTC 시계 확인. capture 후 사람이 다음 명령을 입력한 파일은 대개 이미 오래됨 |
| 인식 실패 | 올바른 serial/좌표 보정/부품 종류/실측 초기 위치, 금속 깊이 누락 확인. 무작위 최초 검출 기능으로 오해하지 않음 |
| 시작 관절 q/q_d 불일치 | 현장에서 기록한 시작 자세 확인. 자동 초기 자세 이동은 없음 |
| TCP/하중/해시 불일치 | 실제 공구·Desk 설정·현장 경로 일치 확인. 경로 수정 뒤 다시 검토 |
| 핀 접촉 중 중단 | 실제 정렬·파지·공차 확인. 자동 force-search 기능이 아님; 한도를 풀어 밀어넣지 않음 |
| 중단 소켓 응답 없음 | 실행 중인 세션과 같은 site IP 설정인지 확인. 별도 현장 하드웨어 정지 절차 사용 |

## 중단

실행 터미널 Ctrl-C 또는 다른 터미널:

```bash
./field-demo run stop --config site/demo/site.json --execute
```

`stop_requested:true`는 요청 접수입니다. 물리 정지는 실제 장치에서 확인합니다. 소프트웨어 정지는 하드웨어 정지 회로를 대신하지 않습니다. 실패 시 손/클램프를 자동으로 열거나 전체 사이클을 재시도하지 않습니다.

실행 프로세스가 비정상 종료되어 오래된 소켓 파일이 남으면 실행 중인 프로세스가 없는지 확인한 뒤 해당 사용자 소켓을 정리합니다. 실행 도중 다른 설정으로 같은 팔을 동시에 명령하지 않습니다.

## 남길 파일

- `reports/arrival.json`, `first-connection.json`: PC/설치·실제 상태 조회.
- `site/demo/`: 실측값, 경로 CSV/해시, 보정/검토/commissioning 기록.
- `logs/<시험명>/`: 단계 이벤트 JSONL, native stdout/stderr, 관절·토크·외력 CSV.
- `logs/observer-*/`: 인식 원본 점군·프레임 메타데이터·추정/거부 결과.

실패한 시험도 원인을 확인할 수 있게 새 이름으로 보존합니다. `hardware_ready:false`는 설치 도구가 실물 인증을 주장하지 않는 표시이며, 개별 단계의 실제 실행 결과는 각 로그에 별도로 기록됩니다.
