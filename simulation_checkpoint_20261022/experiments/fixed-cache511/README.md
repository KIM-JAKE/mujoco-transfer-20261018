# 511: 고정 기준 배치 — PIN 이전 영상 나이 검사에서 중단

510 PLUS와 동일420개 Python/427개 소스·고정 입력을 사용했으나 **51.04747초**에 원래 작은 링 `LoadedContactAperture` 바인딩의 영상 나이 검사에서 중단됐습니다. 측정된 capture age는물리시간0.7125초로 원래최대0.25초를 넘었습니다. 링2개·핀1개 전체 완료가 아니며 `full_task_pass=false`입니다.

PIN pickup 및 새 canonical seed 우선순위는 실행되지 않았습니다. 이 결과를 PIN branch 실패·성공으로 해석하지 않습니다. 원래 영상 나이·파지·충돌·동작 한도는 완화하지 않았습니다. 전체 완료 전 예외로 종료돼 최종 전체 명령 필드 한도 검사도 통과했다고 주장하지 않습니다.

선행 영상 요청이 필수 안정 파지 영상보다 앞서 대기한 스케줄을 조사 중입니다. 후속 수정·성공은 이 기록의 결과에 소급 적용하지 않습니다. 실제 로봇은 미검증이며 속도/반복 성공률을 입증하는 사례가 아닙니다.

원래 오류·실행 명령·관측·정확한 소스를 보존합니다. `canonical-pickup-qualification/actual511-failed-beforePIN.json`과 `pipeline-results.json`을 확인하세요. 관련510 통과 및508 실패는 각각 별도 폴더에 있습니다. 외부 서비스 소스는 현재495 패키지를 그대로 복사했으며 추가 근거는 `supplemental-source-evidence-manifest.json`으로 검증합니다.
