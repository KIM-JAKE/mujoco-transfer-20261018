# 510: 고정 PLUS 배치 — canonical PIN pickup 전체 통과

원래 Isaac Sim/PhysX에서 큰 링·작은 링·핀의 촬영·인식·계산·파지·전달·삽입·고정·손 철수·유지 확인까지 **212.11020초**에 전체 통과했습니다. 물리시간은49.58333초입니다.20/30초 목표는 미달이며 실제 로봇은 미검증입니다.

이번 opt-in 수정은 같은 최신 RGB-D 핀 접근 목표에 대해 기존 로봇 seed의 우선순위만 바꿉니다. 기존 taught seed의 음수 elbow 후보는 원래 경로 검사에서 사용할 경로를 찾지 못했습니다. 두 번째인 기존 catalogue pickup seed가 원래 전체 경로 검사를 통과해 선택됐습니다. 실제 접근 encoder q6=1.281954rad/q7=−1.287141rad이며 기존 GPU/CPU fallback을 사용하지 않았습니다. 임의 저장 좌표나 시뮬레이터 물체 pose를 제어에 넣지 않았습니다.

PIN 정밀 보정은3회 관측/2회 보정 뒤 원래75µm/3mrad/150µm 기준을74.7µm/1.580mrad/76.33µm로 통과했습니다. 공개 RGB-D+SDK로 측정한 손 상대 최대 변화는63.3µm/1.513mrad로 원래 유지 기준 이내였습니다. rebranch와 새 feedback reset은 실행되지 않아 그 효과는 검증하지 않았습니다.

전체23,800개 명령 및 실제 encoder 한도 검사가 통과했습니다. 원래 속도·가속도·jerk·47.5mm/s 손가락 속도·30N 힘 제한과 질량·마찰·충돌·센서·정밀 기준을 유지합니다. attachment/초기화 이후 teleport는 없습니다. 마지막 실제 손 철수 최소거리는182.028mm입니다.

동일420개 Python/427개 소스·고정 입력의511 기준 배치는 PIN 이전 작은 링 영상 나이 검사에서 중단됐습니다. 새 코드의 세 배치 통과나 반복 성공률을 주장하지 않습니다. 이전508 PLUS 실패도 별도 보존합니다. 병렬 공유 자원의 실행시간이므로 단일 수정의 순수 속도 개선량으로 해석하지 않습니다.

`pipeline-results.json`, `acceptance-summary.json`, `exact-source-manifest.json`, 명령·관측 원본과 `canonical-pickup-qualification/actual510-audit.json`을 함께 확인하세요. 외부 서비스 소스는495의 현재 패키지를 그대로 보존했고 `supplemental-source-evidence-manifest.json`으로 추가 근거의 SHA-256을 검증했습니다.
