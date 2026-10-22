# 512: 고정 기준 NOMINAL 배치 — camera-priority/canonical PIN 전체 통과

원래 Isaac Sim/PhysX에서 촬영·인식·계산·두 링과 핀의 파지·전달·삽입·고정·손 철수·유지 확인까지 **214.48711초**에 전체 통과했습니다. 물리시간48.48750초입니다.20/30초 목표는 미달이며 실제 로봇은 미검증입니다.

원래 작은 링 receiver 바인딩을 완료한 뒤 다음 큰 링 초기 영상 요청을 보냈습니다. 같은 요청의 순서만 바꾸며 새 프레임·대기·속도/힘 변화나 영상 나이 기준 완화는 없습니다. 실제 바인딩의 영상 나이는0.131250물리초로 원래최대0.25초 이내입니다. 원래511 영상 나이 중단은 별도 보존합니다.

PIN의 기존 canonical seed 우선순위에서 `catalogue.pickup_q`가 원래 전체 경로 검사를 통과했습니다. 실제 접근 q6=1.281772rad/q7=-1.287174rad입니다. 원래 정밀 기준75µm/3mrad/150µm를3회 관측 뒤 37.31µm/0.5335mrad/shaft38.21µm로 통과했습니다. 재분기 후 feedback reset은 실행되지 않았으므로 그 효과는 검증하지 않았습니다.

저장 경로 실제 승인: SMALL donor=True, SMALL LEFT720=True, LARGE canonical1415=True. 저장 경로 미승인은 원래 전체 검사를 거친 재계획을 사용하며 저장 재사용으로 세지 않습니다. 현재 파지·반대 팔·놓인 부품·실제 encoder/명령·전체 손가락 너비 및 최종 삽입 검사는 유지됩니다.

전체23,274개 명령 필드와 실제 encoder 한도 검사가 통과했습니다. 속도·가속도·jerk·47.5mm/s 손가락 속도·30N 힘·질량·마찰·원래 충돌/센서/유지 기준을 바꾸지 않았습니다. attachment·초기화 뒤 teleport는 없습니다.

동일420개 Python/427개 소스·고정 입력의512 NOMINAL/513 PLUS 결과를 기록하며 MINUS514는 아직 대기 중입니다. 이 기록은 세 배치 통과·반복 성공률·임의 적층·실기 성공이나 순수 단일수정 속도 개선을 주장하지 않습니다. 이전 실패508/511도 각각 보존합니다.

`pipeline-results.json`, `acceptance-summary.json`, `camera-priority-qualification/actual512-audit.json`, 정확한 소스 및 원래 명령·관측을 함께 확인하세요. 외부 서비스 소스는495의 현재30개 파일 패키지와 byte 단위로 같으며 추가 근거는 `supplemental-source-evidence-manifest.json`으로 검증했습니다.
