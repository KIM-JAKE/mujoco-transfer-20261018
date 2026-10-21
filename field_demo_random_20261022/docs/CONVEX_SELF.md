# 선택적 전체 링크 convex-hull 자기 충돌 정밀 검사

기본값은 꺼짐입니다. 공식 FR3-v2 링크 STL 전체를 포함하는 닫힌 convex hull로 **원래 같은 팔 capsule 검사가 거부한 링크 쌍만** 다시 검사합니다. 원본 형상을 줄이지 않습니다. 각 링크의 실제 케이블 여유와 모든 원본 열(column)의 최대 추종·시간 여유, 원래 충돌 margin을 유지하며 추가로 몸체마다 0.5mm를 더 둡니다. 다른 팔, 손가락, 물체, 설비, UNKNOWN 검사는 바뀌지 않습니다.

실제 설치와 공식 모델이 맞는다는 기존 model-starter 기록을 먼저 작성합니다. 이 선택 기능은 추가 `python-fcl==0.7.0.11` 의존성이 필요합니다. 해당 호스트에서 wheel 또는 소스로 별도 설치해야 하며 기본 오프라인 설치 wheelhouse에 포함됐다고 가정하지 않습니다.

```bash
.venv/bin/python -m pip install -r requirements-collision.txt
./field-demo convex-self-model --motion site/random/random-motion.json --output site/random/random-motion-convex.json
```

새 프로파일 경로를 recipe의 기존 motion 설정에 명시적으로 연결합니다. 기존 실제 설치/model 증빙과 SDK per-link FK가 없거나 모델/원본 STL/프로파일 해시가 달라지면 활성화하지 않습니다. 독립적인 두 팔을 공통 Duo 마운트로 가정하지 않으며, 고정 베이스 처리는 [별도 실제 설치 증빙](COMMON_DUO_MOUNT.md)이 필요합니다.

저장된 Isaac 348의 실제 encoder 자세에서 원래 link5/link7 열은 케이블 포함 0.486mm 침범으로 거부했습니다. 원본 STL 전체를 포함하는 hull 간격은 20.442mm이며 케이블 2mm씩·원래 margin 1mm·추가 수치 여유 0.5mm씩을 모두 유지해도 이 쌍은 통과합니다. 실제 SDK flange/EE 및 검증된 공장 마운트로 배치한 고정 palm의 원본 STL hull도 같은 방식으로 검사합니다. 동일 자세의 link5/palm 쌍은 모든 여유를 적용한 뒤 52.264mm이며 전체 자기 충돌 검사를 통과했습니다. 전체 끝 자세는 물체를 held로 제외하지 않은 진단의 손가락/부품 접촉 검사에서 계속 거부됐습니다. 기본 20mrad의 UNKNOWN 거부도 그대로입니다. 이 결과는 실기 접근·파지 성공을 의미하지 않습니다.

8개 집중 회귀 검사는 실제 저장 자세, 실제 충돌/포함, 경로 뒷부분 충돌, 모든 열의 최대 불확실성, 모델/형상/FK 변경, 원래 충돌 검사 연결을 확인합니다. 실제 로봇 명령은 보내지 않았습니다. [구체 결과](../reports/convex-self-integration-status.json)를 확인하세요.
