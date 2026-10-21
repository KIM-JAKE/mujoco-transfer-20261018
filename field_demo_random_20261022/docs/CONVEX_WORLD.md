# 선택적 전체 링크·palm 대 설비/UNKNOWN 상자 정밀 검사

기본값은 꺼짐입니다. 기존 capsule이 설비 또는 RGB-D 상자와 부딪힌다고 판정했을 때, 같은 원본 전체 링크 STL 또는 고정 palm STL을 포함하는 **닫힌 convex hull**로 같은 상자를 다시 검사할 수 있습니다. 모든 원래 SDK FK 샘플에서 형상이 떨어져 있어야 합니다. 원래 케이블·추종·시간 여유와 margin, 몸체 추가0.5mm를 모두 유지합니다.

UNKNOWN을 FREE로 바꾸거나 상자를 줄이지 않습니다. 부분 셀만 통과해서 병합 상자를 무시하지 않습니다. 손가락, 파지 물체, stock 접촉 규칙, 다른 팔과의 접촉 허용 조건은 바꾸지 않습니다. 양팔의 각 실제 SDK FK 문맥을 별도로 검사하며 원본 모델·설치·형상·좌표·이미지·CSV/프로파일 바인딩을 그대로 요구합니다.

실제 모델 starter와 설치 증빙을 먼저 완성한 뒤, 별도 의존성 `requirements-collision.txt`를 해당 호스트에 설치합니다. Python-FCL wheel이 기본 오프라인 wheelhouse에 포함됐다고 가정하지 않습니다.

```bash
.venv/bin/python -m pip install -r requirements-collision.txt
./field-demo convex-self-model --motion site/random/random-motion.json --output site/random/random-motion-convex-world.json --include-world-boxes
```

recipe의 planner_config를 새 프로파일로 명시적으로 연결합니다. 이 명령은 실제 설치 증빙을 대신 만들지 않습니다. 기존 공통 Duo 고정 마운트 증빙은 [별도 절차](COMMON_DUO_MOUNT.md)입니다. 기본20mrad를 유지하며 더 작은 값은 기존 실제 두 팔 STOP 시험·프로파일 절차를 통과해야 합니다.

실행 직전에도 같은 Python geometry_check를 재실행합니다. Site.motion은 공유 cell lock 안에서 다시 검사하고, Hand 폭 변경도 같은 함수로 검사합니다. native SDK 드라이버에 다른 capsule 검사를 몰래 건너뛰게 하지 않습니다. native 코드는 계속 CSV/C2/실제 시작 자세·추종·힘·heartbeat·STOP을 검사합니다. native 소스27개는 이전 봉인본과 같습니다.

저장된 수정 바닥 오버헤드 영상에서 동일4개 큰 링 후보·원본25°파지/손가락 대칭·명목0/조건부5mrad의48개 제안을 다시 검사했습니다. 명목 대칭 첫 자세에서 원래 robot/world 거부11개를 전체 hull로 안전하게 통과했지만 다음 link7 UNKNOWN 검사에서 멈췄습니다. 전체 끝 자세 생존0개(형상44개·IK4개 거부)로 전체 경로·이동·폐쇄는 실행하지 않았습니다. 0/5mrad는 **비실행 진단**이며 실제 설치·카메라·하드웨어 검증이 아닙니다.

6개 추가 회귀 검사는 원본 병합 상자 유지, hull 내부 UNKNOWN/실제 포함 충돌, 뒤쪽 FK 샘플 충돌, 더 큰 불확실성, 기본 비활성/동일 모델 바인딩, 변경 상자/cache, 손가락·payload 제외 및 실제 원래 충돌 검사 연결을 확인합니다. 관련59개 집중 Python 검사가 통과했습니다. [결과](../reports/convex-world-integration-status.json)를 확인하세요.
