# 현장 연결 순서

1. `pip install .`, `capx-fr3 verify-vendor`, `capx-fr3 init site-local`을 실행합니다. 초기 preflight는 미입력 항목을 모두 보고하고 장치에 연결하지 않습니다.
2. 두 FR3의 System Image와 설치된 libfranka 호환성을 확인합니다. 소스 디렉터리는 `capx-fr3 build-source`로 찾습니다. 이 디렉터리는 wheel에도 포함됩니다. 기존 장비 SDK를 임의로 최신 버전으로 교체하지 않습니다.
3. 실제 좌우 base, TCP, 지그, 부하를 측정하고 `site.json`, `hardware.json` 및 실제 보정 기록을 채웁니다. 시뮬레이션의 103.4 mm palm offset이나 command↔actual TCP 편향을 복사하지 않습니다.
4. 실제 카메라 시리얼·stream·K·camera→cell 변환을 측정합니다. `camera.json`에는 측정자·방법·시각·대응점 수·오차 및 비어 있지 않은 evidence 파일을 기록합니다. 측정 대응점 맞춤 도구는 `python -m capx_vendor.field_demo.calibration fit --help`로 확인합니다. 맞춤 결과는 실물 정확도를 별도로 확인해야 합니다.
   현재 adapter는 단일 RealSense stream입니다. 공급대·전달·지그·놓기/철수 후 단계별 실제 가시성을 확인합니다. 시뮬레이션의 고정 뷰 bank를 물리 다중 카메라로 구현한 상태가 아니며, runtime에 가상 카메라 extrinsic을 옮겨 관측을 통과시키지 않습니다.
5. 실제 PLC gateway와 ring1/ring2 clamp·grasp·seated·retained·cell-ready/fault/stop 신호를 매핑합니다. 출력 명령 전송과 실제 클램프·부품 상태 입력을 구분합니다.
6. 원래 C++ state/Hand/stage 도구를 빌드하고 `capx-fr3 pin-files --config site-local/hardware.json --build-dir build/fieldkit`으로 바이너리 해시와 경로를 기록합니다. 이 명령은 motor를 켜거나 보정값을 승인하지 않습니다.
7. `capx-hardware status ... --connect-read-only`로 실제 상태를 읽습니다. `capx-hardware camera ... --connect-read-only --output NEW.npz`로 카메라 한 묶음을 저장합니다. 두 명령 모두 이동·Hand 닫기·클램프 출력을 실행하지 않습니다.
8. 현장 planning adapter와 전체 장면 승인기를 연결합니다. 실제 양팔·손가락 폭·들고 있는 부품·이미 조립된 부품·미조립 물체·설비, 현재 actual↔command 시작 상태, 최종 발행 quintic 곡선의 FR3 한도를 검사해야 합니다. 외부 승인기는 현재 요청의 해시에 결합된 결과를 반환해야 합니다. 이 패키지는 대체용 항상-승인 검사를 넣지 않습니다.
9. 기존 현장 정지·하중·추종 검증과 저속 commissioning 절차를 거쳐, 큰 링과 작은 링 각각 및 연속 작업을 해당 실물에서 검증합니다. 시뮬레이션 성공률을 실물 성공률로 간주하지 않습니다.

현재 배포에는 8번의 **검증된 현장 전체 장면 승인기**와 실측 장비값이 없습니다. 따라서 설치·준비·조회 경로를 제공하지만, 로봇 IP를 입력하는 것만으로 자동 링 조립을 시작하지 않습니다. 누락 항목은 preflight에 표시합니다.

## 설정 수정 이후

`pin-files`는 `site.json`과 카메라 설정의 식별자를 다시 기록합니다. 드라이버 manifest와 현장 기록은 서로 다른 용도입니다. 드라이버 manifest는 설치한 코드의 동일성, 보정 기록은 현장에서 측정한 값의 출처를 나타냅니다. 파일이 바뀌면 이전 실행 객체나 요청을 재사용하지 않고 새 설정으로 다시 시작합니다.

설치본 또는 site 디렉터리를 옮겼다면 새 위치에서 `init`을 실행하고 측정 설정을 검토해 옮깁니다. 장치 IP·K·TCP·정지 조건은 패키지 배포 시 자동 추측하지 않습니다. 작업 로그는 `site-local` 아래에 두고 새 파일명을 사용합니다.

