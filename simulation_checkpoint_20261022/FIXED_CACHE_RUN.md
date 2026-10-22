# 저장 경로 시뮬레이션 재실행

이 안내는 **Isaac Sim 실행**용입니다. 실제 FR3/Hand/PLC를 움직이는 도구가 아닙니다. 서버에서 복원한 전체 CAD/생성 장면, 기존Isaac 컨테이너와bin-picking 의존 파일 및 GPU 서비스가 필요합니다. 현장 실제SDK 도구는 별도FIELD_RGBD 문서를 사용하세요.

현재 서버에서 정확한 성공 소스를 새 private snapshot으로 준비합니다. 기존 코드/실행 결과/장면은 덮어쓰지 않습니다. 아래`--base-repo`는 원래 생성 장면이 있는 작업 저장소입니다.

```bash
python3 simulation_checkpoint_20261022/tools/prepare_fixed_checkpoint.py \
  --base-repo /home/gpuadmin/workspace/jake/mujoco \
  --checkpoint simulation_checkpoint_20261022/experiments/fixed-cache479 \
  --run native-roas-table-my-cache-test
```

이 명령은 소스SHA-256을 확인하고 개인 snapshot과결과 폴더만 준비합니다. 실제479 소스·입력418개로 준비·재검증을 통과했습니다. 새 실행기 자체로 전체 재실행한 성과를 기존479와 혼동하지 않습니다.479는 기준 위치 성공,480은 같은 코드 PLUS 성공,481 MINUS는 핀 삽입 전 충돌 정지입니다. 최신 코드를 모든 위치에서 통과한 배포본으로 해석하면 안 됩니다.

예약한GPU가 해당 컨테이너에 배정되어 있고 기존 작업이 비어 있을 때, **다른 새run 이름**으로 같은 명령에 아래 옵션을 붙이면 Isaac이 시작됩니다. 현재 대화에서는 사용자에게GPU2·3 사용을 허가받았고, 공유 서버에서는 기존GPU 예약 도구를 먼저 사용합니다.

```bash
--container isaac-jake-gpu-3-9aeb439d20 --launch
```

기본 경로 매핑은host`/home/gpuadmin/workspace/jake`→container`/workspace/project`입니다. 다른 서버는`--host-mount`/`--container-mount`와설정의GPU socket 경로를 맞춰야 합니다. `trial-config.json`의camera/IK/route/pair socket은 현재 서버 서비스 주소입니다. 서비스 소스는checkpoint의`service-sources/`와기존GPU planning 패키지를 참조합니다. 일부 서비스 오류는 원래CPU 검사로 돌아가므로 실행 시간은 길어질 수 있습니다.

결과는원래작업 저장소의`isaac_port/runs/<run>/pipeline-results.json`에 생성됩니다. `full_task_pass`와`vision_end_to_end_pass`를 확인하세요. `state=completed_over_budget`는작업은완료했으나시간목표를넘겼다는뜻입니다. `.done`/프로세스정상종료만으로성공판정하지않습니다.

첫 촬영 전에 모델·장면·클램프 스트로크·보정·파지영역을 검사하고 저장 증명을 불러오거나 새로 생성합니다. 첫 준비 시간과 촬영 이후 전체 시간을 각각 보고합니다. 런타임은 새 RGB-D 접근/진입 구간과현재동적조건을 검사하고저장된고정구간을재사용합니다. 물체위치를바꾸거나부착하지않고, 중간LLM/agent 호출없이함수와조건분기로실행합니다.
