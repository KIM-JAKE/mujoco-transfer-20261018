# 저장 경로 시뮬레이션 재실행

이 안내는 **Isaac Sim 실행**용입니다. 실제 FR3/Hand/PLC를 움직이는 도구가 아닙니다. 서버에서 복원한 전체 CAD/생성 장면, 기존Isaac 컨테이너와bin-picking 의존 파일 및 GPU 서비스가 필요합니다. 현장 실제SDK 도구는 별도FIELD_RGBD 문서를 사용하세요.

현재 서버에서 정확한 성공 소스를 새 private snapshot으로 준비합니다. 기존 코드/실행 결과/장면은 덮어쓰지 않습니다. 아래`--base-repo`는 원래 생성 장면이 있는 작업 저장소입니다.

```bash
python3 simulation_checkpoint_20261022/tools/prepare_fixed_checkpoint.py \
  --base-repo /home/gpuadmin/workspace/jake/mujoco \
  --checkpoint simulation_checkpoint_20261022/experiments/fixed-cache533 \
  --run native-roas-table-my-cache-test
```

이 명령은 소스SHA-256을 확인하고 개인 snapshot과 결과 폴더만 준비합니다. 현재 예시는533 PLUS 단일 성공 버전이며430개 소스·고정 입력의 실제 준비/해시 재검증을 통과했습니다. 이 준비 실행기에서 전체 엔진을 재실행한 검증과는 구분합니다.533은 저장 PIN846명령과 새 연결48명령을 실제 사용하고 놓은 뒤 새 RGB-D로 확인해 링2+핀1을216.80초에 완료했습니다. 같은 소스532 기준/534 MINUS는 삽입 부족 뒤 원래 충돌 검사로 실패했습니다. 세 배치·반복 안정성,20/30초·대기시간 대폭 감소 목표는 미달입니다. 과거494는 PIN 운반 폭 검사 누락이 있어 과거 재현용으로만 보존합니다.

예약한GPU가 해당 컨테이너에 배정되어 있고 기존 작업이 비어 있을 때, **다른 새run 이름**으로 같은 명령에 아래 옵션을 붙이면 Isaac이 시작됩니다. 현재 대화에서는 사용자에게GPU2·3 사용을 허가받았고, 공유 서버에서는 기존GPU 예약 도구를 먼저 사용합니다.

```bash
--container isaac-jake-gpu-3-9aeb439d20 --launch
```

기본 경로 매핑은host`/home/gpuadmin/workspace/jake`→container`/workspace/project`입니다. 다른 서버는`--host-mount`/`--container-mount`와설정의GPU socket 경로를 맞춰야 합니다. `trial-config.json`의camera/IK/route/pair socket은 현재 서버 서비스 주소입니다. 서비스 소스는checkpoint의`service-sources/`와기존GPU planning 패키지를 참조합니다. 일부 서비스 오류는 원래CPU 검사로 돌아가므로 실행 시간은 길어질 수 있습니다.

결과는원래작업 저장소의`isaac_port/runs/<run>/pipeline-results.json`에 생성됩니다. `full_task_pass`와`vision_end_to_end_pass`를 확인하세요. `state=completed_over_budget`는작업은완료했으나시간목표를넘겼다는뜻입니다. `.done`/프로세스정상종료만으로성공판정하지않습니다.

첫 촬영 전에 모델·장면·클램프 스트로크·보정·파지영역을 검사하고 저장 증명을 불러오거나 새로 생성합니다. 첫 준비 시간과 촬영 이후 전체 시간을 각각 보고합니다. 런타임은 새 RGB-D 접근/진입 구간과현재동적조건을 검사하고저장된고정구간을재사용합니다. 물체위치를바꾸거나부착하지않고, 중간LLM/agent 호출없이함수와조건분기로실행합니다.


490 외부 서비스는 GPU2의 `collision-gpu2-exact-dedup`과 GPU3의 `rgbd-gpu3-fast-spread`입니다. 바이너리 충돌 주소는 반드시 `pair-binary.sock`으로 끝나야 합니다. 서버의 원래 `pair-exact-dedup.sock`만 직접 지정하면 다른 전송 형식을 선택해 CPU 검사로 돌아갑니다. 현재 배포는 같은 폴더의 상대 symlink `pair-binary.sock -> pair-exact-dedup.sock`을 사용합니다. 서비스 소스·원래 커널·RGB-D 모델의 SHA-256과 실제 연결 검증은490의 `service-sources/`에 있습니다. 해당 서비스가 없거나 주소가 다르면 먼저 복원/경로 설정이 필요하며, 이 준비 도구가 서비스를 자동 생성하지는 않습니다.
