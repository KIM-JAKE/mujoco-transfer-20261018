# FR3 Duo · native Franka Hand · RGB-D 현장 시험

현장 시험 시간: **2026-10-22 오전 11:00 한국 시간**.

이번 작업 패키지는 `field_demo_random_20261022/`에 있습니다. 기존 MuJoCo 복원 자료와 봉인 배포본은 그대로 두었습니다. RealSense RGB-D와 실제 로봇/Hand SDK 상태로 실행하며, 시뮬레이터의 물체 좌표나 별도 파지 센서를 요구하지 않습니다.

**현재 단계는 현장 보정과 저속 단일 파지부터 시작하는 실험 코드입니다. 랜덤 바구니에서 3개를 모두 조립하는 실기 성공이나 20초 완료를 검증한 배포본은 아닙니다.** 각 기능의 구현·검증 범위는 패키지 상태 문서에 구분했습니다. 시뮬레이션 시험 기록도 실기 시험으로 간주하면 안 됩니다.

## 시작

전체 MuJoCo 아카이브를 복원하지 않아도 현장 패키지는 독립적으로 사용할 수 있습니다. 기존 저장소가 있으면 최신 main을 받은 뒤 아래 패키지로 이동합니다. 새 설치에서 대용량 과거 아카이브가 필요 없으면 LFS 내려받기를 생략할 수 있습니다.

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/KIM-JAKE/mujoco-transfer-20261018.git
cd mujoco-transfer-20261018/field_demo_random_20261022
cat START_HERE.md
```

설치·로봇 IP·기준 좌표·RealSense 보정·Hand 및 실제 클램프 설정은 `docs/RANDOM_ONSITE.md` 순서로 진행합니다. 로봇을 연결하기 전 `./field-demo --help`를 볼 수 있습니다. 빌드한 실행 파일과 현장 설정을 확인한 다음, 정지 시험 → 단일 링 파지 → 전달/삽입 → 전체 순서로 시작합니다.

```bash
./field-demo init-random --output site/random --left-ip LEFT_IP --right-ip RIGHT_IP
# site/random 아래 파일에 현장 측정값 입력 후
./field-demo doctor --config site/random/site.json --probe
./field-demo commissioning-test --config site/random/site.json --side right --help
```

`commissioning-test --execute`는 작업자가 비어 있음을 확인한 **로봇 BASE 좌표의 1mm 방향**으로 10% 이하 이동하고, 실제 이동 기록이 확인된 뒤 SDK STOP을 보냅니다. 실제 정지·idle이 확인된 PASS 기록만 현장 `stop_tested` 확인의 근거로 사용합니다. 비상정지 인증 도구는 아닙니다. 양쪽 팔을 각각 확인하고 상세 절차에 따라 진행합니다.

## 실행 구조

RGB-D 취득 및 실제 SDK 렌즈 보정 → 링 영역/깊이 분석 → 크기·중심·축 추정 → 보정된 로봇 좌표 변환 → 파지 후보 및 충돌 검사 → 파지/재관측/양손 전달/삽입 스킬 조합 → RGB-D와 실제 Hand 피드백으로 유지·완료 확인.

실행 중 LLM agent를 호출하지 않는 프로그램 정책입니다. 가려져 확인할 수 없는 항목은 성공으로 처리하지 않습니다. 바구니 주변의 고정 장애물, 다른 팔, 실제 하중, 정지·통신 상실 조건을 유지합니다. 클램프는 실제 PLC 어댑터 또는 명시적인 작업자 조작을 사용합니다.
