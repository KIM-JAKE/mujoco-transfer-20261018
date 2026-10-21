# M80 · 기록된 실행 프로필과 현장 사용 범위

모든 프로필은 **`hardware_ready: false`인 시뮬레이션 검증 기록**이다. 시간은 초기 자세 이동·촬영·위치 추정·계획을 제외한 공정 시간이며, 실물 사이클은 측정하지 않았다. 세 부품의 동적 파지·삽입·유한 힘 클램프·손 해제와 버튼 100ms 유지를 검사했다.

| 기록 | 공정 시간 | 동일 초기 조건 반복 | 수입 모델 최대 토크 변화율 | 양팔 공유 구간 확인 | CSV의 연속 95% 검사 |
|---|---:|---:|---:|---|---|
| [최단 기록](../reports/contact-trim-03/summary.json) | 9.466667초 | 2회 | 6491Nm/s, 1000 초과 | 계획 진행도 | 추가 시간 배율 1.01에서 통과 |
| [모델 예산·자유 이동](../reports/model600-free800-01/summary.json) | 11.652083초 | 2회 | 847Nm/s, 통과 | 계획 진행도 | 배율 1.00에서 통과 |
| [모델 예산·겹침](../reports/model600-overlap-01/summary.json) | 11.889583초 | 2회 | 841Nm/s, 통과 | 계획 진행도 | 배율 1.00에서 통과 |
| [모델 예산·정지 확인](../reports/conservative-model600-release-01/summary.json) | 14.833333초 | 2회 | 915Nm/s, 통과 | 실제 관절 오차·속도 | 배율 1.00에서 통과 |

위 기록 모두 95% 명령 생성 예산과 실제 시뮬레이션 관절 위치·위치별 속도·가속도·jerk 검사를 통과했다. 최단 기록의 원래 C2 재보간 곡선은 순간 jerk가 95% 예산을 넘는다. 표의 1.01은 읽기 전용 SDK 수학 검사 입력을 1% 늦춘 값이며, 해당 배율의 새 물리 공정 시간이나 실물 시간을 검증한 것은 아니다.

11.889583초 경로는 원래 공간 경로에서 양팔 겹침을 늘리고, 접촉 구간에 정렬 확인을 적용했다. 14.833333초 경로의 실제 정지 체크포인트는 관절 오차 0.006rad·속도 0.03rad/s를 요구한다. 두 설정의 대기 조건이 다르므로 현장 `Cell`에 연결할 때 해당 구간 인터록을 확인한다. 정지 확인 대안의 충돌 근거도 표본 검사이며, 추종 오차와 모든 파지 부품 조합을 포함한 연속 안전 증명은 아니다.

동역학 값은 수입한 로봇·그리퍼 모델의 계산으로, 파지 부품·접촉·마찰을 제외한다. 실제 FR3의 하중 설정과 SDK 모델로 다시 검사한다. PhysX의 구동 응답, 정확한 무지연 물체 자세, 클램프·버튼 응답 등도 현장에서 확인해야 한다. 물리 센서 장착 위치는 유지했고 비물리 광축 표시 형상을 제외한 모델을 사용한다. 원본 CAD의 실물 조립 적합성 인증은 하지 않았다.

## 기록 복원

저장소 루트에서 다음 명령을 실행한다. 기록·입력·당시 코드의 SHA-256을 검사하고, 내용이 다른 기존 파일이 있으면 중단한다. 이 복원 명령은 로봇이나 시뮬레이션을 실행하지 않는다.

```bash
python3 isaac_port/archive_run.py restore isaac_port/reports/contact-trim-03/measured-evidence.zip
python3 isaac_port/archive_run.py restore isaac_port/reports/model600-free800-01/measured-evidence.zip
python3 isaac_port/archive_run.py restore isaac_port/reports/model600-overlap-01/measured-evidence.zip
python3 isaac_port/archive_run.py restore isaac_port/reports/conservative-model600-release-01/measured-evidence.zip
```

복원 위치는 `isaac_port/runs/<run_name>`이다. 반복 실행 기록은 각 보고서 폴더의 `repeatability.json`과 `measured-repeat1-evidence.zip`을 확인한다.

CAD·모델 공용 입력이 필요하면 `python3 isaac_port/bundle_assets.py restore`를 사용한다. 물리 재시험은 [전체 재현 절차](../CONCURRENT_TASK.md), 실제 로봇 상태 읽기와 SDK 모델 비교는 [FR3 온보딩](ONBOARDING_FR3.md)을 따른다.

## 현장 연결과 위치 변경

실제 로봇을 위한 진입점은 `policy.assemble_concurrent(cell)`이다. 현장 `Cell`은 실측 좌표·TCP·하중, 관측 기반 보정·접촉 탐색, 그리퍼·PLC·양팔 정지 처리를 연결해야 한다. 제공 C++ 단계 API는 검사된 고정 관절 곡선의 실행·감시용이며, Isaac의 물체 자세 보정이 포팅된 상태는 아니다.

공급 위치가 바뀌면 `M80Skill.prepare()`로 새 촬영에서 파지 경로를 적응하고 충돌·재타이밍·물리 시험을 다시 수행한다. 이 표의 시간은 새 위치 조건의 보장값이 아니다. 현장 설정과 시험 결과는 [기록 양식](COMMISSIONING_RECORD.md)에 남긴다.

## 자유 이동 시간 곡선 재생성

11.652083초 안은 접촉·파지·삽입·버튼 구간의 model600 시간 곡선을 유지하고, 핀 공중 이송과 오른팔 빈손 복귀만 model800으로 준비했다. 위 model600-overlap 기준 기록과 공용 seed를 복원한 뒤 계획 환경에서 실행한다.

```bash
/tmp/m80-plan-venv/bin/python isaac_port/prepare_free_motion_profile.py \
  --baseline isaac_port/runs/contact-model600-overlap-01 \
  --output isaac_port/generated/contact_task/site-model800-001 --model-budget 800
```

재생성된 모든 시간 곡선 배열이 측정 당시 입력과 일치하는 검사를 통과했다. 새 출력은 오프라인 계획 상태이며, 변경 조건과 실행 환경에서는 명목 충돌 검사와 전체 PhysX 검사를 다시 수행한다. `--free-time-scale`은 자유 이동 구간을 추가로 늦출 때 사용한다.
