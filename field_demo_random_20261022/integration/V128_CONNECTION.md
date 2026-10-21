# v128 성공 기록과 실기 실행기의 연결

이 패키지는 `native-roas-table-scatter-speed-v128-closer-transfer-445`를 명시적으로 선택할 수 있다. 원본 실행 폴더와 이전 v119·V1·V2 성공 패키지는 수정하지 않는다. 선택은 성공 출처, 호환되는 인식 코드, 실제 실행기 버전을 묶는다. 시뮬레이터의 관절 기록을 실제 로봇에 재생하지 않는다.

## 확인한 성공 범위

원본 `pipeline-results.json`은 `full_task_pass:true`, `vision_end_to_end_pass:true`, `hardware_tested:false`다. 첫 카메라 촬영부터 최종 유지 확인까지 벽시계 **43.533158828970045초**이며 카메라·CPU 대기를 포함한다. 30초 미만은 아니다. 마지막 유지 관측은 약 0.548초다. 이는 seed 445의 한 시뮬레이션 결과이고 실물 시간·성공률을 의미하지 않는다.

핀은 시뮬레이션에서 축 지름 7.6mm, CAD 명목 8.0mm 사이에 반지름 0.2mm 여유를 적용한 근사 충돌 형상을 사용했다. 지그와 핀 끼움, 클램프 중심·스트로크는 현장 실측과 실물 확인이 필요하다.

## 실행기에 연결되는 것

| v128의 확인된 선택 | 실제 실행기에서 사용하는 대응 |
|---|---|
| 링 1 → 링 2 → 핀 | 같은 순서를 검사한 레시피 |
| 링은 주는 손 먼저 이동한 뒤 받는 손 접근 | 순차 전달 단계. 실제 양손 상태·독립 PLC 파지 확인 후 주는 손 개방 |
| 가까운 전달 지점 선호·받는 손 로컬 대기 | 현장에서 측정한 전달 자세와 회피 경유점. 시뮬레이션 좌표를 가져오지 않음 |
| 전달 후 새 영상으로 파지 관계 확인 | 실제 카메라와 실제 TCP에서 `G=T_tcp_part` 재측정 |
| 경로별 시간 계산 | 실제 SDK 모델과 현재 상태에서 다시 생성하고, 실기 한도·동적 충돌 증거를 검사 |
| 핀 직접 삽입·해제 후 유지 확인 | 같은 손으로 집어 삽입. 실제 착좌·클램프·해제·후퇴·최종 유지 확인 |

v128의 전달 탐색 카탈로그·로컬 대기 선택기·PhysX 경로 타이밍 최적화 코드는 **검증된 참고 소스**로 보존한다. 이 소스 자체를 실기 제어기로 실행하지 않는다. 현재 실기 실행기는 측정된 레시피 자세와 실제 SDK 경로 계획을 사용한다. `43.533초`, 시뮬레이션 속도 비율·30N 손가락 힘·시뮬레이션 카메라 이동값이 실기 설정으로 자동 적용되지 않는다.

## 선택과 변경 검사

`stabilization/v128/field-profile.json`은 성공 보고서·소스 매니페스트·참고 코드·RGB-D 파일과 실제 실행기 코드의 해시를 묶는다. 인식 핵심 두 파일은 기존 v119와 동일함을 확인했다. 다른 참고 파일의 변경은 `IMPORT.json`의 `changed_reference_files`에서 확인한다.

```bash
./field-demo stabilization check --profile stabilization/v128/field-profile.json
./field-demo stabilization list
```

현장 레시피의 `stabilization_profile`에는 선택 파일 경로와 정확한 SHA256을 넣는다. `init-random`이 채운 선택을 사용하거나, 별도 레시피를 연결할 때 같은 파일을 명시한다. 프로파일이 없으면 이전 레시피의 동작을 보존하며 v128이 활성화됐다고 표시하지 않는다.

선택된 레시피는 링 두 개의 전달, 직접 핀 삽입, 부품 순서가 같은 전략인지 확인한다. 코드는 어느 쪽을 주는 손·받는 손으로 사용할지 강제하지 않는다. 실제 팔 배정은 현장 레시피와 충돌 검증에 따른다.

실행기는 계획 전과 운동 직전에 프로파일 호환성을 다시 확인한다. 선택 정보와 의존성 해시는 새로운 관측 기록에 들어가며, 동적 계획 증거에도 포함된다. 원본 보고서·참고 코드·실행기·선택 프로파일이 변경되면 다음 운동을 거부한다. 새 안정화 결과가 나와도 자동으로 바꾸지 않는다.

## 실제 현장 입력

다음 값은 성공 기록에서 복사하지 않고 현장에서 측정한다.

- 실제 카메라별 보정·고정 시야와 양손 물체 가시성
- 로봇 base·TCP·하중, 손의 파지 변환과 폭·힘
- 전달·로컬 대기·회피 위치와 실제 지그의 삽입 목표
- 링 구멍을 보존하는 검증된 물체 외피, 로봇·손·케이블·주변 설비 외피
- 실제 PLC 파지·착좌·클램프·유지·정지 신호

기존 `random-cycle --validate`와 개별 현장 시험을 먼저 통과시켜야 한다. 프로파일 선택은 미보정 설정을 활성화하지 않으며 실기 인증을 만들지 않는다.

## RGB-D 참고 재생

원본에서 복사한 두 초기 NPZ 파일은 매니페스트 해시로 확인한다. 아래 명령은 해당 **시뮬레이션 촬영 영상**을 실제 인식 코드에 넣는 오프라인 검사다.

```bash
./field-demo random-vision detect --snapshot stabilization/v128/replay-frames/observation-001-initial.npz --config stabilization/v128/replay-camera.json --output /tmp/v128-initial-observation.json --replay
```

초기 촬영 두 장은 v128의 전체 관절 경로·전달·삽입을 실기에서 재현하는 파일이 아니다. 별도의 `replay/synthetic-contract`는 합성 영상과 가짜 장치로 동일한 상태 전이 계약을 검사하며, 원본 실험이나 실물 성공으로 표시하지 않는다.

v128 선택과 동일한 실기 상태 전이 엔진의 연결은 다음 합성 계약 재생으로도 검사할 수 있다.

```bash
./field-demo random-cycle --recipe replay/synthetic-contract/recipe-v128.json --dry-run --replay-manifest replay/synthetic-contract/manifest.json --output-dir /tmp/v128-profile-contract
```

24개 합성 RGB-D 패킷을 실제 인식 코드에 넣어 링 두 개 전달과 세 부품 단계를 통과했다. 결과는 `stabilization_profile:v128-closer-transfer-445`, `replay:true`, `motion_executed:false`다. 가짜 장치·계획기를 사용한 계약 검사이며 v128 전체 촬영 기록이나 실물 조립 기록으로 간주하지 않는다.
