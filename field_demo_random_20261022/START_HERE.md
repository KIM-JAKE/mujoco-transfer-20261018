> 이번 후보 탐색 수정: [같은 크기 링을 번갈아 검사](docs/PICKUP_SEARCH.md)합니다. 한 링에 IK 초기값 네 개를 연달아 쓰기 전에 다른 영상 후보를 먼저 확인합니다. 5개 링·초기값 4개·계획 예산16회의 오프라인 경계 검사를 통과했으며, UNKNOWN과 실제 이동/파지 검사는 그대로입니다. 실기 성공을 검증한 결과는 아닙니다.

> 새 카메라 선택 기능: [핀 끝면 원·축 직접 RGB-D 측정](docs/PIN_CAP_RGBD.md). 기본 ICP 유지, 원주가 실제로 보일 때만 측정하며 가림 시 예측 XY/ICP fallback 없이 거부합니다. 원본 시뮬 영상 네 장을 공개 검출기로 재생했습니다. 실제 RealSense·고정 시야·조립은 미검증입니다.

> 새 선택 기능: [핀 삽입 전 정밀 피드백](docs/PIN_PRECISION.md). 기본 비활성, 실제 SDK·새 RGB-D·기존 경로/힘/정지 검사를 유지합니다. 저장된 시뮬 변환식 재생만 확인했으며 현장 ICP 정밀도·파지는 미검증입니다.

# 10월 22일 오전 11시 KST 현장 시험 — RGB-D + Franka SDK

이 패키지는 현장에서 **설정 → 실제 카메라 인식 → 읽기 전용 SDK 확인 → 첫 1mm 이동·STOP → 저속 단품 시험**을 시작하는 코드입니다. 실제 로봇의 랜덤 바구니 파지·링 2개/핀 1개 조립 완주는 아직 검증하지 않았습니다. 본 서버에서는 로봇 명령을 보내지 않았습니다.

이번 변경은 실제 기록을 연결하면서 발견한 실행 문제를 고쳤습니다.

- 4mm 바구니 격자 205,632개가 기본 예산을 넘던 문제를 고쳐 명시적 기본 예산을 1,000,000개로 설정했습니다. UNKNOWN/FREE 판정과 3mm 깊이 여유는 그대로입니다.
- 선택한 링의 표면 검사에서 실제 후보 픽셀만 CAD와 비교합니다. 같은 기록에서 결과 바이트를 유지하면서 해당 격자 작성이 1.771초에서 0.562초로 줄었습니다. 전체 공정 시간이나 실기 성능 수치는 아닙니다.
- 선택적 **5mrad 저속 시운전 프로파일**과 실제 SDK 모델의 **파지 자세·IK 초기값 대안 탐색**을 추가했습니다. 기존 20mrad 기본값은 유지합니다. [시운전 프로파일](docs/LOW_SPEED_COMMISSIONING.md)을 따르세요.

## 먼저 설치하고 설정하기

Ubuntu 22.04 / x86_64 / Python 3.10, 컴파일러와 make가 있는 PC에서:

```bash
chmod +x field-demo scripts/*.sh
./field-demo verify
scripts/install.sh --with-realsense --offline --wheelhouse vendor/wheelhouse/cp310-linux-x86_64
scripts/build-offline-ubuntu22.sh
scripts/offline-check.sh
./field-demo init-random --output site/random
```

다른 호스트는 [설치 안내](docs/INSTALL.md)를 확인합니다. [모델 기반 starter](docs/MODEL_STARTER.md)로 공식 FR3-v2/Franka Hand·부품 CAD 형상을 작성하면 링크와 손 모양을 하나씩 측정할 필요가 없습니다. 실제 설치와 맞는다는 기록 한 개, 실제 양팔 SDK 상태가 필요합니다. 모델 파생 수치를 현장 실측값으로 표시하지 않습니다.

현장에서 넣어야 하는 값은 실제 양팔 IP, 로봇 기저/카메라/바구니/지그 좌표, 실제 Hand/tool/load, 케이블·고정 설비와 통로, 클램프 방식입니다. null을 가짜 확인 값으로 바꾸지 말고 [현장 온보딩](docs/RANDOM_ONSITE.md)을 순서대로 따르세요. 부품별 파지·착좌 PLC 센서는 요구하지 않습니다. 실제 RGB-D와 Hand/로봇 SDK를 사용하며, 클램프는 실제 PLC 또는 명시적 수동 방식입니다.

## 현장 실행 순서

1. `site/random` 설정과 [모델 starter](docs/MODEL_STARTER.md)를 완성합니다.
2. [RGB-D 인식과 읽기 전용 상태](docs/RANDOM_ONSITE.md)를 확인합니다.
3. 실제 빈 방향으로 양팔의 첫 1mm 이동·STOP 시험을 합니다. 더 엄격한 5mrad 프로파일은 [별도 절차](docs/LOW_SPEED_COMMISSIONING.md)에서 만들고, 실제 두 팔의 PASS 기록과 해시로 활성화합니다.
4. 설정 검증 후 `--single-pick ring1`부터 저속으로 시험합니다. 후보가 모두 막히면 움직이지 않고 종료합니다.
5. 파지·단일 물체 확인·전달·삽입·클램프·손을 놓은 뒤 유지 확인을 현장에서 각각 확인한 다음 전체를 시험합니다.

native 실기 실행 상한은 10%입니다. 시뮬 95% 속도나 과거 43.533초 기록을 실기에 적용하지 않습니다. 현장 경로는 현재 SDK 상태에서 다시 만들며 시뮬 관절 CSV를 재생하지 않습니다. GPU/cuRobo의 최신 Isaac 정책 전체를 이 패키지에 포팅한 것은 아닙니다.

## 현재 기록으로 확인한 한계

실제 저장된 Isaac RGB-D를 현재 검출기·모델 형상·5mrad 추종 여유와 연결했습니다. 오버헤드와 글로벌 **가상 카메라 두 대**의 같은 장면을 융합하면 UNKNOWN 4,573개가 실제 ray 근거로 FREE가 됐지만, 24개 후보 중 전체 로봇 끝 자세 검사를 통과한 후보는 0개였습니다. 가장 가까운 UNKNOWN 셀은 병합 상자가 만든 가짜 구멍이 아니라 실제 미확인 셀입니다. 따라서 이 기록에서 접근·Hand 폐쇄·파지 성공을 주장하지 않습니다.

[기록된 파이프라인 검사](docs/RECORDED_PIPELINE_STATUS.md)에 결과와 재생 방법이 있습니다. 두 뷰 융합은 **오프라인 구성 요소 시험**이며 live 다중 카메라/scene lease 기능이 아닙니다. 기존 scene lease도 기본 비활성이며 [별도 조건](docs/STATIC_SCENE_LEASE.md)을 따릅니다. 과거 v128 자료는 출처를 보존한 시뮬 참조이며, 이 패키지의 실기 성공 근거가 아닙니다.
