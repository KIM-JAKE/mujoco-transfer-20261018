# GPU 물리 비교 결과 — 2026-10-22

**현재 성공본에서 GPU 물리만 켜는 방식은 시간 단축에 실패했다. CPU 성공본을 유지한다.**

물리 GPU와 GPU broadphase를 켠 비교에서는 작은 링을 잡고 들어 올리는 데 CPU/GPU 모두 성공했다. 동일한 시뮬 내부 1.502083초의 접촉 구간에서 다음 시간을 측정했다. 사전 준비와 파지 전 구간은 이 표에서 제외했다.

| 항목 | CPU | GPU |
|---|---:|---:|
| 물리 계산 실제 경과 시간 | 5.090초 | 140.221초 |
| 제어·기록 등을 포함한 구간 실제 경과 시간 | 5.416초 | 140.556초 |
| 작은 링 상승량 | 7.460cm | 7.377cm |

GPU의 접촉 구간 물리 계산이 **27.55배 느렸다**. 성공본540의 명령2864개를 그대로 재생했고, native Hand 결합·힘 제한·속도 제한·안전 콜백을 보존했다. 원본 컨트롤러/콜백 함수5개의 AST 동일성을 확인했다. 두 경우 모두 실제 링/손가락 접촉과 3cm 이상 상승 조건을 통과했다. 그러나 이것은 작은 링 파지·들어 올리기 구간1회씩의 결과다. **전달·두 링·핀 삽입·손을 뗀 뒤 fresh RGB-D 완료 검증을 포함한 전체3개 성공률은 확인하지 않았다.**

GPU broadphase의 영향도 짧게 분리했다. 동일한 첫0.5시뮬초의 물리 시간은 CPU 0.696초, GPU물리+GPU broadphase 27.347초, GPU물리+기존CPU MBP 27.380초였다. 이 짧은 대조는 접촉 성공 시험이 아니다.

시뮬 내부 동작 주기1920Hz, TGS128/64반복, CCD, native 원본 모델과 마찰, 힘·속도 제한, NumPy 상태 읽기, 기존 콜백을 유지했다. GPU1에서 단독 GPU 프로세스로 실행했고, CPU/GPU가 동일한 명령 바이트를 사용했다. GPU 활성 설정과 실제 GPU 사용을 확인했고, 물리 로그에서 CUDA 실패·CPU 전체 대체·버퍼 초과·접촉 폐기 경고는 발견하지 못했다. headless 그래픽 초기화 오류와 원본 Button quaternion 경고는 양쪽에 공통으로 존재하며 별도 기록했다. 일부 접촉·CCD·질의가 CPU에 남을 수 있으므로 전 과정이 GPU에서 처리된다는 뜻은 아니다.

고주파의 작은 계산 단계, 높은 solver 반복 수, CPU로 상태를 읽고 매 단계 완료를 기다리는 구조가 느려지는 원인 후보다. 각 원인의 기여도를 GPU 커널 수준에서 측정한 것은 아니다. NVIDIA도 수천 개의 활성 물체에서 GPU 이점이 크고, CPU 동기화 및 일부 CPU 처리 기능을 고려해야 한다고 설명한다.

성공본540·542·543의 핵심 파일51개와 코드·입력289개씩 총867개의 해시가 그대로임을 재확인했다. GPU2·3의 소스·큐·캐시·RPC는 수정하거나 사용하지 않았다. 공유 호스트의 다른 CPU 작업은 계속되어 완전히 전용인 서버의 벤치마크는 아니다.

속도 개선 후보가 되지 못하므로 이 설정의 전체3개 반복 시험으로 확대하지 않았다. 전체정책 CPU/GPU x 기준/PLUS/MINUS6개 실행기는 별도 복사본에 준비되어 있으나 실행하지 않았고 기존 성공본을 대체하지 않는다.

상세 원자료: `comparison.json`, `component-results/*/result.json`, `component-*.log`, `backend-adapter.diff`, `preserved-success-seals.json`.

공식 참고: [PhysX GPU Simulation](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/GPURigidBodies.html), [Isaac Sim Performance Handbook](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/sim_performance_optimization_handbook.html).
