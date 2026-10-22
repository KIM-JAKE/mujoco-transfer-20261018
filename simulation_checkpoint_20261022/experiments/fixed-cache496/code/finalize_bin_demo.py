"""Adopt only a measured under-30-second full physical episode."""
from pathlib import Path
import json,sys,collections,shutil,math,subprocess
HERE=Path(__file__).resolve().parent;RUNS=HERE.parents[1]/'runs';name=sys.argv[1];root=RUNS/name;r=json.load(open(root/'pipeline-results.json'));o=json.load(open(root/'pipeline-options.json'))
assert r['full_task_pass']and r['continuous_cycle_pass']and r['cycle_time_s']<30
assert not r['part_attachments']and r['workpiece_teleports_after_initialization']==0
counts=collections.Counter(row['part']for row in r['size_classification']);assert counts=={'ring1':5,'ring2':5}
peak=r['FR3_command_limit_audit'];checks=r['final_checks'];table='\n'.join('| '+label+' | %.3f mm | %.3f° |'%(checks[key]['position_error_m']*1000,math.degrees(checks[key]['axis_error_rad']))for key,label in [('ring1','큰 링'),('ring2','작은 링'),('pin','핀')])
notes=HERE/'bin-picking/PIPELINE_NOTES.md';archive=HERE/'bin-picking/PIPELINE_NOTES_before_under30.md'
if not archive.exists():shutil.copyfile(notes,archive)
notes.write_text(f'''# 혼합 바구니 → 링 2개·핀 1개 삽입

2026-10-20 채택 실행: **{name} — {r['cycle_time_s']:.2f}초에 연속 완료**.
Isaac Sim 6.1의 CPU PhysX에서 실제 접촉으로 집기·전달·삽입·놓기를 실행했다. 물체 부착과 실행 중 위치 순간이동은 없다. 큰 링 5개와 작은 링 5개는 같은 금속색이며, 그중 각 1개와 홀더의 핀 1개를 조립한다. 원래 Franka Hand 손가락 메시를 사용한다.

## 완료 기준과 측정

세 부품 중심 오차 1.5 mm 미만, 축 오차 0.05 rad 미만, 양손 루트가 모든 작업 부품에서 160 mm 이상 떨어진 상태가 이후 최소 0.5초 유지되는 최초 시각을 완료로 잡는다. 완료 {r['cycle_time_s']:.3f}초 후 {r['verified_elapsed_s']:.3f}초까지 유지 상태를 확인했다. 양팔 최대 접촉력은 {r['cross_arm_peak_contact_n']:.3f} N이다.

| 부품 | 최종 중심 오차 | 최종 축 오차 |
| --- | --- | --- |
{table}

이 시간은 **준비 자세에서 세 부품을 조립하고 손을 뗄 때까지의 시뮬레이션 동작 시간**이다. 오른손은 파지 목표에서 약 {o.get('prepared_hover_m',.13)*100:.2f} cm 위의, 기존 하강 경로로 검사했던 자세에서 시작한다. 부품은 모두 공급 위치에서 시작한다. 초기 위치로 복귀하는 시간, 카메라 처리와 오프라인 계획·렌더링 계산 시간은 포함하지 않는다. 반복 생산 전체 사이클이나 현장 속도를 검증한 값은 아니다.

## 동작과 해결한 문제

1. 큰 링은 지름 전체를 양쪽에서 집어 든다. 왼손은 아래에서 받아 삽입하고, 오른손은 다음 링 준비를 시작한다.
2. 바구니 벽 가까이 있는 작은 링은 가장자리를 집어 바닥 중앙 쪽으로 끌어온다. 놓은 후 현재 위치를 다시 읽고 지름 전체를 잡아 운반한다. 두 번의 위치 보정은 로봇 목표만 바꾼다.
3. 전달 중에는 오른손이 먼저 공간을 비운 뒤 왼손이 출발한다. 링을 놓을 때 파지 예압을 끄고 손가락을 완전히 열어, 이탈하며 링을 다시 빼내던 문제를 없앴다.
4. 왼손이 작은 링을 삽입하는 동안 오른손은 핀을 집는다. 실제 손 안 핀 자세를 기준으로 CAD 장애물을 검사해 삽입 직전 자세와 경로를 다시 찾는다. 핀은 수직으로 부분 삽입하고 클램프에 맡긴다. 손을 아래로 뺀 다음 기존 밀어넣기 동작으로 최종 삽입한다.
5. 자세 보정으로 바뀐 경로에서 명령 제한에 걸리는 구간만 다시 시간 배분한다. 실제 물리 실행은 항상 480 Hz로 진행한다.

## 명령 제한

위치에 따른 FR3 속도 한도의 95%를 기본 상한으로 검사한다. 채택 실행의 최대 명령 속도는 공식 위치 의존 한도의 **{peak['velocity_fraction_of_95_percent_PBV']*.95*100:.3f}%**, 가속도는 {peak['max_acceleration_rad_s2']:.3f} rad/s², jerk는 {peak['max_jerk_rad_s3']:.1f} rad/s³였다. 가속도·jerk 설정 상한은 각각 9.5 rad/s²와 4750 rad/s³다. 그리퍼는 손가락당 속도 47.5 mm/s, 힘 30 N 상한의 근사 구동 모델이다. 실제 로봇의 토크 변화율·추종·통신 성능은 미검증이다.

## 모델과 인식의 범위

- 크기는 색이 아닌 USD 메시에서 측정한다. 큰 링 외경은 약 74.90 mm, 작은 링은 약 61.91 mm다. 위치 보정에는 시뮬 내부의 정답 자세를 사용한다. RGB-D/SAM 기반 카메라 인식 검증은 없다.
- 바구니의 한 가지 혼합 배치에서 연속 완료했다. 무작위 배치 성공률이나 반복 신뢰성을 측정한 결과는 아니다. 핀은 3칸 홀더의 한 위치에서 공급한다.
- 클램프는 유한 힘의 대체 접촉 모델이며 현장 보정 전이다. 핀 충돌체는 4개 동축 원통이다. 명목 8 mm 축을 7.6 mm로 근사해 반경 0.2 mm의 여유를 두었으며 실제 슬릿·탄성을 재현하지 않는다.
- 원래 손가락 형상을 썼지만 구동·마찰·클램프는 보정되지 않았다. **실기에서 같은 속도와 성공을 보장하지 않는다.**

## 결과·영상·재실행

- [물리 실행 결과](../../../runs/{name}/pipeline-results.json)
- [실행 설정](../../../runs/{name}/pipeline-options.json), [시작 명령 경로](../../../runs/{name}/motion-commands.npz), [실제로 실행한 전체 명령](../../../runs/{name}/executed-actuator-commands.npz)
- [물리 상태 기록](../../../runs/{name}/physx-sequence-trace.npz), [소스 사본](../../../runs/{name}/code_snapshot)
- [1배속 영상](../../../runs/{name}-video/three-parts-under30-1x.mp4): 전체 장면과 작업 확대 화면. 물리 실행의 로봇과 세 작업 부품 상태를 재생해 CPU로 렌더링한다. 나머지 8개 링은 시작 배치로 표시하며 클램프 대체 형상은 그림에서 생략한다. 재생 스테이지의 좌표 쓰기는 물리 성공 증거로 사용하지 않는다.

현재 `pipeline-options.json`이 채택 경로를 가리킨다. 저장소 루트에서 아래 명령은 **시뮬레이션만** 새로 실행한다. 고유한 실행 이름을 사용한다. 실행 중 GPU를 노출하지 않는다.

```bash
OPENBLAS_NUM_THREADS=1 keti_m80_mujoco_20261013/.venv/bin/python \\
  isaac_port/native_hand_layout/roas-table/run_cpu_physx.py \\
  run_retimed_cached.py candidate native-roas-table-bin-my-check01
```

재실행에는 저장소의 기존 셀 자산과 `native-roas-table-bin-retimed14` 셀 캐시, `native-roas-table-bin-under30-capture01` 기준 기록이 필요하다. 실패·계획 후보를 포함한 실행 이력은 `RETIMING_STATUS.json`, 이전 설명은 `PIPELINE_NOTES_before_under30.md`에 남겨 두었다.
''')
active=json.load(open(HERE/'ACTIVE_STATUS.json'));legacy={k:active.pop(k)for k in ['ring_source_run','pin_source_run']if k in active}
if legacy:active['legacy_staged_source_runs']=legacy
active.pop('staged_assembly_pass',None);active['prepared_hover_m']=o.get('prepared_hover_m',.13);(HERE/'ACTIVE_STATUS.json').write_text(json.dumps(active,indent=2))
readme=HERE/'bin-picking/README.md';text=readme.read_text();marker='> 최신 연속 물리 실행:'
if marker not in text:text=marker+f' **{r["cycle_time_s"]:.2f}초 완료** ({name}). [현재 결과·모델 범위·재실행](PIPELINE_NOTES.md)을 먼저 확인한다.\n\n'+text;readme.write_text(text)
print('ADOPTED_PHYSICAL_RUN',name,r['cycle_time_s'])
