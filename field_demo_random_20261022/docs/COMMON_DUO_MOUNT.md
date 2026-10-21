# 공식 Duo 공통 마운트의 고정 베이스 처리

공식 FR3 Duo에서는 두 로봇의 link0가 하나의 강체 마운트에 고정됩니다. 이 설치된 고정 조립체 사이에 일반적인 팔 간 동적 여유를 요구하면, 실제 CAD가 떨어져 있어도 보수적인 캡슐·케이블 여유가 겹쳐 모든 자세를 거부할 수 있습니다. 실제348 기록에서 두 고정 베이스 CAD 간격은0.782mm였고, 캡슐+케이블+1mm 여유는4.850mm 겹쳤습니다. 추종 오차에 의한 팽창은 이 고정 link0에는0입니다.

이번 기능은 **같은 공식 공통 강체 마운트에 실제로 설치된 양쪽 link0↔link0만** 팔 간 동적 충돌 대상에서 제외합니다. 기본값은 꺼짐입니다. 독립 설치된 두 로봇에는 적용할 수 없습니다. link0↔움직이는 링크, 팔 자체 충돌, 반대 팔, Hand, 운반 중인 부품, 설비·바구니·상판의 검사와 원래 여유는 그대로 적용됩니다. 마운트 자체도 설비 장애물로 계속 모델링해야 합니다.

## 현장 설정 순서

기존 `model-preset` 작업으로 실제 FR3-v2·Franka Hand·마운트·케이블·부품 CAD 일치 기록과 SDK 상태를 먼저 준비합니다. 두 로봇 IP와 실제 셀 기준 베이스 좌표를 `site.json`에 넣은 뒤, 공통 마운트 기록의 초안을 만듭니다.

```bash
./field-demo common-duo-mount prepare \
  --motion site/random/random-motion.json \
  --site site/random/site.json \
  --output site/random/common-duo-proof.json
```

초안의 모델·상대 변환·해시는 자동 계산됩니다. 공식 `fr3_duo_mount_v0_3`와 실제 공통 강체 설치가 일치하는지 현장에서 확인하고, **그때만** 다음 필드를 작성합니다.

- `verified: true`, `actual_common_rigid_mount_installed: true`
- `by`, `at_utc`: 확인한 사람과 UTC 시간
- `evidence_file`: 실제 설치 확인 자료의 경로. 예제·시뮬 결과·합성 테스트 파일을 사용하지 않습니다.

원본 프로파일을 보존한 채 별도 프로파일을 만듭니다.

```bash
./field-demo common-duo-mount activate \
  --motion site/random/random-motion.json \
  --site site/random/site.json \
  --proof site/random/common-duo-proof.json \
  --output site/random/random-motion-duo.json
```

사용할 새 cycle recipe의 `planner_config`를 `random-motion-duo.json`으로 지정합니다. 기존5mrad 저속 프로파일을 쓰려면 이 새 프로파일을 바탕으로 기존 `low-speed-profile` 절차를 진행하고, **실제 양팔1mm/STOP 기록**을 새 프로파일 해시에 맞춰 준비합니다. 기본20mrad, 속도·가속도·jerk 제한은 바꾸지 않습니다.

## 자동으로 거부하는 조건

공식 모델 원본·생성 preset·실제 설치 기록·증빙·IP·베이스 좌표 중 하나라도 바뀌면 다시 준비해야 합니다. 두 베이스의 상대 변환은 고정 공장 모델과0.5mm/0.002rad 이내에서 일치해야 하며, 활성화 이후에는 실제 설정 좌표가 기록과 정확히 일치해야 합니다. 앞의 수치는 모델 식별 조건이며 로봇의 추종 오차·정지 성능을 보증하지 않습니다. 공통 마운트로 확인되지 않거나 실제 배치가 다르면 기능을 활성화하지 않습니다.

실행할 때마다 증빙 해시와 설치·모델·베이스 변환을 다시 검사합니다. JSON에 단순히 예외 플래그를 넣는 것으로는 검사에서 제외되지 않습니다.

## 검증 범위

합성 테스트에서 독립 로봇의 기본 거부, 실제 기록 바인딩, 잘못된 모델·설치·베이스 변경 거부, 움직이는 link1 및 비인접 링크의 충돌 유지 등을 확인했습니다.348 저장 시뮬 기록을 재평가하면 고정 베이스의 잘못된 거부는 사라지지만, UNKNOWN 또는 비인접 이동 링크 캡슐 검사에서 계속 멈춥니다. **현장 파지 경로·전체 작업 성공이나 실제 하드웨어 검증을 의미하지 않습니다.**

고정 모델은 `frankarobotics/franka_description`의 고정 커밋에서 가져온 Duo xacro/YAML과 함께 원본 라이선스를 동봉했습니다. 설치 기록을 자동으로 실제 검증된 값으로 표시하지 않습니다.
