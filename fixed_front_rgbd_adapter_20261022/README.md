# 고정 공급대 RGB-D 카메라 관측 추가 패키지

기존608개 파일의 현장 패키지를 **새 디렉터리에 복사한 뒤**, 432에서 사용한 signed-face 링 검출기를 카메라 전용 명령으로 추가한다. 기존 배포본과 로봇 실행기는 수정하지 않는다. 이 폴더는 작은 소스 추가분이며218MB 기본 패키지를 Git에 다시 넣지 않는다.

```bash
python3 install.py \
  --base ../field_demo_random_20261022 \
  --output ../field_demo_fixed_native_camera
cd ../field_demo_fixed_native_camera
./field-demo verify
./field-demo fixed-native-vision --help
```

`--base`는 원본608 배포본이어야 한다. 설치기는 원본 CLI·608개 파일·추가 소스의 SHA-256을 확인한다. `--output`이 이미 있거나 원본 내부이면 거부한다. 새 디렉터리에서만 명령을 등록하고 전체 파생 소스 목록/체크섬을 갱신한다. 원본608 manifest와 체크섬은 `integration/base608-*`로 보존한다. 설치 후 기본 환경·SDK 의존성은 기존 설치 안내대로 새 환경에서 준비한다. 원본 `.venv`와 캐시는 복사하지 않는다.

사용법과 실제 보정값은 설치된 `docs/FIXED_NATIVE_RGBD.md`에 있다. 초기 위치를 모르는 링은 현재 RGB-D에서 크기·중심·축·앞뒤를 검출한다. 손에 든 링은 공개한 실제 SDK FK와 앞선 RGB-D 파지 관계로 관측 영역만 정한 뒤 새 영상에서 다시 추정한다. `--enable-live-camera`를 지정해야 카메라를 열며, 로봇·Hand·PLC에는 연결하지 않는다.

검증은 실제432의 저장 RGB-D4개를 새 설치본 CLI로 재생해 수행했다. 중심은 기존 결과와 같고 signed axis는 수치 정밀도 내에서 일치했다. 실제 카메라 또는 로봇을 연결한 성공·속도 검증은 아니다. 원본 기록은 이 작은 overlay에 중복 포함하지 않는다.

```bash
python3 tests/replay_actual432.py \
  --runtime ../field_demo_fixed_native_camera \
  --source-run /path/to/native-roas-table-fixed-front-v432-phase-offset-paired-native-all3 \
  --output /tmp/fixed-native-replay-evidence.json
```

추가 소스는 `src/`, `config/`, `docs/`, `tests/`와 설치기/manifest다. 테스트용 심볼릭 링크 overlay나 로컬 런타임은 배포하지 않는다.
