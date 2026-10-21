# 5 mrad 저속 시운전 프로파일

새 프로파일은 허용 추종 오차를20→5 mrad로 줄여 **더 일찍 정지**합니다. 실제 로봇이 이 오차 안에서 동작한다고 검증한 결과는 아닙니다. 기존20 mrad 설정은 보존합니다. 두 팔의 첫1mm 이동·STOP 시험은 실제 장비에서 수행해야 합니다.

IP·카메라/베이스/지그/바구니 좌표·실제 Hand/tool load·클램프 설정을 기존 온보딩대로 준비한 뒤 실행합니다. `--execute`가 없는 준비 명령은 로봇에 접속하지 않습니다.

```bash
./field-demo low-speed-profile prepare --source site/random/random-motion.json --output site/random/random-motion-low-speed.json
./field-demo commissioning-test --config site/random/site.json --tracking-profile site/random/random-motion-low-speed.json --side right --axis z --direction 1 --output-dir site/random/logs/first-right-5mrad --execute
./field-demo commissioning-test --config site/random/site.json --tracking-profile site/random/random-motion-low-speed.json --side left --axis z --direction 1 --output-dir site/random/logs/first-left-5mrad --execute
```

`axis/direction`은 **각 로봇 BASE 좌표**에서 실제로1mm 비어 있는 방향을 지정합니다. 이 시험은 기존의 안전 정지·공간·tool 확인 항목을 그대로 요구하지만, 미리 성공한 STOP 시험을 요구하지 않습니다. 같은 프로파일 JSON의5mrad 값이 실제 native 제어기, 경로 검증, 정지 상태 Hand 모니터에 전달됩니다. CSV의 실제 `q-q_d` 최대값과 명시적 STOP 반환·idle 상태를 확인하며, 실제 관측값이 없으면5mrad PASS를 만들지 않습니다.

두 결과가 모두 실제 PASS일 때만 별도의 ready 프로파일과 시운전 레시피를 만듭니다. 파일·바이너리·프로파일·telemetry 해시가 바뀌면 활성화가 거부됩니다.

```bash
./field-demo low-speed-profile activate --profile site/random/random-motion-low-speed.json --left-record site/random/logs/first-left-5mrad/stop-test.json --right-record site/random/logs/first-right-5mrad/stop-test.json --output site/random/random-motion-low-speed-ready.json
./field-demo low-speed-profile make-cycle --source site/random/random-cycle.json --profile site/random/random-motion-low-speed-ready.json --output site/random/random-cycle-low-speed.json
```

생성된 레시피는 픽셀로 고른 물체에 대해 최대24회,8개 기존 파지 자세×3개 IK 초기값을 검사합니다. IK 초기값은 팔에 보내는 관절 명령이 아닙니다. 실제 read-only SDK 모델에서 다른 해를 찾을 뿐이며, 실행 경로는 항상 현재 측정된 `q_d`에서 시작하고 기존 전체 로봇·반대팔·바구니·UNKNOWN·물체·속도·추종오차 검사를 통과해야 합니다. 후보가 모두 막히면 움직이지 않고 종료합니다.

실제 v280 시뮬 RGB-D와 SDK 모양의 로봇 모델을 연결한 현재 오프라인 캠페인에서는24개 중21개 IK가 풀렸지만 전체 외형/UNKNOWN 검사를 통과한 끝 자세는0개였습니다. 따라서 이 프로파일은 실제 시험을 시작하는 도구이며, 랜덤 바구니 파지나3개 삽입의 현장 성공을 인증하지 않습니다. 실제 하드웨어 실행은 이 서버에서 하지 않았습니다.

두 뷰 비교를 포함한 최신 제한은 [기록 파이프라인 결과](RECORDED_PIPELINE_STATUS.md)를 확인하세요. 다중 카메라 융합은 이 패키지의 live scene lease에 연결하지 않았습니다.
