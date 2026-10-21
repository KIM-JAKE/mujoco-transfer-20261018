# 카메라·좌표·실기 집기 연결

이 패키지는 실제 카메라 점군을 읽고 **측정 초기 위치 주변의 CAD 정합**으로 부품 자세를 추정합니다. 무작위 더미에서 부품을 처음 찾아 분리하는 검출기는 아닙니다. 현재 시뮬의 무작위 바구니 정책을 실물 검증 없이 대체하지 않습니다.

`motion plan`은 읽기 전용 실제 로봇 모델로 **집기 경로 후보**를 만듭니다. 장애물 경로 탐색, 연속 충돌 자동 판정, 자동 접촉 탐색/압입은 하지 않습니다. 실제 전체 경로를 검토·등록하고 최신 관측이 맞는 경우에만 저속 실행기에 넘깁니다.

## 1. 보정 전 카메라 확인

실제 RealSense serial을 확인합니다. 다른 모델의 카메라는 지원 어댑터가 필요합니다.

```bash
./field-demo camera list
./field-demo camera sample --serial ACTUAL_SERIAL --output logs/raw-001.npz
./field-demo preview --snapshot logs/raw-001.npz --output logs/raw-001.html
```

HTML을 현장 PC의 브라우저로 열어 화면과 유효한 깊이를 확인합니다. 픽셀을 눌러 SDK가 측정한 camera 광학 좌표를 읽을 수 있습니다. 원시 데이터는 `UNMEASURED`로 표시되며 실기 관측으로 실행할 수 없습니다. 광택 금속의 누락·가짜 깊이를 먼저 확인합니다.

## 2. 카메라→공통 좌표 보정

카메라에서 보이는 **고정 기준점**과 실제 공통 cell 좌표의 대응을 측정합니다. 같은 물리 점을 양쪽 좌표계에서 기록해야 합니다. 3점 이상 비공선 점이 필요하며 가능한 한 작업 영역에 퍼진 여러 점과 별도 검증점을 씁니다. TCP로 점을 측정할 경우 실제 TCP 접촉점의 오프셋을 반영합니다.

`site/demo/correspondences.json` 형식:

```json
{
  "units": "m",
  "points_camera_m": [[0.01, 0.02, 0.65], [0.10, 0.02, 0.65], [0.01, 0.12, 0.65]],
  "points_cell_m": [[0.41, 0.22, 0.05], [0.50, 0.22, 0.05], [0.41, 0.32, 0.05]]
}
```

위 숫자는 **파일 형식 예시**이며 현장값으로 사용하지 않습니다.

```bash
./field-demo calibration fit --correspondences site/demo/correspondences.json --output site/demo/camera-fit.json
```

결과의 `T_cell_camera`와 잔차를 검토하고 별도 점으로 방향과 오차를 확인합니다. 결과 파일 자체는 `measured_verified:false`입니다. 실제 확인한 기록을 남긴 뒤 `site/demo/camera.json`에 반영합니다.

필수값:

- 카메라 serial, `camera_frame`(RGB optical), `cell_frame`, 보정 ID.
- `T_cell_camera`: `p_cell = T_cell_camera @ p_camera`.
- 양팔 `T_cell_base`: `p_cell = T_cell_base @ p_base`. 로봇 설정과 동일한 실측값.
- `measured`: 측정자·시각·방법·원자료 파일·대응점 수·잔차.
- `nominal_stock_poses`: 실제 각 부품의 시작 위치 주변 초기 자세. 동작 중 집기 위치가 달라지면 다시 측정/인식해야 합니다.
- `part_robot`: 기본 고정 팔 레시피는 `ring1:right`, `ring2:left`, `pin:left`. 전달을 포함한 명시적 `cycle`을 쓰면 해당 부품의 `pickup_side`에 맞춥니다. [CYCLE.md](CYCLE.md)를 따릅니다.
- `pin_holder_tracks_pin`: 실제 홀더 관측 방식에 맞춘 명시적인 true/false.

```bash
./field-demo calibration validate --config site/demo/camera.json
```

보정 수학 검사가 통과해도 삽입 공차를 보장하지 않습니다. 실제 지그 정렬과 부품 공차를 별도로 확인합니다.

## 3. 부품 하나의 지속 관측

부품별 첫 시험에서는 한 부품만 선택합니다. 별도 터미널에서 계속 실행합니다.

```bash
./field-demo camera observe --config site/demo/camera.json --part ring2 --output site/demo/live-ring2.json --session-dir logs/observer-ring2-001
```

모델과 카메라를 한 번 초기화하고 새 프레임을 반복 처리합니다. `live-ring2.json`은 완성된 새 결과로 한 번에 교체합니다. 인식 실패·오래된 영상·중단 시에는 목표 없는 실패 상태로 바뀝니다. 오래된 성공 좌표를 계속 재사용하지 않습니다.

기본 신선도는 0.5초이고 실제 노출 시각·RGB/깊이 동기화를 확인합니다. SDK 전역/시스템 시각이나 노출 메타데이터를 확보하지 못하면 거부합니다. 여러 PC를 사용할 때는 UTC 시계를 맞춰야 합니다. 서로 다른 PC의 monotonic 시간을 비교하지 않습니다.

과거 점군만 검사할 때는 다음을 씁니다. replay 결과는 운동에 사용할 수 없습니다.

```bash
./field-demo observe estimate --snapshot logs/recorded.npz --config site/demo/camera.json --part ring2 --output logs/replay-ring2.json --replay
```

## 4. 실제 손과 부품 사이의 관계 측정

물체 중심과 손 TCP는 다릅니다. `config/templates/grasp.json`을 부품별로 복사해 실제 손으로 잡는 관계를 기록합니다.

`T_part_tcp`는 `p_part = T_part_tcp @ p_tcp` 방향입니다. 실제로 잡은 상태의 부품 자세와 읽힌 TCP를 함께 측정하면 다음 관계로 계산합니다.

```text
T_cell_tcp = T_cell_base @ T_base_tcp
T_part_tcp = inverse(T_cell_part) @ T_cell_tcp
```

SDK의 `O_T_EE_column_major`는 열 우선 배열이므로 행 우선 4×4 JSON으로 옮길 때 전치/reshape 방향을 확인합니다. 시뮬의 grasp offset을 실측값으로 표시하지 않습니다. 각 파일의 `part`, `robot`, 단위, 측정자·시각·증거를 기록합니다.

## 5. 읽기 전용 실제 모델 계획

```bash
./field-demo motion plan --observation site/demo/live-ring2.json --camera-config site/demo/camera.json --grasp-transform site/demo/grasp-ring2.json --part ring2 --output site/demo/trajectories/ring2-preview.json
```

위 명령은 목표 좌표만 미리 보여주며 로봇에 연결하지 않습니다. 실제 SDK 모델로 후보를 만들려면 다음을 사용합니다.

```bash
./field-demo motion plan --observation site/demo/live-ring2.json --camera-config site/demo/camera.json --grasp-transform site/demo/grasp-ring2.json --part ring2 --config site/demo/site.json --read-hardware --output site/demo/trajectories/ring2-candidate.json
```

팔은 정지 상태여야 합니다. 이 명령은 로봇 상태·모델을 **읽기만** 합니다. 실제 TCP·하중과 현장 설정을 대조하고, SDK 모델의 역기구학으로 시작→집기 목표의 10% 한계 관절 곡선을 검사합니다.

생성된 2점 곡선이 바구니나 다른 팔을 피한다는 보장은 없습니다. 우회 경유점이 필요하면 현장에서 기록한 접근 경로를 사용해야 합니다. 역기구학 해가 없거나 관절 한계를 넘으면 실행하지 않고 거부합니다. 손/물체의 안전한 접근·다른 팔 상태·손가락 개방을 포함한 전체 경로를 검토합니다.

## 6. 검토 후 등록·개별 시험

`site/demo/path-review.json`에 검토자·시각·정확한 candidate SHA256와 실제 전체 경로 검토 결과를 기록합니다. 파일 해시는 다음처럼 확인합니다.

```bash
sha256sum site/demo/trajectories/ring2-candidate.json
```

```bash
./field-demo motion register --candidate site/demo/trajectories/ring2-candidate.json --config site/demo/site.json --stage left.grasp.ring2 --review site/demo/path-review.json --observation site/demo/live-ring2.json
```

등록하면 전체 조립 승인은 false로 되돌아갑니다. 실행 직전에는 **새 관측이 등록한 목표와 일치**해야 합니다. 기본 허용치는 위치 1mm, 각도 0.01rad이며 공정 정확도 보증값이 아닙니다. 목표가 움직였거나 보정/CSV/검토 파일이 바뀌면 재계획·재검토가 필요합니다.

기록된 삽입 경로의 시작 관절 자세는 실제 파지 종료 자세와 이어져야 합니다. 새 카메라 목표로 집은 뒤 예전 삽입 경로를 무조건 재생하지 않습니다. 실제 파지 상태에서 이어지는 삽입/손 철수 경로를 기록·검토하고 부품 하나부터 시험합니다.

## 현장 준비가 필요한 나머지

- 실제 카메라 금속 깊이 품질과 인식 지연.
- 무작위 더미의 최초 검출과 바뀐 잡기 자세에 따른 경로 생성.
- 실측 레시피로 연결한 양팔 전달·삽입 경로의 실제 파지 유지 및 현장 검증.
- 실제 핀 공차, 지그 정렬, 힘 제어 압입.

이 항목들을 오프라인 테스트 성공으로 대신하지 않습니다.
