# 자기 Hand가 RGB-D 장애물로 보이는 경우

이 작업본의 추가 기능은 기본 비활성입니다. 실제 SDK·카메라를 사용한 in-bin 파지 검사를 아직 통과하지 않았습니다. 기존 배포본의 실제 폭 검사와 후보 재선택은 그대로 유지합니다.

`random-motion.json`의 `scene.known_hand_surface`를 설정하면 바구니 촬영 전후 실제 양팔 SDK 상태를 읽고, 두 읽기 사이의 실제 노출만 사용합니다. 실제 q/q_d, TCP 설정, Hand 폭, 증가하는 SDK 시계가 맞아야 합니다. 이미 받은 영상을 새 시각으로 바꾸지 않습니다. 기존 셀 제어 잠금 안에서 실행하며, 읽기/노출 간 최대 간격은 200ms입니다.

필요한 값은 `surface_tolerance_m`(최대 0.003m), `pose_uncertainty_m`(실제 카메라 외부좌표·로봇 기준좌표·Hand 설치 오차를 포함하는 양수 상한), `max_sdk_capture_gap_s`(최대 0.2s)입니다. 수치를 임의로 0으로 설정하면 안 됩니다. 카메라의 기존 `depth_uncertainty_m`와 pose 상한의 합이 표면 허용 범위 이하여야 합니다. 원래 scene 측정 기록과 설치 기록을 그대로 검증합니다.

원본 official Hand 표면과 실제 SDK FK/폭으로 설명되는 **관측된 표면**을 같은 body에만 연결합니다. 예를 들어 오른손 손바닥에서 나온 점유를 왼손이나 운반 링이 무시하지 않습니다. 임의의 물체 점이나 가려진 영역을 FREE로 바꾸지 않습니다. 섞인 voxel에 설명되지 않는 점이 하나라도 있으면 그 voxel은 계속 일반 장애물입니다. 입력 SDK 상태·사진·보정 파일·표면 연결 결과는 관측 기록에 고정되고 계획 시 다시 검증됩니다.

선택적인 `robot_material_prior.geometry_uncertainty_m`는 **정확히 닫힌 official visual CAD 부분** 내부만 사용합니다. voxel 전체를 감싸는 구와 실제 pose/depth/CAD 오차를 포함해 안쪽으로 깎은 영역이어야 합니다. 음의 방향인 내부 구멍을 보존합니다. 해당 영역은 같은 body의 `known_robot_material`로 분류하며 FREE로 만들지 않습니다. 다른 팔·부품에는 여전히 장애물입니다.

현재 official 손가락 visual mesh는 닫혀 있지만, 큰 손바닥 외피는 모든 material을 합쳐도 열려 있습니다. 따라서 손바닥 collision hull 내부 전체를 물리적 재질로 간주하지 않습니다. 닫힌 두 작은 부속만 이 prior에 들어갑니다. 실제 장비에서 확인되지 않은 외피 닫힘이나 막힌 구멍을 만들어내지 않습니다.

검사 결과: 기록된 Isaac RGB-D/엔코더 4장에서는 8mm voxel 표면 연결이 0개였고, 2mm 국소 voxel도 0–46개에 그쳤습니다. 실행 시간도 3.2–5.1초여서 현재 2초 영상 유효기간을 넘습니다. 따라서 이 기능이 자기 Hand 간섭 문제를 해결했다고 표시하지 않습니다. 결과와 정확한 범위는 `reports/known-hand-selfsurface/STATUS.json`에 있습니다. 실제 닫힌 손바닥 외형 자료 또는 더 정교한 관측 점유 표현이 다음 작업입니다.

## 현장 프로토타입용 collision-envelope 가정

닫힌 physical visual CAD만 쓰는 방식과 별도로, `collision_envelope_prior.geometry_uncertainty_m`를 양수로 명시할 수 있습니다. `robot_material_prior`와 동시에 켜지 않습니다. 실제 설치가 기존 official FR3/native Hand 모델·factory mount와 맞는다는 설치 기록이 필요합니다.

이 opt-in은 **원본 official collision 모델 내부가 해당 로봇 body에 속한다는 공학적 가정**입니다. physical material이나 실제 센서 검증 결과라고 부르지 않습니다. 실제 SDK frame/폭으로 각각의 손바닥/손가락을 변환한 후, voxel 전체와 pose/depth/geometry 오차가 안쪽에 들어가는 부분만 `known_robot_envelope`로 분류합니다. FREE로 만들지 않으며, 그 body 자신만 해당 점유를 무시합니다. 외부 shadow/UNKNOWN, 모델 밖, 손가락 사이 틈, 다른 body/팔/운반 부품 검사는 유지합니다. 이미 관측된 selected stock과 전체 detected stock CAD도 유지합니다. Collision mesh가 실제 표면의 오목한 부분을 덮을 수 있는 가정이므로 실제 공장 장착 상태와 형상 차이를 확인해야 합니다.

설정 예시의 값은 현장 측정으로 채워야 합니다. 필드명은 `scene.known_hand_surface.surface_tolerance_m`, `.pose_uncertainty_m`, `.max_sdk_capture_gap_s`, `.collision_envelope_prior.geometry_uncertainty_m`입니다. 임의의 pass boolean이나 0 오차로 활성화하지 않습니다.

이 가정을 적용한 264 기록 재생에서는 8mm voxel 622–696개가 같은 body의 envelope로 분류됐고 생성 시간은 0.74–1.38초였습니다. 하지만 nominal Hand capsule의 잔여 점유 접촉은 여전히 남습니다. 전체 in-bin 파지 전 검사가 해결됐다고 주장하지 않습니다. 이 단계는 실제 촬영/SDK 입력으로 시험 가능한 명시적 프로토타입이며, 기록 재생은 SDK 하드웨어 검증이 아닙니다.
