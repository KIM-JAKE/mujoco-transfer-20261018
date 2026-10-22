> **현재 우선 작업: 기존 선반의 핀 → 작은 링 → 큰 링 고정 배치, native Franka Hand, 저장 경로 + RGB-D 보정.** [최신 시뮬레이션 코드·성공/실패·시간·영상](FIXED_FRONT_20261022.md)을 먼저 확인하세요. [현장 SDK 온보딩](FIELD_RGBD_20261022.md)은 별도 실험용 저속 패키지이며, 최신 시뮬 정책의 전체 실기 실행을 검증했다는 뜻은 아닙니다.

# MuJoCo handoff for AIDAS GPU server

## M80 Isaac Sim / 현장 연결 준비

**Franka Hand 현장 실행은 [전용 빠른 시작](isaac_port/fieldkit/FRANKA_HAND_QUICKSTART.md)을 먼저 읽는다.** native Hand 통신·저속 팔 실행·실물 경로 기록·PLC 피드백 기반 순차 CaP 어댑터를 제공한다. 실제 자동 조립은 현장 경로/입출력 commissioning 후에 실행한다. 기존 Robotiq 9초 경로는 native Hand 실기에 재생하지 않는다.

현장 엔지니어용 **[기존 FR3 온보딩](isaac_port/fieldkit/ONBOARDING_FR3.md)**과 [과거 원본 배치9.466667초 기록](isaac_port/reports/contact-trim-03/summary.json)은 보존한다. 이 과거 수치를 현재 native Hand 고정 공급 배치의 RGB-D 촬영·인식·검사 포함 시간으로 사용하면 안 된다. 현재 검증 범위와 시간은 위 최신 안내를 기준으로 한다. 실제 양팔·그리퍼·PLC 통합 동작은 아직 미검증이다.

새 PhysX 포팅과 현장용 점검 도구는 [isaac_port](isaac_port/README.md), 실기 연결 절차와 미검증 조건은 [fieldkit 안내](isaac_port/fieldkit/README.md)에 있다. Python만으로 `python3 -m isaac_port.fieldkit preflight`와 `dry-run`을 실행할 수 있다. 이 점검에는 아래 전체 아카이브·SAM3 복원이 필요 없다. 현재 실기 자동 조립은 미검증이며, 10초 미만 최적화 목표와 실제 로봇의 가동 가능 여부를 구분한다.

Private migration snapshot for `/home/gpuadmin/workspace/jake/mujoco`. It contains the M80 CAD model, FR3 Duo and G2 exchange experiments, local robot learning MuJoCo work from the other session, code, vendor assets, saved trajectories, validation reports, playbooks, and environment snapshots. SSH keys, tokens, and account configuration are outside the snapshot.

## Included source trees

| Restored path | Source on Kakao server | Purpose |
| --- | --- | --- |
| `keti_m80_mujoco_20261013/` | `/home/jovyan/data-1/jake/keti_m80_mujoco_20261013/` | M80 CAD, FR3 Duo, side exchange, CuRobo and MuJoCo runs |
| `g2_replenishment_poc_20261016/` | `/home/jovyan/data-1/jake/g2_replenishment_poc_20261016/` | Original G2 physical trajectory and scene referenced by the exchange |
| `fr3_cube_mujoco_handoff_20261012/` and `_v2/` | `/home/jovyan/data-1/jake/` | Earlier FR3 cube MuJoCo handoffs |
| `robotics_agent/` | `/home/jovyan/data/wjkim/robotics_agent/` | Other-session Duo refilling, trajectory, top-view and ring experiments |

The archive also includes the three existing FR3 cube ZIP/checksum files. The archive is split into Git LFS parts; `archives/SHA256SUMS` validates every part before extraction. `relocation_report.json` records code/XML path edits made after restore. Original byte content remains in the archive. Historical reports may still name the old server path as provenance.

The other-session SAM3.1 checkpoint was stored outside those source trees in the Kakao Hugging Face cache. It is included separately as `archives/sam31-checkpoint.zst.part-*` and restored to `robotics_agent/mujoco_duo_fr3/runtime/sam31_checkpoint/sam3.1_multiplex.pt` with its own SHA-256 check.

## On AIDAS

```bash
mkdir -p /home/gpuadmin/workspace/jake
cd /home/gpuadmin/workspace/jake
# Authenticate to GitHub on AIDAS first (for example, gh auth login).
git lfs install
git clone https://github.com/KIM-JAKE/mujoco-transfer-20261018.git mujoco
cd mujoco
git lfs pull
./restore.sh
./setup_main_env.sh
# After installing AIDAS-compatible PyTorch and torchvision in Python 3.11:
./setup_sam31_env.sh
```

The repository is private. GitHub authentication must be configured on AIDAS before cloning; do not copy the Kakao server's private SSH keys or tokens. If AIDAS already has a GitHub SSH key with repository access, `git@github.com:KIM-JAKE/mujoco-transfer-20261018.git` is also usable.

`./restore.sh` checks SHA-256, extracts to the clone directory, restores the SAM3.1 checkpoint, and rewrites active Python, shell, and XML paths for the new root. It does not modify the archive. The copied `.venv`, `.venv-curobo`, and SAM3 environment directories are source snapshots; their Python symlinks and compiled packages may not run on AIDAS. `setup_main_env.sh` preserves the copied `.venv` as `.venv-source-snapshot` and creates the main Python 3.13 environment from `docs/requirements-main-snapshot.txt`. The source SAM3 worker used a separate ego-scale-compiler Python environment; `setup_sam31_env.sh` creates its new location after an AIDAS-compatible PyTorch/torchvision is installed in Python 3.11. Dependency records are in `docs/`; see `docs/ENVIRONMENTS.md` for scope and limits.

## First checks

```bash
cd /home/gpuadmin/workspace/jake/mujoco/keti_m80_mujoco_20261013
source env.sh
.venv/bin/python - <<'PY'
import mujoco
m = mujoco.MjModel.from_xml_path('m80.xml')
print('M80 model:', m.nbody, 'bodies,', m.ngeom, 'geoms')
PY
.venv/bin/python - <<'PY'
import mujoco
m = mujoco.MjModel.from_xml_path('duo_fr3_2f85_20261013/g2_fr3_side_exchange_20261016/scene_outbound_candidate.xml')
print('Exchange model:', m.nbody, 'bodies,', m.ngeom, 'geoms')
PY
```

Read `keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/g2_fr3_side_exchange_20261016/README.md` for the verified exchange stages and their limitations. Read `robotics_agent/mujoco_duo_fr3/README_LOCAL.txt`, `README_REFILL.txt`, and `README_RING1_OPT10.txt` for the other-session work. Saved reports and trajectories are source evidence; no new physical result is implied by migration.

## Migration status

This repository is a transfer artifact. AIDAS-side smoke checks must be run after clone; package and hash checks on the Kakao server do not establish that the new server's Python, GPU/CuRobo, or render stack works.
