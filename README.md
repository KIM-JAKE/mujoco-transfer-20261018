# MuJoCo handoff for AIDAS GPU server

Private migration snapshot for `/home/gpuadmin/workspace/jake/mujoco`. It contains the M80 CAD model, FR3 Duo and G2 exchange experiments, local robot learning MuJoCo work from the other session, code, vendor assets, saved trajectories, validation reports, playbooks, and environment snapshots. SSH keys, tokens, and account configuration are outside the snapshot.

## Included source trees

| Restored path | Source on Kakao server | Purpose |
| --- | --- | --- |
| `keti_m80_mujoco_20261013/` | `/home/jovyan/data-1/jake/keti_m80_mujoco_20261013/` | M80 CAD, FR3 Duo, side exchange, CuRobo and MuJoCo runs |
| `g2_replenishment_poc_20261016/` | `/home/jovyan/data-1/jake/g2_replenishment_poc_20261016/` | Original G2 physical trajectory and scene referenced by the exchange |
| `fr3_cube_mujoco_handoff_20261012/` and `_v2/` | `/home/jovyan/data-1/jake/` | Earlier FR3 cube MuJoCo handoffs |
| `robotics_agent/` | `/home/jovyan/data/wjkim/robotics_agent/` | Other-session Duo refilling, trajectory, top-view and ring experiments |

The archive also includes the three existing FR3 cube ZIP/checksum files. The archive is split into Git LFS parts; `archives/SHA256SUMS` validates every part before extraction. `relocation_report.json` records code/XML path edits made after restore. Original byte content remains in the archive. Historical reports may still name the old server path as provenance.

## On AIDAS

```bash
mkdir -p /home/gpuadmin/workspace/jake
cd /home/gpuadmin/workspace/jake
git lfs install
git clone git@github.com:KIM-JAKE/mujoco-transfer-20261018.git mujoco
cd mujoco
git lfs pull
./restore.sh
./setup_main_env.sh
```

`./restore.sh` checks SHA-256, extracts to the clone directory, and rewrites active Python, shell, and XML paths for the new root. It does not modify the archive. The copied `.venv`, `.venv-curobo`, and SAM3 environment directories are source snapshots; their Python symlinks and compiled packages may not run on AIDAS. `setup_main_env.sh` preserves the copied `.venv` as `.venv-source-snapshot` and creates the main Python 3.13 environment from `docs/requirements-main-snapshot.txt`. The CuRobo and SAM3 package snapshots are in `docs/requirements-curobo-snapshot.txt` and `docs/requirements-sam3-snapshot.txt`; configure those against AIDAS GPU drivers separately.

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
