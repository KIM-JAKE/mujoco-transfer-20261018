from pathlib import Path
import os
PACKAGE = Path(__file__).resolve().parent
PROJECT = Path(os.environ['M80_PROJECT_ROOT']) if 'M80_PROJECT_ROOT' in os.environ else PACKAGE.parents[3]
ROOT = Path(os.environ.get('M80_GPU_PLANNING_HOME', str(PROJECT / '.local/curobo-gpu2')))
ROOT.mkdir(parents=True, exist_ok=True)
SOURCE = Path(os.environ.get('M80_GPU_ROBOT_JSON', str(PACKAGE / 'reference-right-native-kinematics.json')))
URDF = Path(os.environ.get('M80_GPU_URDF', str(ROOT / 'production-native-fr3-right.urdf')))
VENDOR = Path(os.environ.get('M80_CUROBO_SOURCE', str(PROJECT / 'keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/vendor/curobo')))
