import os,runpy,traceback
from pathlib import Path
os.environ['M80_COMPONENT_THREADS']='2'
os.environ['M80_COMPONENT_OUT']='/workspace/project/mujoco/.local/thread-component-494/threads-2'
os.environ['M80_COMPONENT_BASE']='/workspace/project/mujoco/.local/thread-component-494'
os.environ['M80_COMPONENT_H']='/workspace/project/mujoco/.local/thread-component-494/tree/isaac_port/native_hand_layout/roas-table'
os.environ['M80_ROAS_RUN']='independent-thread-component-494'
try:
 runpy.run_path('/workspace/project/mujoco/.local/thread-component-494/component.py',run_name='__main__')
except BaseException:
 (Path(os.environ['M80_COMPONENT_OUT'])/'error.txt').write_text(traceback.format_exc())
finally:
 import omni.kit.app
 omni.kit.app.get_app().post_quit(0)
