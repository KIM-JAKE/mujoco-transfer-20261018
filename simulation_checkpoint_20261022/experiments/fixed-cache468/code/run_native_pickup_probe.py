"""Isaac Kit entry point: slow physical pickup test with both native Hands."""
import os,traceback,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
os.environ['M80_PICKUP_PROBE']='1'
import native_roas_physx as scene
try:
    scene.main();scene.omni.kit.app.get_app().post_quit(0)
except Exception:
    (scene.OUT/'pickup-error.txt').write_text(traceback.format_exc())
    traceback.print_exc();scene.omni.kit.app.get_app().post_quit(1)
