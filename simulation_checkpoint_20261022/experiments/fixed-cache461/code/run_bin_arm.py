from pathlib import Path
import sys,os,traceback,shutil,json,importlib.abc,importlib.util
sys.path.insert(0,str(Path(__file__).resolve().parent));os.environ['M80_BIN_TASK']='1'
import omni.kit.app
try:
    root=Path(__file__).resolve().parents[2]/'runs'/os.environ['M80_ROAS_RUN'];manifest_path=root/'code-manifest.json'
    if manifest_path.exists():
        manifest=json.load(open(manifest_path))
        class FrozenLoader(importlib.abc.Loader):
            def __init__(self,name):self.name=name
            def create_module(self,spec):return None
            def exec_module(self,module):
                origin=manifest[self.name]['original_file'].replace('/home/gpuadmin/workspace/jake','/workspace/project',1);module.__file__=origin;exec(compile((root/'code_snapshot'/(self.name+'.py')).read_text(),origin,'exec'),module.__dict__)
        class FrozenFinder(importlib.abc.MetaPathFinder):
            def find_spec(self,fullname,path=None,target=None):
                if fullname in manifest:return importlib.util.spec_from_loader(fullname,FrozenLoader(fullname),origin=manifest[fullname]['original_file'])
        sys.meta_path.insert(0,FrozenFinder())
    import native_roas_physx
    for name in ['arm-plan.json','initial-robot-pose.json']:
        if not(native_roas_physx.OUT/name).exists():shutil.copyfile(Path(__file__).resolve().parent/'bin-picking'/name,native_roas_physx.OUT/name)
    os.environ['M80_BIN_INPUT']=str(native_roas_physx.OUT)
    native_roas_physx.main();omni.kit.app.get_app().post_quit(0)
except Exception:traceback.print_exc();omni.kit.app.get_app().post_quit(1)
