"""Verify the installed wheel away from source; no device connections."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="capx-installed-verification-") as directory:
        directory = Path(directory)
        for name in ("tests", "scripts"):
            shutil.copytree(project/name, directory/name, ignore=shutil.ignore_patterns("__pycache__"))
        code = r'''
import importlib,json,os,pathlib,sys,unittest
project=pathlib.Path(sys.argv[1]).resolve()
modules=[importlib.import_module(n)for n in ('hardware_v1.backend','capx_ring_tasks.bridge','capx_deploy.cli','fr3_capx')]
paths={m.__name__:str(pathlib.Path(m.__file__).resolve())for m in modules}
assert all(not pathlib.Path(p).is_relative_to(project)for p in paths.values()),paths
installed=pathlib.Path(modules[0].__file__).resolve().parents[1]
os.environ['CAPX_HARDWARE_BACKEND_ROOT']=str(installed)
os.environ['CAPX_RING_TASK_SRC']=str(installed)
suite=unittest.defaultTestLoader.discover('tests')
result=unittest.TextTestRunner(verbosity=2).run(suite)
report=dict(installed_modules=paths,tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),device_connected=False,motion_issued=False,hardware_ready=False)
pathlib.Path(sys.argv[2]).write_text(json.dumps(report,indent=2)+'\n')
sys.exit(0 if result.wasSuccessful() and not result.skipped else 1)
'''
        environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        environment.pop("PYTHONPATH", None)
        with output.with_suffix(".log").open("w") as log:
            test = subprocess.run([sys.executable, "-c", code, str(project), str(output)],
                                  cwd=directory, env=environment, stdout=log, stderr=log)
        if test.returncode:
            raise RuntimeError("Installed checks failed; see "+str(output.with_suffix(".log")))
        config_dir = directory/"site"
        def cli(*arguments):
            return subprocess.run([sys.executable, "-m", "capx_deploy.cli", *arguments],
                                  cwd=directory, env=environment, capture_output=True, text=True)
        initialized = cli("init", str(config_dir))
        if initialized.returncode:
            raise RuntimeError(initialized.stderr)
        preflight = cli("preflight", "--config", str(config_dir/"hardware.json"))
        inspected = json.loads(preflight.stdout)
        if preflight.returncode != 2 or inspected["local_config_complete"] or inspected["device_connected"] or inspected["motion_issued"]:
            raise RuntimeError("Empty site did not remain incomplete and non-actuating")
        report = json.loads(output.read_text())
        report.update(init_cli_exit=0, empty_site_preflight_exit=2,
                      empty_site_blockers=len(inspected["blockers"]), source_directory_not_on_import_path=True,
                      hardware_qualified=False)
        output.write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
