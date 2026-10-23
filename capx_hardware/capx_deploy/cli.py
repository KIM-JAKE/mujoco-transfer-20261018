"""Create local site settings and verify installed sources, without device I/O."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = HERE.parent


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+"\n")


def verify_vendor():
    provenance = json.loads((HERE/"data/vendor-provenance.json").read_text())
    failed = [name for name, row in provenance.items()
              if not (PACKAGE_ROOT/name).is_file() or digest(PACKAGE_ROOT/name) != row["sha256"]]
    if failed:
        raise ValueError("Bundled driver source differs: "+", ".join(failed))
    return {"verified_vendor_files": len(provenance), "device_connected": False,
            "motion_issued": False, "hardware_ready": False}


def initialize(output):
    """Generate fresh configuration for this installation, never overwrite a site."""
    verified = verify_vendor()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    templates = HERE/"templates"
    for source in templates.glob("*.json"):
        (output/source.name).write_bytes(source.read_bytes())
    provenance = json.loads((HERE/"data/vendor-provenance.json").read_text())
    manifest = {"schema": "capx-bundled-drivers/v1",
                "packages": {family: os.path.relpath(PACKAGE_ROOT/"capx_vendor"/family, output)
                             for family in ("fieldkit", "field_demo")},
                "files": {os.path.relpath(PACKAGE_ROOT/name, output): row["sha256"]
                          for name, row in provenance.items()}}
    write_json(output/"driver-manifest.json", manifest)
    config = json.loads((output/"hardware.json").read_text())
    config["driver_manifest"] = {"path": "driver-manifest.json", "sha256": digest(output/"driver-manifest.json")}
    config["site_config"] = "site.json"
    config["site_config_sha256"] = digest(output/"site.json")
    config["camera"]["calibration"] = "camera.json"
    config["camera"]["calibration_sha256"] = digest(output/"camera.json")
    task_files = {p.name: digest(p) for p in (PACKAGE_ROOT/"capx_ring_tasks").glob("*.py")}
    config["task_source_sha256"] = hashlib.sha256(
        json.dumps(task_files, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    config["motion_enabled"] = False
    write_json(output/"hardware.json", config)
    return dict(verified, site_directory=str(output), config=str(output/"hardware.json"),
                next_step="Fill measured site/camera/tool/fixture values, then run capx-fr3 preflight")


def pin_files(config_path, build_dir=None):
    """Record edited settings/binary identities; this never approves or enables motion."""
    config_path = Path(config_path).resolve()
    config = json.loads(config_path.read_text())
    base = config_path.parent
    def resolve(value):
        path = Path(value)
        return path if path.is_absolute() else base/path
    site_path = resolve(config["site_config"])
    site = json.loads(site_path.read_text())
    if build_dir is not None:
        directory = Path(build_dir).resolve()
        binaries = {"state": "m80_read_state", "hand": "m80_franka_hand", "motion": "m80_site_motion"}
        records = {}
        for name, filename in binaries.items():
            binary = directory/filename
            if not binary.is_file() or not os.access(binary, os.X_OK):
                raise ValueError("Build the original compatible libfranka tools first: "+str(binary))
            records[name] = {"path": str(binary), "sha256": digest(binary)}
        config["binaries"] = records
        site["state_binary"] = str(directory/binaries["state"])
        site["motion_binary"] = str(directory/binaries["motion"])
        for row in site["grippers"].values():
            row["binary"] = str(directory/binaries["hand"])
        write_json(site_path, site)
    config["site_config_sha256"] = digest(site_path)
    config["camera"]["calibration_sha256"] = digest(resolve(config["camera"]["calibration"]))
    # Do not enable motion just because a file hash or build changed.
    config["motion_enabled"] = False
    write_json(config_path, config)
    return {"config": str(config_path), "pinned_local_files": True,
            "motion_enabled": False, "hardware_ready": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("verify-vendor")
    sub.add_parser("build-source")
    init = sub.add_parser("init")
    init.add_argument("directory", help="New site directory; existing directories are never overwritten")
    pre = sub.add_parser("preflight")
    pre.add_argument("--config", required=True)
    pin = sub.add_parser("pin-files")
    pin.add_argument("--config", required=True)
    pin.add_argument("--build-dir")
    args = parser.parse_args(argv)
    try:
        code = 0
        if args.command == "init":
            result = initialize(args.directory)
        elif args.command == "verify-vendor":
            result = verify_vendor()
        elif args.command == "build-source":
            print(PACKAGE_ROOT/"capx_vendor/fieldkit")
            return 0
        elif args.command == "pin-files":
            result = pin_files(args.config, args.build_dir)
        else:
            from hardware_v1.backend import HardwareBackend
            backend = HardwareBackend(args.config)
            try:
                result = backend.preflight()
            finally:
                backend.close()
            code = 0 if result["local_config_complete"] else 2
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return code
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(json.dumps({"error": str(error), "device_connected": False, "motion_issued": False}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
