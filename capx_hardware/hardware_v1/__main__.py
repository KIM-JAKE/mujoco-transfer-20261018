"""Offline inspection by default. Explicit status/camera probes never arm motion."""
import argparse
import json
from .backend import HardwareBackend


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("preflight", "status", "camera"))
    parser.add_argument("--config", required=True)
    parser.add_argument("--connect-read-only", action="store_true")
    parser.add_argument("--side", choices=("left", "right"))
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    if args.command != "preflight" and not args.connect_read_only:
        parser.error("Device reads require explicit --connect-read-only; preflight connects nothing")
    backend = HardwareBackend(args.config)
    try:
        if args.command == "preflight":
            report = backend.preflight()
            code = 0 if report["local_config_complete"] else 2
        elif args.command == "status":
            sides = (args.side,) if args.side else ("left", "right")
            report = {"domain": "hardware", "robots": {s: backend.read_state(s) for s in sides},
                      "hands": {s: backend.hand_status(s) for s in sides},
                      "motion_issued": False, "hardware_ready": False}
            code = 0
        else:
            if not args.output:
                parser.error("camera requires --output NEW_CAPTURE.npz")
            packet, metadata = backend.capture()
            backend._drivers().module("field_demo", "camera").save_capture(args.output, packet, metadata)
            report = metadata
            code = 0
        print(json.dumps(report, indent=2, allow_nan=False))
        return code
    except (ValueError, RuntimeError, OSError, KeyError, TypeError) as exc:
        print(json.dumps({"schema": "capx-hardware-operation-failed/v1", "operation": args.command,
                          "error_type": type(exc).__name__, "error": str(exc),
                          "device_read_requested": args.command != "preflight",
                          "motion_issued": False, "hardware_ready": False}, indent=2))
        return 1
    finally:
        backend.close()


if __name__ == "__main__":
    raise SystemExit(main())
