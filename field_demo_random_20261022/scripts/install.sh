#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
CAMERA=0
REALSENSE=0
OFFLINE=0
WHEELHOUSE=""
while (( $# )); do
  case "$1" in
    --help|-h) echo 'Usage: scripts/install.sh [--with-camera] [--with-realsense] [--wheelhouse DIR] [--offline]'; echo 'Default: create .venv only; no sudo, package downloads or robot connection.'; echo '--with-camera explicitly installs NumPy/SciPy; --with-realsense also installs pyrealsense2.'; echo '--offline uses only local wheels (--no-index); use a wheelhouse matching the onsite Python/OS/architecture.'; exit 0 ;;
    --wheelhouse) [[ $# -ge 2 ]] || { echo '--wheelhouse requires a directory' >&2; exit 2; }; WHEELHOUSE=$2; shift 2 ;;
    --offline) OFFLINE=1; shift ;;
    --with-camera) CAMERA=1; shift ;;
    --with-realsense) CAMERA=1; REALSENSE=1; shift ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
case "$(uname -s):$(uname -m)" in Linux:x86_64|Linux:aarch64|Linux:arm64) ;; *) echo 'Supported host: Linux x86_64 or aarch64 with a compatible Franka SDK.' >&2; exit 2 ;; esac
PYTHON=${FIELD_DEMO_PYTHON:-python3}
"$PYTHON" -c 'import sys; assert sys.version_info >= (3, 10), "Python 3.10 or newer is required"'
mkdir -p "$ROOT/reports"
if ! "$PYTHON" -m venv "$ROOT/.venv" >"$ROOT/reports/venv.log" 2>&1; then
  if [[ -f "$ROOT/vendor/virtualenv.pyz" ]]; then
    echo 'System venv is unavailable; using bundled virtualenv without downloads. See reports/venv.log.' >&2
    if ! "$PYTHON" "$ROOT/vendor/virtualenv.pyz" --no-download --no-periodic-update --app-data "$ROOT/.virtualenv-cache" "$ROOT/.venv" >>"$ROOT/reports/venv.log" 2>&1; then
      echo 'Offline virtual environment creation failed; see reports/venv.log. Install the matching Python venv package and rerun.' >&2
      exit 2
    fi
  else
    echo 'Python venv/ensurepip and bundled vendor/virtualenv.pyz are unavailable. Install the matching Python venv package (Ubuntu: python3-venv), then rerun. See reports/venv.log; no download was attempted.' >&2
    exit 2
  fi
fi
PIP_ARGS=()
if [[ -n "$WHEELHOUSE" ]]; then
  [[ -d "$WHEELHOUSE" ]] || { echo 'Wheelhouse directory does not exist.' >&2; exit 2; }
  PIP_ARGS+=(--find-links "$WHEELHOUSE")
fi
if (( OFFLINE )); then
  if (( CAMERA )) && [[ -z "$WHEELHOUSE" ]]; then echo '--offline camera installation requires --wheelhouse DIR.' >&2; exit 2; fi
  PIP_ARGS+=(--no-index)
fi
if (( CAMERA )); then "$ROOT/.venv/bin/python" -m pip install "${PIP_ARGS[@]}" -r "$ROOT/requirements.txt"; fi
if (( REALSENSE )); then "$ROOT/.venv/bin/python" -m pip install "${PIP_ARGS[@]}" -r "$ROOT/requirements-camera.txt"; fi
mkdir -p "$ROOT/reports" "$ROOT/logs" "$ROOT/site/poses" "$ROOT/site/trajectories"
if [[ ! -e "$ROOT/site/site.json" ]]; then cp -- "$ROOT/config/templates/site.json" "$ROOT/site/site.json"; fi
printf '%s\n' 'Local environment prepared. Site values remain unmeasured; no robot was connected.'
printf '%s\n' 'Next: ./field-demo doctor; scripts/build.sh --help; scripts/offline-check.sh'
