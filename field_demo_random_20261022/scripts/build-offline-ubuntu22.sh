#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:-}" in
  --help|-h)
    echo 'Usage: scripts/build-offline-ubuntu22.sh'
    echo 'Ubuntu 22.04 Linux x86_64 only: extract bundled .deb dependencies into .local-sdk and build local source.'
    echo 'Requires an existing C++ compiler, Python 3.10+, make/ninja, and dpkg-deb. No sudo, system installation, downloads, or robot connection.'
    echo 'Bundled SDK/shared libraries are specific to this host family; these binaries are not universal.'
    exit 0 ;;
  '') ;;
  *) echo 'Unexpected argument; see --help.' >&2; exit 2 ;;
esac
if (( $# )); then echo 'Unexpected arguments; see --help.' >&2; exit 2; fi
PYTHON=${FIELD_DEMO_PYTHON:-python3}
if ! "$PYTHON" -c 'import platform,sys; info=platform.freedesktop_os_release(); sys.exit(0 if platform.system()=="Linux" and platform.machine()=="x86_64" and info.get("ID")=="ubuntu" and info.get("VERSION_ID")=="22.04" and sys.version_info>=(3,10) else 2)'; then
  echo 'Offline bundled dependencies require Ubuntu 22.04 x86_64 and Python 3.10+. Use scripts/build.sh with a compatible locally installed SDK on other hosts.' >&2
  exit 2
fi
for tool in dpkg-deb c++; do
  if ! command -v "$tool" >/dev/null 2>&1; then echo "Required existing tool is missing: $tool" >&2; exit 2; fi
done
if ! command -v make >/dev/null 2>&1 && ! command -v ninja >/dev/null 2>&1; then echo 'An existing make or ninja build tool is required.' >&2; exit 2; fi
shopt -s nullglob
DEBS=("$ROOT"/vendor/debs/*.deb)
if (( ${#DEBS[@]} == 0 )); then echo 'Bundled vendor/debs/*.deb files are missing. Nothing was downloaded.' >&2; exit 2; fi
SDK="$ROOT/.local-sdk"
mkdir -p "$SDK" "$ROOT/reports"
rm -f -- "$SDK/TESTED_HOST.json"
for package in "${DEBS[@]}"; do
  arch=$(dpkg-deb -f "$package" Architecture)
  case "$arch" in amd64|all) ;; *) echo "Unsupported bundled package architecture: $arch" >&2; exit 2 ;; esac
  printf 'Extracting %s\n' "$(basename -- "$package")"
  dpkg-deb -x "$package" "$SDK"
done
if [[ -L "$ROOT/build" ]]; then echo 'Refusing to change a symlinked build directory. Choose a real package-local build directory.' >&2; exit 2; fi
if [[ -f "$ROOT/build/CMakeCache.txt" ]]; then
  if ! "$PYTHON" - "$ROOT" "$SDK" <<'PY_CACHE'
from pathlib import Path
import sys
root, sdk = (Path(value).resolve() for value in sys.argv[1:])
valid = True
for line in (root / 'build/CMakeCache.txt').read_text().splitlines():
    if line.startswith(('Franka_DIR:', 'Eigen3_DIR:', 'CMAKE_HOME_DIRECTORY:')) and '=' in line:
        key, value = line.split('=', 1)
        target = Path(value).resolve()
        expected = root / 'src/isaac_port/fieldkit' if key.startswith('CMAKE_HOME_DIRECTORY:') else sdk
        matches = target == expected if key.startswith('CMAKE_HOME_DIRECTORY:') else target.is_relative_to(expected)
        valid = valid and matches
sys.exit(0 if valid else 2)
PY_CACHE
  then
    echo 'Resetting only generated CMakeCache.txt/CMakeFiles: previous build references another SDK or package location.' >&2
    rm -f -- "$ROOT/build/CMakeCache.txt"
    rm -rf -- "$ROOT/build/CMakeFiles"
  fi
fi
export PATH="$SDK/usr/bin:$ROOT/.venv/bin:$PATH"
export LD_LIBRARY_PATH="$SDK/usr/lib/x86_64-linux-gnu:$SDK/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_PREFIX_PATH="$SDK/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
"$ROOT/scripts/build.sh" 2>&1 | tee "$ROOT/reports/offline-build.log"
"$PYTHON" - "$SDK" "${DEBS[@]}" <<'PY'
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys
sdk = Path(sys.argv[1])
packages = [{'name': Path(name).name, 'sha256': hashlib.sha256(Path(name).read_bytes()).hexdigest()} for name in sys.argv[2:]]
report = {'schema_version': 1, 'os_id': 'ubuntu', 'os_version': '22.04', 'architecture': 'x86_64',
          'kernel': platform.release(), 'built_at_utc': datetime.now(timezone.utc).isoformat(),
          'hardware_ready': False, 'robot_connected': False, 'source_build_and_offline_tests_passed': True,
          'packages': packages, 'note': 'Runtime libraries tested only for Ubuntu 22.04 x86_64; no robot execution is certified.'}
(sdk / 'TESTED_HOST.json').write_text(json.dumps(report, indent=2) + '\n')
PY
printf '%s\n' 'Offline source build passed. Local SDK marker written; robot compatibility and execution still require onsite verification.'
