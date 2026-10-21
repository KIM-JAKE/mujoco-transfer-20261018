#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
STATE_ONLY=OFF
while (( $# )); do
  case "$1" in
    --help|-h) echo 'Usage: scripts/build.sh [--prefix SDK_PREFIX] [--state-only]'; echo 'Build from source using the installed compatible SDK; no sudo, downloads or robot connection.'; echo 'Full motion build targets libfranka >=0.18 / robot protocol 10. --state-only builds only the read-only probe.'; exit 0 ;;
    --prefix) [[ $# -ge 2 ]] || { echo '--prefix requires a directory' >&2; exit 2; }; export CMAKE_PREFIX_PATH="$2${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"; shift 2 ;;
    --state-only) STATE_ONLY=ON; shift ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
case "$(uname -s):$(uname -m)" in Linux:x86_64|Linux:aarch64|Linux:arm64) ;; *) echo 'Supported host: Linux x86_64 or aarch64 with a compatible Franka SDK.' >&2; exit 2 ;; esac
export PATH="$ROOT/.venv/bin:$PATH"
if ! command -v cmake >/dev/null 2>&1; then echo 'CMake is missing. Install a compatible local CMake before building; doctor lists missing prerequisites.' >&2; exit 2; fi
mkdir -p "$ROOT/reports"
BUILD_DIR="$ROOT/build"
if [[ "$STATE_ONLY" == ON ]]; then BUILD_DIR="$ROOT/build-state"; fi
cmake -S "$ROOT/src/isaac_port/fieldkit" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -DM80_STATE_PROBE_ONLY="$STATE_ONLY" 2>&1 | tee "$ROOT/reports/build-configure.log"
cmake --build "$BUILD_DIR" --parallel "${FIELD_DEMO_BUILD_JOBS:-2}" 2>&1 | tee "$ROOT/reports/build.log"
if [[ "$STATE_ONLY" == OFF ]]; then cmake -E chdir "$BUILD_DIR" ctest --output-on-failure 2>&1 | tee "$ROOT/reports/ctest.log"; fi
printf '%s\n' "Source build finished: $BUILD_DIR. Hardware connection and motion remain unverified."
