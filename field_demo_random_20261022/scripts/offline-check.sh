#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:-}" in --help|-h) echo 'Usage: scripts/offline-check.sh'; echo 'Run offline unit/integration tests, including camera tests requiring NumPy/SciPy. No hardware connection.'; exit 0 ;; '') ;; *) echo 'Unexpected argument; see --help.' >&2; exit 2 ;; esac
if (( $# )); then echo 'Unexpected arguments; see --help.' >&2; exit 2; fi
PYTHON=${FIELD_DEMO_PYTHON:-$ROOT/.venv/bin/python}
if [[ ! -x "$PYTHON" ]]; then PYTHON=python3; fi
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=${FIELD_DEMO_BLAS_THREADS:-1}
export OMP_NUM_THREADS=${FIELD_DEMO_BLAS_THREADS:-1}
mkdir -p "$ROOT/reports"
if ! "$PYTHON" -c 'import numpy, scipy' >/dev/null 2>&1; then
  echo 'Offline camera tests require NumPy/SciPy. Run scripts/install.sh --with-camera (or use --offline --wheelhouse DIR) first.' >&2
  exit 2
fi
"$PYTHON" -m unittest discover -s "$ROOT/tests" -p 'test_*.py' -v 2>&1 | tee "$ROOT/reports/offline-tests.log"
"$PYTHON" -m unittest isaac_port.fieldkit.test_hand isaac_port.fieldkit.test_devices isaac_port.fieldkit.test_fieldkit -v 2>&1 | tee "$ROOT/reports/fieldkit-tests.log"
