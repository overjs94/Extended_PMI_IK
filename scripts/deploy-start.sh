#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
exec "${EXTENDED_PMI_PYTHON:-python3}" -m uvicorn extended_pmi_ik.api:app --host 0.0.0.0 --port "${PORT:-8000}"
