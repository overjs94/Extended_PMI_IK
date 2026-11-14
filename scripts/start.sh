#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OPENBLAS_NUM_THREADS=1
exec .venv/bin/python -m uvicorn extended_pmi_ik.api:app --host 0.0.0.0 --port "${PORT:-8000}"
