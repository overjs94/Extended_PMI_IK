#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_python="${EXTENDED_PMI_PYTHON:-python3}"
build_cache="${EXTENDED_PMI_BUILD_CACHE:-/tmp/extended-pmi-build-cache}"
"$build_python" -m pip install --cache-dir "$build_cache/pip" -r requirements.lock
"$build_python" -m pip install --cache-dir "$build_cache/pip" --no-deps -e .
npm --prefix web ci --cache "$build_cache/npm" --no-audit --no-fund
npm --prefix web run build
"$build_python" scripts/fetch_models.py
