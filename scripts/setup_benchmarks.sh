#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m venv .benchmark-venv
.benchmark-venv/bin/python -m pip install --cache-dir /workspace/.cache/pip -r benchmarks/requirements.lock
.benchmark-venv/bin/python -m pip install --no-deps -e .
