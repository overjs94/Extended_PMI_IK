#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Existing isolated task checkout: do not create a Git worktree.
python3 -m venv .venv
.venv/bin/python -m pip install --cache-dir /workspace/.cache/pip -r requirements.lock
.venv/bin/python -m pip install --no-deps -e .
npm --prefix web ci --cache /workspace/.cache/npm --no-audit --no-fund
npm --prefix web run build

.venv/bin/python scripts/fetch_models.py
