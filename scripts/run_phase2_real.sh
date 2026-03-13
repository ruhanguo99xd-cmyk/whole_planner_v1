#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
echo "[deprecated] use: bash scripts/run_profile.sh integrated"
exec bash scripts/run_profile.sh integrated "$@"
