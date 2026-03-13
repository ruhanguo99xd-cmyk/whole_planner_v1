#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
echo "[deprecated] use: bash scripts/build_workspace.sh"
exec bash scripts/build_workspace.sh "$@"
