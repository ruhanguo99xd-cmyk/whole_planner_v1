#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

profile="${1:-}"
if [[ -z "$profile" ]]; then
  echo "Usage: bash scripts/run_profile.sh <mock|integrated|hmi> [extra ros args]" >&2
  exit 1
fi
shift || true

if [[ ! -f install/setup.bash ]]; then
  echo "install/setup.bash not found. Build the workspace first:" >&2
  echo "  bash scripts/build_workspace.sh" >&2
  exit 1
fi

set +u
source install/setup.bash
set -u

case "$profile" in
  mock)
    ros2 launch mission_bringup phase1_mock.launch.py "$@"
    ;;
  integrated)
    ros2 launch mission_bringup phase2_real.launch.py "$@"
    ;;
  hmi)
    ros2 run mission_operator_hmi integrated_operator_hmi "$@"
    ;;
  *)
    echo "Unsupported profile: $profile" >&2
    echo "Supported profiles: mock, integrated, hmi" >&2
    exit 1
    ;;
esac
