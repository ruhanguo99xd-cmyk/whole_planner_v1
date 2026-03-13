#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if ! dpkg-query -W -f='${db:Status-Status}\n' libnlopt-dev 2>/dev/null | grep -qx installed; then
  echo "Missing required system package: libnlopt-dev" >&2
  echo "Install it with: sudo apt-get update && sudo apt-get install -y libnlopt-dev" >&2
  exit 1
fi

colcon --log-base log_install_fix build \
  --build-base build \
  --install-base install \
  --packages-select \
    integrated_mission_interfaces \
    mission_dispatcher \
    mission_operator_hmi \
    plc_adapter \
    mobility_planner_core \
    excavation_planner_core \
    mission_bringup \
    planner_client \
    autonomous_walk \
    point_cloud_processing_pkg \
    PRSdata_send \
    truck_perceive \
    shovel_interfaces \
    plc_control \
    tra_planning \
    load \
    return
