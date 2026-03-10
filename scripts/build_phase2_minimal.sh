#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
colcon --log-base log_phase2 build \
  --build-base build_phase2 \
  --install-base install_phase2 \
  --packages-select \
    integrated_mission_interfaces \
    mission_dispatcher \
    plc_adapter \
    mobility_planner_core \
    excavation_planner_core \
    mission_bringup \
    planner_client \
    autonomous_walk \
    point_cloud_processing_pkg \
    shovel_interfaces \
    plc_control
