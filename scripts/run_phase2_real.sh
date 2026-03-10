#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source install_phase2/setup.bash
ros2 launch mission_bringup phase2_real.launch.py
