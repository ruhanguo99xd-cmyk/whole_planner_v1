#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
colcon build --packages-up-to mission_bringup
source install/setup.bash
ros2 launch mission_bringup phase1_mock.launch.py
