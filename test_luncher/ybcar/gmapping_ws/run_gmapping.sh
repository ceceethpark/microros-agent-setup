#!/usr/bin/env bash
set -e
# Simple helper to build (if needed) and run the gmapping launch for this workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

cd "$SCRIPT_DIR"
if [ -f install/setup.bash ]; then
  echo "Sourcing workspace install/setup.bash"
  source install/setup.bash
fi

echo "Launching gmapping (yahboomcar_nav map_gmapping_launch.py)..."
ros2 launch yahboomcar_nav map_gmapping_launch.py
