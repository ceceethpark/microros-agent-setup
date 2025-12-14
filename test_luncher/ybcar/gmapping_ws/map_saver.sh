#!/usr/bin/env bash
set -euo pipefail
# Simple wrapper to save the current /map topic to files using nav2_map_server's map_saver_cli
# Usage: ./map_saver.sh /path/to/output_basename

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 /full/path/output_basename (without extension)"
  exit 1
fi
OUT="$1"

echo "Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "Saving /map to ${OUT}.pgm and ${OUT}.yaml using map_saver_cli..."
ros2 run nav2_map_server map_saver_cli -f "${OUT}"
echo "Saved map -> ${OUT}.pgm, ${OUT}.yaml"
