#!/usr/bin/env bash
set -e
# Run teleop_twist_keyboard in the current terminal to publish cmd_vel
echo "Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "Start teleop (will use your keyboard in this terminal). Ctrl-C to exit."
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard
