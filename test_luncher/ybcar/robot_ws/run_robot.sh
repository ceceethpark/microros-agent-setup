#!/usr/bin/env bash
set -e

# Helper to source ROS and launch robot bringup.
# Edit the ros2 launch line to match your bringup package and launch file.

source /opt/ros/jazzy/setup.bash
if [ -f "../install/setup.bash" ]; then
  source ../install/setup.bash
fi

# Change to your bringup package/launch
ros2 launch robot_bringup bringup.launch.py
