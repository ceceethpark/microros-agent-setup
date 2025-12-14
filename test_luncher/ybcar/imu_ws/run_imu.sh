#!/usr/bin/env bash
set -e

# Simple helper to source ROS and launch the IMU bringup.
# Edit the ros2 launch line to match your IMU driver package and launch file.

source /opt/ros/jazzy/setup.bash
# if built in-workspace, source install/setup.bash
if [ -f "../install/setup.bash" ]; then
  source ../install/setup.bash
fi

# Change these to your package/launch
ros2 launch imu_driver imu_bringup.launch.py
