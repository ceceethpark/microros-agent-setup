#!/usr/bin/env bash
# Copy of scripts/wait_for_agent.sh for manual transfer to Pi
# Usage on Pi: sudo cp ~/prj_ROS2/doc/wait_for_agent.sh /home/ros2/bin/wait_for_agent.sh && sudo chmod +x /home/ros2/bin/wait_for_agent.sh

set -euo pipefail

timeout=${1:-30}
device=${2:-/dev/ttyUSB0}
check_session=${3:-0}

end_time=$((SECONDS + timeout))
echo "[wait_for_agent] waiting for device $device (timeout ${timeout}s)"
while [ $SECONDS -lt $end_time ]; do
  if [ -e "$device" ]; then
    echo "[wait_for_agent] device $device present"
    break
  fi
  sleep 0.5
done

if [ ! -e "$device" ]; then
  echo "[wait_for_agent] ERROR: timeout waiting for $device" >&2
  exit 1
fi

if [ "$check_session" -ne 1 ]; then
  exit 0
fi

echo "[wait_for_agent] waiting for micro-ROS 'Created session' message (timeout ${timeout}s)"
start_time=$(date +%s)
while [ $(date +%s) -lt $((start_time + timeout)) ]; do
  if journalctl -u micro_ros_agent --since "-${timeout} seconds" | grep -q "Created session"; then
    echo "[wait_for_agent] Created session found in journal"
    exit 0
  fi
  sleep 1
done

echo "[wait_for_agent] ERROR: timeout waiting for 'Created session' in journal" >&2
exit 2
