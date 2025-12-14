#!/usr/bin/env bash
# Install helper (copy) script for manual use on the Pi
# Usage on Pi (from where you copied this file):
#   sudo bash install_on_pi.sh

set -euo pipefail

REPO_DOC_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Installing from $REPO_DOC_DIR"

if [ ! -f "$REPO_DOC_DIR/wait_for_agent.sh" ]; then
  echo "ERROR: wait_for_agent.sh not found in $REPO_DOC_DIR" >&2
  exit 2
fi

sudo mkdir -p /home/ros2/bin
sudo cp "$REPO_DOC_DIR/wait_for_agent.sh" /home/ros2/bin/wait_for_agent.sh
sudo chmod +x /home/ros2/bin/wait_for_agent.sh
sudo chown ros2:ros2 /home/ros2/bin/wait_for_agent.sh || true

sudo mkdir -p /etc/systemd/system/micro_ros_agent.service.d
sudo cp "$REPO_DOC_DIR/override.conf" /etc/systemd/system/micro_ros_agent.service.d/override.conf

sudo systemctl daemon-reload
sudo systemctl restart micro_ros_agent
sudo systemctl status micro_ros_agent --no-pager -l || true

echo "Installed. Check logs: journalctl -u micro_ros_agent -f"
