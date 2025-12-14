#!/usr/bin/env bash
set -eu

# collect_logs.sh
# Usage: ./collect_logs.sh [DURATION_SECONDS] [IFACE] [OUTDIR]
# Example: ./collect_logs.sh 60 wlan0 ./doc/logs

DURATION=${1:-60}
IFACE=${2:-wlan0}
OUTDIR=${3:-./doc/logs}
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
mkdir -p "$OUTDIR"
PCAP="$OUTDIR/rtps_$TIMESTAMP.pcap"
AGENT_LOG="$OUTDIR/agent_$TIMESTAMP.log"
SERIAL_BIN="$OUTDIR/serial_$TIMESTAMP.bin"

echo "Collecting logs: duration=$DURATION iface=$IFACE outdir=$OUTDIR"

# prerequisites
if ! command -v tcpdump >/dev/null 2>&1; then
  echo "tcpdump not found. Install: sudo apt install tcpdump" >&2
  exit 1
fi

# remember systemd state
SYSTEMD_ACTIVE=0
if systemctl is-active --quiet micro_ros_agent; then
  SYSTEMD_ACTIVE=1
  echo "Stopping micro_ros_agent systemd service..."
  sudo systemctl stop micro_ros_agent
fi

# start tcpdump
echo "Starting tcpdump on $IFACE -> $PCAP"
sudo tcpdump -i "$IFACE" -n udp and portrange 7400-7600 -w "$PCAP" &
TCPDUMP_PID=$!

# source workspace if present
if [ -f "$HOME/microros_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$HOME/microros_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export RMW_NETWORK_INTERFACE=${RMW_NETWORK_INTERFACE:-$IFACE}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-20}

# run micro-ROS Agent verbosely
echo "Starting micro-ROS Agent (verbose) -> $AGENT_LOG"
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v9 |& tee "$AGENT_LOG" &
AGENT_PID=$!

# let things run for duration
echo "Running capture for $DURATION seconds..."
sleep "$DURATION"

# short serial capture
echo "Capturing serial (10s) -> $SERIAL_BIN"
sudo timeout 10 cat /dev/ttyUSB0 > "$SERIAL_BIN" || true

# cleanup
echo "Stopping tcpdump (pid $TCPDUMP_PID) and agent (pid $AGENT_PID)"
sudo kill "$TCPDUMP_PID" 2>/dev/null || true
kill "$AGENT_PID" 2>/dev/null || true
sleep 1

if [ "$SYSTEMD_ACTIVE" -eq 1 ]; then
  echo "Restarting micro_ros_agent systemd service"
  sudo systemctl start micro_ros_agent
fi

echo "Done. Logs:"
echo "  - $PCAP"
echo "  - $AGENT_LOG"
echo "  - $SERIAL_BIN"
exit 0
