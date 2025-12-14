#!/bin/bash
# Topic Pub/Sub Tester for ROS2
# Usage: bash topic_tester.sh [list|info|echo|pub|monitor]

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== ROS2 Topic Tester ===${NC}\n"

# 환경 확인
echo -e "${YELLOW}[1/6] Checking environment...${NC}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-not set}"
echo ""

# 노드 목록
echo -e "${YELLOW}[2/6] ROS2 Nodes:${NC}"
if ros2 node list 2>/dev/null; then
    echo ""
else
    echo -e "${RED}Warning: ros2 node list failed (normal on WSL)${NC}\n"
fi

# 토픽 목록 (list)
echo -e "${YELLOW}[3/6] Available Topics:${NC}"
if [ "$1" == "list" ] || [ -z "$1" ]; then
    ros2 topic list -v
    echo ""
fi

# 토픽 정보 (info)
if [ "$1" == "info" ] && [ -n "$2" ]; then
    echo -e "${YELLOW}[4/6] Topic Info for $2:${NC}"
    ros2 topic info "$2"
    echo ""
fi

# 토픽 구독 (echo)
if [ "$1" == "echo" ] && [ -n "$2" ]; then
    echo -e "${YELLOW}[4/6] Echoing topic: $2${NC}"
    echo "(Press Ctrl+C to stop)"
    ros2 topic echo "$2"
fi

# 토픽 발행 (pub)
if [ "$1" == "pub" ] && [ -n "$2" ] && [ -n "$3" ]; then
    echo -e "${YELLOW}[4/6] Publishing to: $2${NC}"
    echo "Message: $3"
    ros2 topic pub "$2" "$3"
fi

# 에이전트 모니터링 (monitor)
if [ "$1" == "monitor" ]; then
    echo -e "${YELLOW}[4/6] Monitoring Topics (refresh every 2 sec)...${NC}"
    while true; do
        clear
        echo -e "${BLUE}=== ROS2 Topic Monitor ===${NC}"
        echo "Time: $(date '+%Y-%m-%d %H:%M:%S')"
        echo ""
        echo -e "${YELLOW}Topics:${NC}"
        ros2 topic list
        echo ""
        echo -e "${YELLOW}Agent Status:${NC}"
        systemctl is-active micro_ros_agent.service || echo "Agent not running"
        echo ""
        echo "Press Ctrl+C to exit"
        sleep 2
    done
fi

# 사용법
if [ "$1" == "help" ] || [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    cat << EOF
Usage: bash topic_tester.sh [command] [args]

Commands:
  list                      List all topics (default)
  info <topic_name>         Show topic information
  echo <topic_name>         Subscribe and display topic data
  pub <topic_name> <type>   Publish to topic
  monitor                   Monitor topics (updates every 2 sec)
  help                      Show this help message

Examples:
  bash topic_tester.sh list
  bash topic_tester.sh info /battery
  bash topic_tester.sh echo /battery
  bash topic_tester.sh echo /imu
  bash topic_tester.sh monitor

  # Publish velocity command
  bash topic_tester.sh pub /cmd_vel geometry_msgs/msg/Twist

EOF
fi

# 에이전트 상태 확인
echo -e "${YELLOW}[5/6] Micro-ROS Agent Status:${NC}"
systemctl status micro_ros_agent.service --no-pager || echo "Agent service not found"
echo ""

# 네트워크 인터페이스
echo -e "${YELLOW}[6/6] Network Interfaces:${NC}"
ip addr | grep -E "inet|link/ether" | grep -A1 "eth0\|wlan0"
echo ""

echo -e "${GREEN}Done!${NC}"
