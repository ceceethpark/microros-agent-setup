# ============================================
# ROS2 Jazzy 환경 설정
# ============================================

# ROS2 Jazzy 기본 설정
source /opt/ros/jazzy/setup.bash

# micro-ROS 워크스페이스 (필요시 주석 해제)
# source ~/microros_ws/install/setup.bash

# ============================================
# ROS2 네트워크 설정
# ============================================

# Domain ID (YB_Car 전용 네트워크)
export ROS_DOMAIN_ID=20

# DDS 미들웨어 (Cyclone DDS 사용)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 네트워크 인터페이스 (WiFi 사용)
export RMW_NETWORK_INTERFACE=wlan0

# ============================================
# 편의 기능 별칭
# ============================================

# 상태 확인
alias ros2-status='echo "Domain: $ROS_DOMAIN_ID | DDS: $RMW_IMPLEMENTATION | Interface: $RMW_NETWORK_INTERFACE"'

# micro-ROS Agent 제어
alias agent-status='sudo systemctl status micro_ros_agent'
alias agent-log='journalctl -u micro_ros_agent -n 50 --no-pager'
alias agent-logf='journalctl -u micro_ros_agent -f'
alias agent-restart='sudo systemctl restart micro_ros_agent'

# ROS2 단축 명령어
alias r2n='ros2 node list'
alias r2t='ros2 topic list'
alias r2te='ros2 topic echo'

# ROS_DOMAIN_ID 설정 (멀티 머신 통신용)
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export RCUTILS_COLORIZED_OUTPUT=1

# 환경 정보 출력
if [ -z "$SSH_TTY" ]; then
  # 로컬 콘솔에서만 출력
  echo "=== ROS2 Environment ==="
  echo "Device: Raspberry Pi"
  echo "Distribution: Jazzy"
  echo "Domain ID: $ROS_DOMAIN_ID"
  echo "RMW: $RMW_IMPLEMENTATION"
  echo "IP: $(hostname -I | awk '{print $1}')"
  echo "========================"
fi

# CycloneDDS 설정 파일 경로 (중복 제거)
export CYCLONEDDS_URI=file:///home/ros2/.config/cyclonedds/cyclonedds.xml

# 프롬프트에 ROS2 정보 표시 (선택사항, 주석 해제하려면 아래 줄의 # 제거)
# PS1="\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\] [ROS:$ROS_DOMAIN_ID] \$ "

# ============================================
# micro-ROS Agent 제어 (권장: systemd 사용)
# ============================================
# 권장: systemd 서비스로 관리하세요. .bashrc에서 자동 실행하지 마십시오.
# Systemd 사용 예:
#   sudo systemctl enable --now micro_ros_agent
#   sudo systemctl status micro_ros_agent

# 편의 alias (수동/디버그용)
alias agent-enable='sudo systemctl enable --now micro_ros_agent'
alias agent-status='sudo systemctl status micro_ros_agent'
alias agent-start='sudo systemctl start micro_ros_agent'
alias agent-stop='sudo systemctl stop micro_ros_agent'
alias agent-restart='sudo systemctl restart micro_ros_agent'

# 수동 실행 (디버그용 — 서비스 대신 직접 실행하려면 사용)
# 포트와 baudrate는 환경에 맞춰 조정하세요 (/dev/ttyUSB0, 921600 등)
alias agent-run-manual='ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v6'

# tmux에서 실행하는 헬퍼 함수 (tmux 설치 필요)
agent-tmux(){
  if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux가 설치되어 있지 않습니다. sudo apt install tmux"
    return 1
  fi
  tmux new-session -d -s micro_ros_agent "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v6"
  echo "micro_ros_agent started in tmux session 'micro_ros_agent'"
}

