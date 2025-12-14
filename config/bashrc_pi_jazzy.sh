# ROS2 환경 설정 - Raspberry Pi (Ubuntu 24.04, Jazzy)
# 사용법: 이 내용을 ~/.bashrc 파일 끝에 추가

# ROS2 Jazzy 환경 로드
source /opt/ros/jazzy/setup.bash

# 작업 공간 설정 (있는 경우)
# source ~/ros2_ws/install/setup.bash

# ROS_DOMAIN_ID 설정 (멀티 머신 통신용)
export ROS_DOMAIN_ID=20

# Cyclone DDS 사용 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 네트워크 설정
export ROS_LOCALHOST_ONLY=0

# Pi 네트워크 정보
# LAN: 192.168.0.103
# WiFi: 192.168.0.88

# 추가 설정
export RCUTILS_COLORIZED_OUTPUT=1

# 호스트 정보
echo "ROS2 Environment Loaded: Raspberry Pi (Jazzy)"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "IP: $(hostname -I | awk '{print $1}')"
