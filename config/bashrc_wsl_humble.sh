# ROS2 환경 설정 - WSL (Ubuntu 22.04, Humble)
# 사용법: 이 내용을 ~/.bashrc 파일 끝에 추가

# ROS2 Humble 환경 로드
source /opt/ros/humble/setup.bash

# 작업 공간 설정 (있는 경우)
# source ~/ros2_ws/install/setup.bash

# ROS_DOMAIN_ID 설정 (멀티 머신 통신용)
export ROS_DOMAIN_ID=20

# Cyclone DDS 사용 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 네트워크 설정
export ROS_LOCALHOST_ONLY=0

# 추가 설정
export RCUTILS_COLORIZED_OUTPUT=1

# 호스트 정보
echo "ROS2 Environment Loaded: WSL (Humble)"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
