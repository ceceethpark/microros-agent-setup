# ROS2 환경 설정

## 네트워크 구성

### 1. Windows PC (개발 머신)
- **Hostname**: wsl-65
- **OS**: Windows + WSL (Ubuntu 22.04)
- **ROS2 Distribution**: Humble
- **DDS**: rmw_fastrtps_cpp (Fast-DDS)
- **ROS_DOMAIN_ID**: 20
- **역할**: 개발 및 모니터링

### 2. Raspberry Pi 보드
- **Hostname**: ROS2_1-103
- **OS**: Ubuntu 24.04
- **ROS2 Distribution**: Jazzy
- **DDS**: rmw_fastrtps_cpp (Fast-DDS)
- **ROS_DOMAIN_ID**: 20
- **역할**: micro-ROS Agent
- **네트워크**:
  - LAN: 192.168.0.103
  - WiFi: 192.168.0.88
- **연결 장치**: ESP32-WROOM (micro-ROS)

### 3. Linux PC
- **Hostname**: ROS_1-154
- **OS**: Ubuntu 24.04
- **ROS2 Distribution**: Jazzy
- **DDS**: rmw_fastrtps_cpp (Fast-DDS)
- **ROS_DOMAIN_ID**: 20
- **IP**: 192.168.0.154
- **역할**: 테스트 노드 실행

## ROS2 멀티 머신 통신 설정

### ROS_DOMAIN_ID 설정
모든 머신에서 동일한 도메인 ID 사용:
```bash
export ROS_DOMAIN_ID=20
```

### DDS 미들웨어 확인
현재 사용 중인 DDS 확인:
```bash
# 환경 변수 확인
echo $RMW_IMPLEMENTATION

# ROS2 설정 전체 확인
ros2 doctor --report | grep rmw

# 또는 간단히
printenv | grep RMW
```

기본 DDS:
- Humble: Fast-DDS (rmw_fastrtps_cpp)
- Jazzy: rmw_fastrtps_cpp 또는 Cyclone DDS

### DDS 변경 (필요시)
**중요: 멀티 머신 통신 시 모든 머신이 동일한 DDS를 사용해야 합니다!**

```bash
# Cyclone DDS 사용
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Fast-DDS 사용 (기본값)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Fast-DDS 버퍼 문제 해결
"sequence size exceeds remaining buffer" 에러 발생 시:

**방법 1: Cyclone DDS로 변경 (권장)**
```bash
# Cyclone DDS 설치 (각 머신에서)
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# 환경 변수 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# .bashrc에 추가하여 영구 적용
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

**방법 2: Fast-DDS XML 설정**
```bash
# Fast-DDS 설정 파일 생성
cat > ~/fastdds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
            <maxMessageSize>65500</maxMessageSize>
            <sendBufferSize>1048576</sendBufferSize>
            <receiveBufferSize>4194304</receiveBufferSize>
        </transport_descriptor>
    </transport_descriptors>
    
    <participant profile_name="default_participant">
        <rtps>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF

# 환경 변수로 설정 파일 지정
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds.xml
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds.xml" >> ~/.bashrc
```

## micro-ROS Agent 설정 (Raspberry Pi)

### Agent 실행
```bash
# USB 연결 시
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# UDP 연결 시
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## 작업 공간 설정

```bash
# 작업 공간 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 의존성

- 필수 패키지:
  - 
  - 

- 선택 패키지:
  - 
  - 

## 참고 자료

- [ROS2 공식 문서](https://docs.ros.org/en/rolling/)
- 

---
