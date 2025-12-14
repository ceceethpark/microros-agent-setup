# ROS2 환경 설정

## 빠른 설치 및 설정 (요약)

이 파일은 Pi 및 개발 머신에서 ROS2 및 micro-ROS Agent를 빠르게 설정하는 단계별 지침을 제공합니다.

## 환경 변수(권장)
모든 머신에서 동일한 Domain 및 원하는 RMW 구현체를 명시하세요.

```bash
export ROS_DOMAIN_ID=20
# 예: Cyclone DDS 권장
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Agent가 사용할 인터페이스 명시
export RMW_NETWORK_INTERFACE=wlan0
```

## 필수 패키지 설치 (예: Raspberry Pi, Ubuntu 24.04)
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions tcpdump git
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp  # ROS 배포판에 맞게 변경
```

## Cyclone DDS로 변경 (버퍼/Discovery 안정화 목적)
```bash
sudo apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=20" >> ~/.bashrc
echo "export RMW_NETWORK_INTERFACE=wlan0" >> ~/.bashrc
source ~/.bashrc
```

## micro-ROS Agent: 빌드 및 실행(개발 워크스페이스에서)
```bash
# 워크스페이스에서 빌드한 경우
source ~/microros_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RMW_NETWORK_INTERFACE=wlan0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v6
```

## systemd 서비스 예시 (권장 설정)
서비스 파일: `/etc/systemd/system/micro_ros_agent.service`
```ini
[Unit]
Description=micro-ROS Agent (serial)
After=network-online.target dev-ttyUSB0.device
Wants=network-online.target

[Service]
User=ros2
Environment="ROS_DOMAIN_ID=20"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
Environment="RMW_NETWORK_INTERFACE=wlan0"
ExecStart=/usr/bin/env ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v6
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

## 네트워크 문제 검사(간단 체크리스트)
- `/dev/ttyUSB0` 존재: `ls -l /dev/ttyUSB0`
- network-online 서비스: `systemctl status NetworkManager-wait-online.service` 또는 `systemctl status systemd-networkd-wait-online.service`
- Agent 로그: `journalctl -u micro_ros_agent -f`
- RTPS 패킷 캡처: `sudo tcpdump -i wlan0 -n udp and portrange 7400-7600`

## 참고: Fast-DDS 고급 설정
Fast-DDS를 계속 사용해야 한다면 `fastdds.xml`로 버퍼·전송 설정을 조정하세요(예: `maxMessageSize`, send/receive buffer).

---

파일 역할: 자세한 분석 및 사례는 `4_notes.md`, 테스트 로그는 `5_test_log.md`를 참조하세요。
