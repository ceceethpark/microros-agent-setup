# WSL Ubuntu 24.04 + ROS2 Jazzy 설치 가이드

## 방법 1: 새 WSL 인스턴스 설치 (권장)

### PowerShell에서 실행:

```powershell
# 1. 사용 가능한 배포판 확인
wsl --list --online

# 2. Ubuntu 24.04 설치
wsl --install -d Ubuntu-24.04

# 3. 설치 후 Ubuntu 24.04 실행
wsl -d Ubuntu-24.04

# 4. 사용자 계정 생성 (프롬프트가 나오면)
# Username과 Password 입력
```

### Ubuntu 24.04 내부에서 ROS2 Jazzy 설치:

```bash
# 1. 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# 2. locale 설정
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 3. ROS2 GPG 키 추가
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 4. ROS2 저장소 추가
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# 6. ROS2 Jazzy 설치
sudo apt install -y ros-jazzy-desktop

# 7. 개발 도구 설치
sudo apt install -y python3-pip python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-argcomplete

# 8. rosdep 초기화
sudo rosdep init
rosdep update

# 9. Cyclone DDS 설치
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp

# 10. .bashrc 설정
cat >> ~/.bashrc << 'EOF'

# ========================================
# ROS2 환경 설정 - WSL Ubuntu 24.04 (Jazzy + Cyclone DDS)
# ========================================

# ROS2 Jazzy 환경 로드
source /opt/ros/jazzy/setup.bash

# 작업 공간 설정 (있는 경우)
# source ~/ros2_ws/install/setup.bash

# ROS_DOMAIN_ID 설정 (멀티 머신 통신용)
export ROS_DOMAIN_ID=20

# Cyclone DDS 사용
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 네트워크 설정
export ROS_LOCALHOST_ONLY=0

# 화면 출력 색상
export RCUTILS_COLORIZED_OUTPUT=1

# 환경 정보 표시
echo "=== ROS2 Environment ==="
echo "Device: WSL Ubuntu 24.04"
echo "Distribution: Jazzy"
echo "Domain ID: $ROS_DOMAIN_ID"
echo "RMW: $RMW_IMPLEMENTATION"
echo "IP: $(hostname -I | awk '{print $1}')"
echo "========================"

EOF

# 11. 설정 적용
source ~/.bashrc

# 12. 테스트
ros2 --version
echo $ROS_DISTRO
ros2 topic list
```

## 방법 2: 기존 Ubuntu 22.04를 24.04로 업그레이드 (복잡함)

**주의**: 데이터 백업 필수!

```bash
# 기존 WSL에서
sudo apt update && sudo apt upgrade -y
sudo do-release-upgrade -d
```

## 설치 확인

```bash
# Ubuntu 버전 확인
lsb_release -a

# ROS2 버전 확인
echo $ROS_DISTRO  # jazzy 출력

# DDS 확인
echo $RMW_IMPLEMENTATION  # rmw_cyclonedds_cpp 출력

# 멀티 머신 테스트
ros2 node list
ros2 topic list
```

## 기존 Ubuntu 22.04 유지하려면

PowerShell에서 두 버전 동시 사용:
```powershell
# Ubuntu 22.04 (Humble) 실행
wsl -d Ubuntu

# Ubuntu 24.04 (Jazzy) 실행
wsl -d Ubuntu-24.04
```

---
설치 후 새 터미널에서 모든 머신이 동일한 Jazzy 버전으로 통일됩니다!
