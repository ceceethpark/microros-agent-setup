# ROS2 환경 설정 파일

각 머신별 .bashrc 설정 파일입니다.

## 파일 목록

- `bashrc_wsl_humble.sh` - WSL (Ubuntu 22.04, Humble) 설정
- `bashrc_pi_jazzy.sh` - Raspberry Pi (Ubuntu 24.04, Jazzy) 설정
- `bashrc_linux_jazzy.sh` - Linux PC (Ubuntu 24.04, Jazzy) 설정

## 적용 방법

### 1. WSL (p@thparki7)
```bash
# 파일 내용을 .bashrc에 추가
cat bashrc_wsl_humble.sh >> ~/.bashrc
source ~/.bashrc

# 또는 수동으로
nano ~/.bashrc
# bashrc_wsl_humble.sh 내용을 복사하여 끝에 추가
```

### 2. Raspberry Pi (ros2@ubuntu2404, ROS2_1-103)
```bash
cat bashrc_pi_jazzy.sh >> ~/.bashrc
source ~/.bashrc
```

### 3. Linux PC (ros@ubuntu2404, ROS_1-154)
```bash
cat bashrc_linux_jazzy.sh >> ~/.bashrc
source ~/.bashrc
```

## 사전 준비

### Cyclone DDS 설치

**WSL (Humble):**
```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

**Pi & Linux PC (Jazzy):**
```bash
sudo apt update
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

## 확인

모든 머신에서 새 터미널을 열고:
```bash
echo $ROS_DOMAIN_ID        # 20 출력되어야 함
echo $RMW_IMPLEMENTATION    # rmw_cyclonedds_cpp 출력되어야 함
ros2 node list              # 에러 없이 실행되어야 함
```

## 주의사항

- **모든 머신을 동시에 설정**해야 합니다
- 일부 머신만 Cyclone DDS로 변경하면 통신 불가
- 기존 Fast-DDS XML 설정(FASTRTPS_DEFAULT_PROFILES_FILE)이 있다면 제거하세요

---
