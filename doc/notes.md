# 개발 노트

ROS2 개발 및 테스트 중 발견한 중요 사항들을 기록합니다.

## 주요 개념

### ROS2 기본 구조
- **Node**: ROS2의 기본 실행 단위
- **Topic**: 노드 간 비동기 통신
- **Service**: 요청-응답 방식의 동기 통신
- **Action**: 장기 실행 작업을 위한 통신

### 주요 명령어

```bash
# Linux 버전 확인
lsb_release -a          # 상세 정보
cat /etc/os-release     # 배포판 정보
uname -a                # 커널 정보

# ROS2 버전 확인
printenv | grep ROS     # ROS 관련 환경 변수
echo $ROS_DISTRO        # ROS 배포판 이름 (humble, jazzy 등)
# 주의: ros2 --version은 지원되지 않음, $ROS_DISTRO 사용

# 노드 실행
ros2 run <package_name> <executable_name>

# 토픽 리스트 확인
ros2 topic list

# 토픽 정보 확인
ros2 topic info <topic_name>

# 노드 정보 확인
ros2 node list
ros2 node info <node_name>

# 멀티 머신 통신 확인
ros2 topic list  # 다른 머신의 토픽이 보이는지 확인
ros2 topic echo <topic_name>  # 다른 머신의 토픽 데이터 수신 확인

# DDS 설정 확인
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
ros2 doctor --report
```

## 문제 해결

### "sequence size exceeds remaining buffer" 에러
Fast-DDS의 버퍼 크기 제한으로 인한 문제

**증상:**
- `ros2 node list` 실행 시 에러 발생
- 멀티 머신 통신 시 노드 정보 교환 실패

**해결 방법:**
1. **Cyclone DDS로 변경 (가장 간단)**
   - 모든 머신에서 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
   
2. **Fast-DDS XML 설정으로 버퍼 크기 증가**
   - [setup.md](setup.md)의 Fast-DDS XML 설정 참고

3. **네트워크 최적화**
   - MTU 크기 확인: `ip link show`
   - 방화벽 규칙 확인

**참고:** 모든 머신이 동일한 DDS 미들웨어를 사용해야 합니다.


## 유용한 팁

### Cyclone DDS vs Fast-DDS 비교

**Cyclone DDS의 장점:**
1. **WSL 호환성**: WSL 환경에서 버퍼 크기 문제 없음
2. **간단한 설정**: 추가 XML 설정 파일 불필요
3. **메모리 효율**: 더 적은 메모리 사용
4. **안정성**: 네트워크 환경 변화에 더 견고함
5. **멀티 머신 통신**: Discovery 프로토콜이 더 안정적

**Fast-DDS의 장점:**
1. **ROS2 기본값**: 대부분의 ROS2 배포판 기본 DDS
2. **성능**: 높은 처리량이 필요한 경우 더 빠를 수 있음
3. **QoS 옵션**: 더 세밀한 QoS(Quality of Service) 설정 가능

**현재 프로젝트 상황:**
- WSL에서 Fast-DDS 버퍼 문제 발생
- 토픽 통신은 정상이지만 노드 discovery 실패
- **권장**: 모든 머신을 Cyclone DDS로 통일

**변경 방법:**
```bash
# 설치
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# 설정
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```


## 참고 링크


---
