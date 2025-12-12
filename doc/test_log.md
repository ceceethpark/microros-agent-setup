# ROS2 테스트 로그

테스트 진행 상황과 결과를 날짜별로 기록합니다.

## 2025-12-12

### 테스트 환경
- OS: Windows
- ROS2 버전: 
- Python 버전: 

### 진행 내용
- 프로젝트 초기 설정
- 문서 구조 생성
- 멀티 머신 환경 확인 (3개 머신 모두 Fast-DDS, ROS_DOMAIN_ID=20)
- 통신 테스트 시작

### 테스트 결과

#### WSL 환경 확인 (p@thparki7, wsl-65)
- **OS**: Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- **ROS_DISTRO**: humble
- **DDS**: rmw_fastrtps_cpp (Fast-DDS)
- **문제**: "sequence size exceeds remaining buffer" 에러 발생
- **명령어**: `ros2 node list` 실패

#### 노드 확인 성공 (ubuntu2404, ROS2_1-103 Pi)
- **머신**: ubuntu2404 (ROS2_1-103 Pi 보드)
- **DDS**: rmw_fastrtps_cpp (Fast-DDS)
- **실행 노드**: `/YB_Car_Node`
- **상태**: 정상 작동 중
- **명령어**: `ros2 node list` 정상 실행

#### DDS 상태
- **모든 머신**: rmw_fastrtps_cpp (Fast-DDS) 사용 중 - 동일함 ✓
- **문제**: WSL에서만 버퍼 에러 발생 (환경별 차이)

#### 멀티 머신 통신 테스트 성공 ✓
**테스트 1**: WSL → 다른 머신들
- **발행**: WSL에서 `ros2 topic pub /test std_msgs/String "data: 'from wsl'"`
- **수신 1**: ros2@ubuntu2404 (ROS2_1-103 Pi) - 정상 수신 ✓
- **수신 2**: ros@ubuntu2404 (ROS_1-154 Linux PC) - 정상 수신 ✓
- **결과**: 멀티 머신 토픽 통신 정상 작동!

#### YB_Car_Node 분석 (Pi에서 실행)
**노드 정보**:
- **Subscribers** (수신 토픽):
  - `/beep`: std_msgs/msg/UInt16
  - `/cmd_vel`: geometry_msgs/msg/Twist
  - `/servo_s1`: std_msgs/msg/Int32
  - `/servo_s2`: std_msgs/msg/Int32
  
- **Publishers** (발행 토픽):
  - `/battery`: std_msgs/msg/UInt16
  - `/imu`: sensor_msgs/msg/Imu
  - `/odom_raw`: nav_msgs/msg/Odometry
  - `/scan`: sensor_msgs/msg/LaserScan

- **Service Servers**: 없음
- **Service Clients**: 없음
- **Action Servers**: 없음
- **Action Clients**: 없음

**노드 정보 접근**:
- Pi (ros2@ubuntu2404): `ros2 node info /YB_Car_Node` 성공 ✓
- Linux PC (ros@ubuntu2404): `ros2 node info /YB_Car_Node` 성공 ✓
- WSL (p@thparki7): "Unable to find node '/YB_Car_Node'" 실패 ✗
  - 원인: Fast-DDS 버퍼 제한으로 노드 discovery 실패
  - 참고: 토픽 통신은 정상 작동


### 이슈 및 해결방법

**이슈 #1: WSL Fast-DDS 버퍼 크기 에러**
- 상태: 해결 시도 중
- 문제: WSL 환경에서 `ros2 node list`, `ros2 node info` 실행 불가
- 원인: Fast-DDS의 버퍼 크기 제한 (WSL 네트워크 특성)
- **시도한 해결 방법**:
  - ✗ Fast-DDS XML 설정 (FASTRTPS_DEFAULT_PROFILES_FILE) - 효과 없음
  - 다음: Cyclone DDS로 변경 필요
- **영향**:
  - ✗ 노드 discovery 실패 (ros2 node 명령어 사용 불가)
  - ✓ 토픽 통신은 정상 작동 (pub/sub/echo 가능)
- **DDS 상태**: 모든 머신이 Fast-DDS 사용 중 (동일함 ✓)
- **권장 조치**: WSL을 포함한 모든 머신을 Cyclone DDS로 변경

### 다음 계획

**현재 상태**: 
- ✓ 멀티 머신 토픽 통신 정상 작동
- ✓ YB_Car_Node 정보 확인 완료

**다음 단계:**
1. ✓ 멀티 머신 통신 테스트 완료
2. ✓ YB_Car_Node 상세 분석 완료
3. ESP32 micro-ROS 통신 확인
   - micro-ROS Agent 상태 확인
   - ESP32에서 발행/구독하는 토픽 확인
   - YB_Car_Node와 ESP32 연결 관계 파악
4. 실제 사용 시나리오 테스트
   - cmd_vel 토픽으로 제어 명령 테스트
   - 센서 데이터(imu, scan) 모니터링

---