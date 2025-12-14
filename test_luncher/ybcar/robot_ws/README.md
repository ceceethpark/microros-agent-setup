# Robot Bringup & Test Guide

이 문서는 `robot_ws`(로봇 드라이버·bringup 패키지)를 빠르게 빌드하고 현장에서 테스트하기 위한 가이드입니다. `gmapping_ws`와 `imu_ws`에서 제공하는 테스트 흐름과 연계하여 사용하세요.

## 전제
- ROS2 Jazzy 환경에서 동작 (`source /opt/ros/jazzy/setup.bash`).
- 로봇에는 최소한 다음이 구현되어 있어야 함:
  - `cmd_vel` 구독(주행 명령 수신)
  - `odom` 발행(오도메트리)
  - 레이저 스캔(`scan`) 발행 또는 IMU(`/imu`) 발행(선택적)
  - TF 트리: `base_link` ↔ `odom` 등
- micro-ROS(ESP32 등)와 연동 시 micro-ROS Agent가 올바르게 구성되어 있어야 함(시리얼/UDP 등).

## 빌드
```bash
source /opt/ros/jazzy/setup.bash
cd test_luncher/ybcar/robot_ws
colcon build --packages-select <robot_bringup_package>
source install/setup.bash
```
`<robot_bringup_package>`를 실제 사용중인 패키지명으로 교체하세요.

## 실행(간단)
- 드라이버/bringup 예시(패키지 및 런치파일명은 환경에 맞게 변경):

```bash
ros2 launch robot_bringup bringup.launch.py
```

- 제공된 실행 스크립트 사용:

```bash
./run_robot.sh
```

### 스크립트 동작
`run_robot.sh`는 ROS 환경을 소싱하고 로봇 bringup 런치를 실행하는 간단한 헬퍼입니다. 필요에 맞춰 수정하세요.

## 실행 확인
- `cmd_vel`이 퍼블리시 되는지(또는 구독 중인지) 확인:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

- `odom` 확인:

```bash
ros2 topic echo /odom --once
ros2 topic hz /odom
```

- 레이저/IMU 토픽 확인:

```bash
ros2 topic echo /scan
ros2 topic echo /imu
```

- TF 확인: RViz 또는 `ros2 run tf2_tools view_frames` 사용.

## 통합 포인트 (gmapping_ws, imu_ws, micro-ROS)
- GMapping 사용 시:
  - `gmapping_ws`의 `slam_gmapping`은 `scan`과 `odom`/TF를 사용합니다. `robot_ws`가 `scan`과 `odom`을 안정적으로 제공해야 합니다.
  - 권장 주행 패턴(맵 품질 향상): 천천히 직선 주행(0.1–0.3 m/s)과 완만한 회전(0.2–0.5 rad/s)을 섞어 영역을 스트라이프(zig-zag) 형태로 커버하세요.
- IMU 통합(옵션): IMU가 있다면 `imu_ws`의 EKF(`robot_localization`)를 통해 `odom`을 보정해 주세요.
- micro-ROS 연동:
  - Agent가 시리얼 모드인 경우 디바이스 권한(`/dev/ttyUSB*`)과 baudrate(예: 921600)를 확인하세요.
  - Agent를 systemd 서비스로 띄우면 부팅시 자동 연결성이 좋아집니다(기존 `config/` 자료를 참고).

## 권장 점검/디버깅 절차
1. 하드웨어 연결과 권한 확인(`/dev/tty*`, USB 전원 등).
2. `ros2 topic list`로 필수 토픽(`/cmd_vel`, `/odom`, `/scan` 등) 존재 확인.
3. `ros2 topic echo`로 메시지 내용/주기 확인.
4. `ros2 run tf2_ros static_transform_publisher ...`로 임시 TF를 만들어 테스트.
5. 로그(`journalctl` 또는 노드의 stdout)를 확인하여 드라이버 오류 체크.

## 예시 파라미터 & 튜닝 포인트
- 컨트롤 루프 주파수: 10–50 Hz (모터 드라이버와 요구 응답성에 따라 조정)
- 속도 리미트: 로봇 물리 성능에 맞게 `max_vel`/`max_rot` 설정
- 오도메트리 필터: 저속에서는 엔코더 기반 오도메트리가 더 안정적일 수 있음. IMU+EKF 통합 권장

## 수동 주행(텔레옵)
- 텔레옵 사용 예:

```bash
# 터미널 A: 로봇 bringup
./run_robot.sh

# 터미널 B: 텔레옵 실행
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

텔레옵이 `cmd_vel`을 퍼블리시하면 `robot_ws`의 드라이버가 이를 받아 모터를 제어해야 합니다.

## 맵 생성 워크플로(요약)
1. `robot_ws`로 `scan`/`odom`/TF가 안정적으로 나오는지 확인
2. `gmapping_ws`에서 `slam_gmapping` 실행(또는 `yahboomcar_nav` 통합 런치)
3. 수동 주행 또는 자동 탐색으로 맵 생성
4. `map_saver_cli`로 맵 저장

## 자주 발생하는 문제 및 해결 팁
- `cmd_vel` 입력이 있는데 로봇이 움직이지 않음: 드라이버가 `cmd_vel`을 구독하는지, 권한 및 모터 전원 확인
- `odom`이 부정확함: 바퀴 슬립, 인코더 분해능, 캘리브레이션 확인
- TF 불일치: `base_link`/`odom` 프레임 네이밍과 브로드캐스트 지점 확인

## 추가 자료 및 다음 단계
- 자동 탐색/Navigation2 통합: `nav2` 런치·파라미터 추가 고려
- 로컬화 안정성 향상: `robot_localization`(EKF) 튜닝 및 IMU 캘리브레이션

## Bringup 예시 런치 파일
저는 `robot_ws`에 간단한 예시 런치 파일을 추가했습니다. 위치:

- `launch/bringup_example_launch.py` — 워크스페이스/시스템에 있는 `robot_bringup` 패키지의 `bringup.launch.py`를 실행하는 scaffold 런치입니다.

사용 예:

```bash
# ROS 환경 소스 후
ros2 launch robot_ws launch/bringup_example_launch.py
```

무엇을 테스트하나:
- 하드웨어 bringup이 정상적으로 실행되는지(드라이버 노드가 올라오는지)
- `cmd_vel`, `odom`, 센서 토픽(`scan`/`imu`) 및 TF가 정상적으로 발행되는지

편집 팁:
- 실제 패키지/런치 이름이 다르면 `launch/bringup_example_launch.py`의 `ExecuteProcess` 명령을 수정하세요.

## 통합 맵 런치(Bringup + IMU EKF + GMapping)
통합 테스트를 위한 간단한 통합 런치도 추가했습니다:

- `launch/integrated_map_launch.py` — 순차적으로 `robot_bringup` bringup, `imu_ws`의 EKF 런치, 그리고 `yahboomcar_nav`의 `map_gmapping_launch.py`를 실행하는 scaffold 런치입니다.

사용 예:

```bash
ros2 launch robot_ws launch/integrated_map_launch.py
```

무엇을 테스트하나:
- 전체 통합 워크플로: 하드웨어 bringup → IMU 기반 EKF 보정 → GMapping 맵 생성
- 각 단계에서 토픽/TF/오도메트리/맵 퍼블리시가 정상 동작하는지 확인

주의 및 맞춤 설정:
- 패키지/런치 파일 이름(`robot_bringup`, `imu_ws`, `yahboomcar_nav`)은 환경에 맞게 바꾸셔야 합니다.
- 통합 런치는 각 서브런치(예: bringup, ekf, gmapping)가 독립적으로 동작해야 원활히 통합됩니다. 먼저 각 워크스페이스를 개별적으로 확인하세요.

## 통합 사용 가이드(권장 순서)
1. `robot_ws` 단독 실행으로 `cmd_vel`/`odom`/`scan`이 안정적으로 나오는지 확인
2. `imu_ws`의 `ekf_launch.py`를 실행하여 IMU→`odom` 보정 확인
3. `gmapping_ws`에서 `slam_gmapping`을 실행하여 맵 생성 테스트
4. 위 세 단계를 통합 런치(`integrated_map_launch.py`)로 검증

## 변경사항 요약
- 추가된 파일:
  - `launch/bringup_example_launch.py`
  - `launch/integrated_map_launch.py`
- README에 사용법 및 통합 테스트 절차를 문서화함

원하시면 제가 추가로 아래 작업을 진행하겠습니다:
- `integrated_map_launch.py`를 더 견고하게 수정(런치 포함 방식 변경, IncludeLaunchDescription 사용)
- `.gitattributes` 추가로 LF 고정
- 통합 런치에서 파라미터 전달(예: imu/odom 토픽명) 기능 추가
