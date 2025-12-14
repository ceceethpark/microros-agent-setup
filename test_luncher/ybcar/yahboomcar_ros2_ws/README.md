# Yahboomcar ROS2 Workspace

이 문서는 `yahboomcar_ros2_ws` 워크스페이스의 목적, 빌드·실행 방법, 통합 워크플로(맵 작성·로컬라이제이션·네비게이션) 및 디버깅 체크리스트를 정리합니다.

요약
- 목적: Yahboom 로봇의 bringup, 센서 드라이버, 맵핑(예: `gmapping`), EKF 통합, 그리고 Navigation2 연동을 재현·검증하기 위한 통합 워크스페이스.
- 위치(예시): `test_luncher/ybcar/yahboomcar_ros2_ws`

전제
- ROS2 Jazzy가 설치되어 있고 `source /opt/ros/jazzy/setup.bash`가 가능해야 합니다.
- 로봇 하드웨어: 라이다(`scan`), (선택)IMU(`/imu`), 엔코더/모터 드라이버, micro-ROS(ESP32) 등이 있고 드라이버가 존재해야 합니다.

빠른 빌드
```bash
source /opt/ros/jazzy/setup.bash
cd test_luncher/ybcar/yahboomcar_ros2_ws
colcon build
source install/setup.bash
```

주요 포함 항목(프로젝트별 차이 있음)
- `launch/` : bringup, integrated map, nav 관련 런치
- `params/` : gmapping, ekf(robot_localization), nav2 파라미터
- `src/` : 드라이버/브리지/노드 소스
- `scripts/` 또는 루트의 `run_*.sh` : 실행 헬퍼 스크립트
- `README.md`, `docs/` : 문서

핵심 런치(예)
- Bringup(하드웨어): `ros2 launch robot_bringup bringup.launch.py`
- Map 통합: `ros2 launch yahboomcar_nav map_gmapping_launch.py` 또는 `ros2 launch gmapping_ws slam_gmapping.launch.py`
- 통합(Bringup+EKF+GMapping): 제공되는 `integrated_map_launch.py` 사용 가능

권장 통합 테스트 순서
1. `robot_ws` 또는 `robot_bringup`으로 하드웨어 드라이버와 TF/odom 확인
   - 체크: `ros2 topic list`, `ros2 topic echo /odom --once`, `ros2 run tf2_tools view_frames`
2. IMU가 있으면 `imu_ws`의 `ekf_launch.py`로 EKF 실행하여 odom 보정 확인
3. `gmapping_ws`에서 `slam_gmapping` 실행하여 맵 생성 테스트 (권장 주행 패턴 적용)
4. 맵 저장: `map_saver_cli` 사용 (nav2 map_server 필요)
5. Navigation2 통합(AMCL/Planner/Controller) — 1–4가 안정화된 뒤 수행

권장 주행 패턴 및 초기 파라미터
- 주행: 직진 0.1–0.3 m/s, 회전 0.2–0.5 rad/s, 영역을 스트라이프(zig-zag)로 커버
- gmapping 시작값:
  - `particles`: 60
  - `linearUpdate`: 0.5
  - `angularUpdate`: 0.2
  - `map_update_interval`: 3.0
  - `maxUrange`: 5.0
- EKF(예): `frequency: 30.0`, `two_d_mode: true`

## 자동 생성 인벤토리
워크스페이스의 런치/파라미터/소스 파일 목록을 자동으로 수집하는 인벤토리 파일을 추가했습니다.

- 파일: `INVENTORY.md`
- 생성 스크립트: `scripts/generate_inventory.py` — 실행하면 `INVENTORY.md`를 최신화합니다.

또한 상세 자동 인벤토리(`DETAILED_INVENTORY.md`)를 생성하는 스크립트를 추가했습니다:

- `scripts/generate_detailed_inventory.py` — 런치 파일 내 포함 노드/IncludeLaunchDescription 및 파라미터 파일의 최상위 키를 추출하여 `DETAILED_INVENTORY.md`를 생성합니다.

생성 명령 예:
```bash
python scripts/generate_detailed_inventory.py
cat DETAILED_INVENTORY.md
```

사용 예:
```bash
cd test_luncher/ybcar/yahboomcar_ros2_ws
python scripts/generate_inventory.py
cat INVENTORY.md
```

`INVENTORY.md`에는 `launch/`, `params/`, `src/`, `scripts/` 아래의 파일 목록 요약이 포함됩니다. 이 파일은 워크스페이스 점검과 통합 테스트 준비에 유용합니다.
디버깅 체크리스트
- 토픽이 안 뜨면: 드라이버 노드가 실행 중인지, 디바이스 권한(`/dev/tty*`) 확인
- TF lookup 실패: `scan.header.frame_id`와 `base_link`/`odom` 이름 일치 여부 확인
- QoS 문제: 레이저는 SensorData QoS 권장 — 퍼블리셔/구독 QoS 일치 확인
- 맵 불안정: 오도메트리 품질(슬립, 엔코더 설정) 혹은 파라미터(파티클 수 등) 조정
- map_saver_cli 설치 여부: `sudo apt install ros-jazzy-nav2-map-server`

파일/검사 우선순위(빠르게 살펴볼 곳)
1. `launch/` 폴더: 어떤 런치들이 포함되어 있는지(파일명, 포함하는 노드) 확인
2. `params/` 폴더: gmapping/ekf/nav2 파라미터
3. `src/` 드라이버 소스: 퍼블리시/구독 토픽명과 QoS
4. `run_*.sh` 스크립트: 환경 소스 방식과 실행명

테스트 명령 모음
```bash
# 빌드
colcon build
source install/setup.bash

# bringup 예
ros2 launch robot_bringup bringup.launch.py

# gmapping 예
ros2 launch yahboomcar_nav map_gmapping_launch.py

# 간단 텔레옵 테스트
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# 맵 저장
ros2 run nav2_map_server map_saver_cli -f /home/ros2/maps/my_map
```

다음 권장 작업(원하시면 제가 진행):
- 워크스페이스 내 `launch/`·`params/`·`src/`를 자동 스캔해 포함된 패키지/런치/파라미터 목록 리포트 생성
- 통합 런치(IncludeLaunchDescription + 인자화) scaffold 생성
- `.gitattributes` 추가로 LF 고정 (Windows CRLF 경고 해결)

---

이 README를 커밋·푸시하겠습니다.