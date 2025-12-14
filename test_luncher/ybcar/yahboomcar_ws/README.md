# yahboomcar_ws

이 워크스페이스는 Yahboom 기반 로봇의 ROS2 패키지들을 모아 둔 상위 워크스페이스입니다.

요약
- 목적: 하드웨어 드라이버, bringup, 제어 및 통합 런치 파일을 포함하는 로컬 개발/테스트용 워크스페이스
- 위치: `test_luncher/ybcar/yahboomcar_ws`

필수사항
- ROS 2 설치 (예: Humble / Rolling 등 사용 중인 배포판)
- colcon, Python 3

빌드
```bash
# 터미널에서
source /opt/ros/<ROS2_DISTRO>/setup.bash
colcon build --merge-install
source install/setup.bash
```

실행 예시
- 개별 패키지 런치(예):
```bash
ros2 launch <package> <launch_file>.py
```
- 통합 런치(예): `test_luncher/ybcar/yahboomcar_ros2_ws/launch/integrated_map_launch.py`를 사용하여 여러 워크스페이스/런치를 결합합니다.

개발 노트
- 패키지별 `launch/`, `params/`, `src/` 구조를 따릅니다.
- 워크스페이스 통합이나 문서 자동화는 `test_luncher/ybcar/yahboomcar_ros2_ws/scripts/`의 인벤토리 스크립트를 참고하세요.

문제 보고
- 변경사항은 로컬에서 테스트 후 커밋하고 PR로 올려주세요.
- Windows에서 작업 시 `.gitattributes`로 LF 강제 설정을 권장합니다.

## 패키지 요약
이 워크스페이스의 패키지별 요약 (유지관리자, 핵심 의존성, 빌드타입):

- `laserscan_to_point_publisher`: Maintainer: parallels <parallels@todo.todo>; Build: `ament_python`; Key deps: (none declared). 용도: `LaserScan` → 포인트 변환 파이썬 패키지.
- `robot_pose_publisher_ros2`: Maintainer: milan <milan.madathiparambil@gmail.com>; Build: `ament_cmake`; Key deps: `rclcpp`, `geometry_msgs`, `tf2_ros`. 용도: odom/센서 기반 pose/tf 발행기.
- `yahboomcar_astra`: Maintainer: nx-ros2 <13377528435@sina.cn>; Build: `ament_python`; Key deps: (none declared). 용도: Orbbec Astra/깊이 카메라 드라이버 통합.
- `yahboomcar_base_node`: Maintainer: nx-ros2 <nx-ros2@todo.todo>; Build: `ament_cmake`; Key deps: `rclcpp`, `geometry_msgs`, `tf2`, `tf2_ros`, `nav_msgs`. 용도: 베이스 하드웨어(모터/인코더) 제어 노드.
- `yahboomcar_bringup`: Maintainer: nx-ros2 <nx-ros2@todo.todo>; Build: `ament_python`; Key deps: `rclcpp`, `std_msgs`. 용도: 하드웨어 초기화 및 통합 런치/설정 제공.
- `yahboomcar_ctrl`: Maintainer: nx-ros2 <nx-ros2@todo.todo>; Build: `ament_python`; Key deps: (none declared). 용도: 텔레옵/로컬 제어 로직(컨트롤러) 패키지.
- `yahboomcar_description`: Maintainer: yahboom <yahboom@todo.todo>; Build: `ament_python`; Key deps: (none declared). 용도: URDF/xacro, 메쉬 등 로봇 모델 리소스.
- `yahboomcar_laser`: Maintainer: yahboom <yahboom@todo.todo>; Build: `ament_python`; Key deps: (none declared). 용도: 레이저 센서 래퍼 및 유틸리티.
- `yahboomcar_mediapipe`: Maintainer: nx-ros2 <13377528435@sina.cn>; Build: `ament_python`; Key deps: (none declared). 용도: Mediapipe 기반 영상처리 통합.
- `yahboomcar_msgs`: Maintainer: root <1461190907@qq.com>; Build: `ament_cmake`; Key deps: `geometry_msgs`, `std_msgs`; Uses `rosidl_default_generators`/`rosidl_default_runtime`. 용도: 커스텀 메시지/서비스 정의 패키지.
- `yahboomcar_multi`: Maintainer: root <root@todo.todo>; Build: `ament_python`; Key deps: (none declared). 용도: 다중 로봇/멀티 인터페이스 헬퍼 모음.
- `yahboomcar_nav`: Maintainer: yahboom <yahboom@todo.todo>; Build: `ament_python`; Key deps: (none declared). 용도: SLAM/Localisation/Navigation 통합(예: gmapping/AMCL/ekf 샘플 포함).
- `yahboomcar_visual`: Maintainer: nx-ros2 <1461190907@qq.com>; Build: `ament_python`; Key deps: (none declared). 용도: RViz 구성 및 시각화 유틸리티.
- `yahboom_app_save_map`: Maintainer: yahboom <yahboom@todo.todo>; Build: `ament_python`; Key deps: `yahboom_web_savmap_interfaces` (exec). 용도: 맵 저장/관리 앱 래퍼.
- `yahboom_web_savmap_interfaces`: Maintainer: yahboom <yahboom@todo.todo>; Build: `ament_cmake`; Key deps: `geometry_msgs`; Uses `rosidl_default_generators`/`rosidl_default_runtime`. 용도: 웹/네트워크 맵 저장 인터페이스 메시지 정의.

원하시면 각 패키지의 `README.md`와 주요 소스(예: `launch/`, `src/` 진입점)를 자동으로 파싱해 더 정확한 한줄 요약(핵심 노드, 실행 예시, 라이선스)을 추가하겠습니다.

## AMCL / EKF (로컬라이제이션 안내)
	1. 하드웨어 드라이버 및 센서(encoder/IMU/LiDAR) 노드 실행
	2. `ekf_node` 실행하여 안정적인 `/odom`을 퍼블리시
	3. `map_server` + `amcl` 실행하여 맵 기반 로컬라이제이션 수행
	4. RViz로 `map->base_link` (또는 `pose`) 확인
```bash
# EKF (예: imu_ws에 있는 샘플 launch를 사용)
ros2 launch imu_ws ekf_launch.py

# map_server + AMCL (예: yahboomcar_nav 내의 런치를 사용)
ros2 launch yahboomcar_nav amcl_launch.py

# 통합된 테스트(통합 런치를 제공하는 경우)
ros2 launch test_luncher.ybcar.yahboomcar_ros2_ws integrated_map_launch.py
```
참고: 위 `ros2 run <pkg> <exe>` 예시는 각 패키지의 `setup.py`의 `entry_points`(console_scripts) 또는 `CMakeLists.txt`의 `add_executable`을 기준으로 자동 생성했습니다. 필요하면 각 스크립트의 인자/환경(예: 카메라 디바이스, 시뮬레이션 플래그)을 README에 추가할 수 있습니다.

### 런타임 인자 / 권장 패턴
아래 패턴은 `ros2 run` / `ros2 launch` 를 실제 환경에 맞춰 실행할 때 자주 사용하는 인자 예시입니다.

- 네임스페이스 지정 (멀티 로봇):
```bash
ros2 run <pkg> <exe> --ros-args -r __ns:=/robot1
```
- 파라미터 설정(예: 시뮬레이션 모드, 디바이스 경로):
```bash
ros2 run <pkg> <exe> --ros-args -p use_sim_time:=true -p device:=/dev/video0
```
- 런치에서 파라미터 파일 지정:
```bash
ros2 launch <pkg> <launch>.py params_file:=/path/to/params.yaml
```

각 콘솔스크립트/노드는 자체 플래그/파라미터를 가질 수 있으니, 패키지별 `README.md` 또는 `--help`(예: `ros2 run <pkg> <exe> --ros-args --help`)로 확인하세요.

- **참고:** 자동으로 수집한 CLI 옵션은 `scripts/CLI_OPTIONS.md`에 정리되어 있습니다. (생성/갱신: `python3 scripts/extract_cli_options.py > scripts/CLI_OPTIONS.md`)

## AMCL 상세 설명 및 EKF 예제
아래는 AMCL과 EKF 설정 항목의 의미와 권장 초기값, 그리고 EKF(예제) YAML 스켈레톤입니다. 이 값들은 하드웨어/주행 환경에 따라 조정해야 합니다.

### AMCL 주요 파라미터 설명 (요약)
- `min_particles` / `max_particles`: 파티클 필터의 최소/최대 입자 수. 실내 소형 로봇의 초기값: 100~2000 (정확도 vs 성능 트레이드오프).
- `update_min_d` / `update_min_a`: 로봇이 이동한 거리/각도 기준으로 AMCL이 업데이트 하는 최소값. 예: `0.25` (m), `0.2` (rad).
- `laser_model_type`: 센서 모델. `likelihood_field` 권장(지형이 복잡하지 않을 때 안정적).
- `z_hit`, `z_rand`, `z_short`, `z_max`: 관측 모델 가중치 — `z_hit` 증가 → 관측에 더 민감, `z_rand` 증가 → 잡음 관대.
- `max_beams` / `max_range`: 레이저 빔 수/최대 범위로 성능/정밀도에 영향.
- `beam_skip_*`: 다중 경로/반사로 인한 잘못된 빔을 건너뛰는 옵션.

권장 워크플로우: 먼저 센서(레이저, IMU, odom)가 신뢰할 수 있게 퍼블리시 되는지 확인하고, EKF로 `odom`을 안정화한 뒤 AMCL을 실행해 보세요.

### EKF (`robot_localization`) 설정 포인트
- `frequency`: EKF 노드 주기(Hz). 보통 30~100Hz.
- `sensor_timeout`: 센서 입력 지연 허용 시간(s).
- `process_noise_covariance` / `measurement_noise_covariance`: 시스템/측정 잡음 공분산. 작은 값 → 신속한 반응, 큰 값 → 더 부드러운 출력.
- `odom0` / `imu0` 등: 입력 토픽 이름(환경에 맞춰 정확히 지정해야 함).

### 예제: 간단한 EKF 파라미터 YAML (스켈레톤)
```yaml
ekf_filter_node:
	ros__parameters:
		frequency: 50.0
		sensor_timeout: 0.1
		two_d_mode: true
		odom0: /odom
		imu0: /imu/data
		odom0_config: [true, true, false,
									 false, false, true,
									 false, false, false]
		imu0_config: [false, false, false,
									true, true, true,
									false, false, false]
		odom0_queue_size: 10
		imu0_queue_size: 50
		process_noise_covariance: [0.05, 0, 0, 0, 0, 0,
															 0, 0.05, 0, 0, 0, 0,
															 0, 0, 0.01, 0, 0, 0,
															 0, 0, 0, 0.01, 0, 0,
															 0, 0, 0, 0, 0.01, 0,
															 0, 0, 0, 0, 0, 0.1]

```

위 스켈레톤은 시작점입니다 — 각 항목(특히 공분산 행렬)은 센서 성능과 로봇 동특성에 맞춰 튜닝해야 합니다. 원하시면 현재 워크스페이스에 있는 파라미터 파일들을 기반으로 권장값을 자동 제안해 드리겠습니다.
- 튜닝 포인트(간단):
	- EKF: 센서별 `process_noise` / `measurement_noise`, `frequency`, 입력 토픽의 `odom0`/`imu0` 등 설정
	- AMCL: `min_particles`/`max_particles`, `update_min_d`, `laser_max_beams`, 센서 모델 가중치
- 팁: EKF로 먼저 `odom`이 안정화되어야 AMCL이 더 빠르게 수렴합니다. 센서 캘리브레이션(특히 IMU/encoder scale, LiDAR mounting)과 TF(링크 프레임) 정의를 먼저 검증하세요.

## 자동 수집(확장): 콘솔스크립트, 파라미터 권장 및 Nav2 검증 체크리스트

**A) 콘솔스크립트 및 인자 요약:**
- **방법:** 워크스페이스의 `setup.py` `entry_points`와 CMake `add_executable`를 스캔하여 `ros2 run <pkg> <exe>` 실행 예시를 생성했습니다. 파이썬 스크립트 내부에 `argparse`/`click`가 없으면 별도 플래그는 없으므로 `--ros-args -p ...` 또는 네임스페이스 재지정 패턴을 사용하세요.
- **발견된 콘솔스크립트(예시):**
	- `yahboomcar_nav`: `stop_car`, `stop_robot1_car`, `stop_robot2_car` — (모듈: `yahboomcar_nav.stop_car` 등). No custom CLI flags detected; 사용법: `ros2 run yahboomcar_nav stop_car` (추가 설정은 `--ros-args -p ...`)
	- `yahboomcar_visual`: `simple_AR`, `laser_to_image`, `pub_image`, `astra_rgb_image`, `astra_depth_image`, `astra_image_flip`, `astra_color_point` — 대부분 이미지/카메라 관련 노드로 내부에서 디바이스 경로 등을 하드코딩하거나 환경변수를 참조합니다. 기본 실행: `ros2 run yahboomcar_visual simple_AR`
	- `yahboom_app_save_map`: `server`, `client` — 서버/클라이언트 앱 엔트리포인트; 기본 실행: `ros2 run yahboom_app_save_map server`
- **권장 실행 패턴:** 네임스페이스/파라미터 적용 예
```bash
ros2 run <pkg> <exe> --ros-args -r __ns:=/robot1 -p use_sim_time:=true -p device:=/dev/video0
ros2 launch <pkg> <launch>.py params_file:=/path/to/params.yaml
```

**B) AMCL / EKF — 현재값 기반 권장 조정(요약):**
- **읽은 파일:**
	- [src/yahboomcar_multi/param/robot1_amcl_params.yaml](src/yahboomcar_multi/param/robot1_amcl_params.yaml) — `min_particles: 500`, `max_particles: 2000`, `max_beams: 60`, `update_min_d: 0.25`, `z_hit: 0.5`, `z_rand: 0.5`, 등.
	- [src/yahboomcar_multi/param/ekf_robot1.yaml](src/yahboomcar_multi/param/ekf_robot1.yaml) — `frequency: 30.0`, `sensor_timeout: 0.1`, `two_d_mode: true`, 상세 `process_noise_covariance` / `initial_estimate_covariance` 포함.
- **권장(초기) 변경 제안:**
	- AMCL
		- `min_particles`: 200~500 (빠른 시험), `max_particles`: 1000~2000 (제한적 환경에서는 낮춰 성능 개선)
		- `max_beams`: 30~60 — 라이다 해상도/연산 여건에 따라 낮추면 CPU 부하 감소
		- `update_min_d`: 0.1~0.25, `update_min_a`: 0.1~0.2 — 이동이 적은 환경이면 낮춰 수렴 가속
		- 관측모델: `z_hit`/`z_rand` 비율로 센서 노이즈 민감도 조정 (실측 잡음이 크면 `z_rand` 증가)
	- EKF (`robot_localization`)
		- `frequency`: 30→40~50Hz (센서/CPU 여유에 따라 높이면 반응성 개선)
		- `sensor_timeout`: 0.1 (현재) 유지 또는 센서 주기에 맞춰 약간 증가
		- `odom0_config`/`imu0_config`: 현재 설정(예: yaw 포함) 확인 — IMU에서 각속도/가속도를 보내면 해당 축 `true`로 설정
		- `process_noise_covariance`: 특정 축(예: yaw, vx)에 대해 조금 증가시켜 회전 시 수렴을 빠르게 할 수 있음
- **권장 워크플로우:**
	1. 모든 센서(`scan`, `imu`, `odom`)가 안정적으로 퍼블리시되는지 확인
	2. EKF를 띄워 `/odometry/filtered` 또는 `/odom`이 안정화되는지 확인 (RViz에서 `odom->base_link` 확인)
	3. AMCL 실행 후 초기 파티클 수/확률 관련 파라미터를 조정하여 빠르게 수렴시키기
	4. 로그(토픽, `/diagnostics`, 로봇 위치 에러)를 기반으로 `process_noise`/`measurement_noise` 재조정

**C) Navigation2(AMCL→Planner) 검증 체크리스트 및 예제 개선 제안:**
- **목표:** Bringup → EKF(odom 안정화) → AMCL(로컬라이즈) → Nav2(경로생성/팔로잉) 순서로 문제를 축소해 검증.
- **검증 체크리스트:**
	- 하드웨어/드라이버: 모든 센서 토픽(`/scan`, `/imu`, `/odom`, 카메라 등) 확인 (`ros2 topic echo`/`ros2 topic hz`)
	- TF 트리: `ros2 run tf2_tools view_frames` 또는 RViz에서 `map->base_link` 존재 확인
	- EKF: `ros2 topic echo /odometry/filtered` 또는 `ros2 topic hz`로 주기 확인; RViz에서 `odom->base_link` 일관성 확인
	- AMCL: 초기화 후 `amcl` 노드가 파티클 수를 줄이며 수렴하는지 확인 (RViz의 particle cloud)
	- Nav2 lifecycle: `nav2_lifecycle_manager`로 각 노드(AMCL, planner, controller, bt_navigator) 활성화 및 상태 확인
	- 간단한 경로 테스트: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: ...}'`로 단거리 목표 수행 확인
	- 로컬 플래너 테스트: 장애물 회피/경로 재계산, recovery behaviour 수행 확인
	- 실패 모드: AMCL 실패 시 `relocalize` 또는 초기 pose 재설정 절차 문서화
- **예제 명령 (요약):**
```bash
# 1) EKF 실행 (imu_ws의 샘플)
ros2 launch imu_ws ekf_launch.py

# 2) map_server + AMCL
ros2 launch yahboomcar_nav map_gmapping_launch.py    # 또는 yahboomcar_multi robot1_amcl_launch.py

# 3) Nav2(라이프사이클 활성화 포함)
ros2 launch yahboomcar_multi navigation_launch.py params_file:=src/yahboomcar_multi/param/robot1_nav_params.yaml

# 4) 간단 경로 테스트(예)
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose '{"pose": {"header": {"frame_id": "map"}, "pose": {"position": {"x": 1.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0}}}}'
```

---
원하시면 제가:
- 각 콘솔스크립트의 파이썬 모듈을 더 깊게 파싱해 사용 가능한 `--help`/`argparse` 옵션을 자동으로 문서화하거나,
- AMCL/EKF 파라미터 파일들의 값을 분석해 YAML로 직접 수정한 추천 프로파일(예: `robot1_amcl_tuned.yaml`, `ekf_tuned.yaml`)을 생성해 드릴 수 있습니다.
어떤 것을 먼저 만들까요? (원하시면 둘 다 생성해 드립니다.)

## 자동 수집: 실행 예시 & 핵심 노드
아래는 워크스페이스 내 각 패키지의 런치 파일과 핵심 노드를 자동으로 스캔하여 만든 간단 실행 예시입니다.

- `laserscan_to_point_publisher`
	- Launch: (없음)
	- 실행 예시: (직접 노드 엔트리 또는 패키지 스크립트 확인 필요)

- `robot_pose_publisher_ros2`
	- Launch: `launch/robot_pose_publisher_launch.py`
	- 실행 예시:
		```bash
		ros2 launch robot_pose_publisher_ros2 robot_pose_publisher_launch.py
		```
	- 핵심 노드: `robot_pose_publisher_ros2/robot_pose_publisher` (파라미터: `use_sim_time`, `is_stamped`, `map_frame`, `base_frame`)

- `yahboomcar_astra`
	- Launch: `launch/colorTracker_X3.launch.py`
	- 실행 예시:
		```bash
		ros2 launch yahboomcar_astra colorTracker_X3.launch.py
		```
	- 핵심 노드: `yahboomcar_astra/colorHSV` (이름: `coloridentify`) — 드라이버는 `yahboomcar_bringup` 런치를 포함합니다.

- `yahboomcar_base_node`
	- Launch: (없음)
	- 실행 예시: 베이스 노드가 패키지 내부 `src/`에 있을 가능성, 개별 노드 실행법은 패키지별 README 참조

- `yahboomcar_bringup`
	- Launch: `launch/yahboomcar_bringup_launch.py`
	- 실행 예시:
		```bash
		ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
		```
	- 핵심 노드 / 포함 런치:
		- 포함: `imu_complementary_filter` 런치, `robot_localization`의 `ekf.launch.py`, `yahboomcar_description`의 description 런치
		- 노드: `tf2_ros/static_transform_publisher` (base_link↔imu_frame 정적 TF)
	- 파라미터: `param/imu_filter_param.yaml` (예: `fixed_frame`, `use_mag`, `publish_tf`)

- `yahboomcar_ctrl`
	- Launch: `launch/yahboomcar_joy_launch.py`
	- 실행 예시:
		```bash
		ros2 launch yahboomcar_ctrl yahboomcar_joy_launch.py
		```
	- 핵심 노드: `yahboomcar_ctrl/yahboom_joy`, 외부 의존: `joy/joy_node`

- `yahboomcar_description`
	- Launch: `launch/description_launch.py`, `description_multi_robot1.launch.py`, `display_launch.py`
	- 실행 예시:
		```bash
		ros2 launch yahboomcar_description description_launch.py
		```
	- 핵심 노드: `robot_state_publisher`, `joint_state_publisher`, `tf2_ros/static_transform_publisher` (robot_description 기반)

- `yahboomcar_laser`
	- Launch: (없음)
	- 실행 예시: 레이저 드라이버 래퍼인 경우 패키지의 노드 엔트리를 확인하세요

- `yahboomcar_mediapipe`
	- Launch: `launch/mediapipe_points.launch.py`
	- 실행 예시:
		```bash
		ros2 launch yahboomcar_mediapipe mediapipe_points.launch.py
		```
	- 핵심 노드: `yahboomcar_point/pub_point`, `rviz2` (RViz config 경로 인자로 사용)

- `yahboomcar_msgs`
	- Launch: (none) — 메시지 정의 패키지
	- 실행 예시: 빌드시 `rosidl` 생성기를 사용하며 런타임에는 메시지 타입을 임포트하여 사용

- `yahboomcar_multi`
	- Launch: 여러 네비게이션/AMCL 관련 런치 포함 (`robot1_amcl_launch.py`, `navigation_launch.py`, 등)
	- 실행 예시:
		```bash
		ros2 launch yahboomcar_multi robot1_amcl_launch.py
		```
	- 핵심 노드: `nav2_amcl/amcl`, `nav2_lifecycle_manager/lifecycle_manager`, `tf2_ros/static_transform_publisher` (네임스페이스 사용 예시 포함)

- `yahboomcar_nav`
	- Launch: `launch/map_gmapping_launch.py`, `launch/navigation_dwb_launch.py`, 등
	- 실행 예시:
		```bash
		ros2 launch yahboomcar_nav map_gmapping_launch.py
		```
	- 핵심 노드: `slam_gmapping` 포함 런치, `tf2_ros/static_transform_publisher` (맵/레이저 TF 정적 설정)

- `yahboomcar_visual`
	- Launch: (none) — RViz/시각화 설정 및 리소스
	- 실행 예시: RViz 구성 파일 경로를 사용하여 `rviz2 -d <config>`

- `yahboom_app_save_map`
	- Launch: (none) — 맵 저장/관리 앱(실행 스크립트 또는 노드 확인 필요)
	- 핵심 의존: `yahboom_web_savmap_interfaces`

- `yahboom_web_savmap_interfaces`
	- Launch: (none) — 메시지/서비스 정의용 인터페이스 패키지

위 내용은 워크스페이스의 `launch/` 디렉터리와 런치 파일 내 `Node(...)`/`IncludeLaunchDescription(...)` 등을 간단히 스캔해 자동 생성했습니다. 원하시면:
- 각 패키지의 `src/` 진입점(노드 실행 스크립트)까지 파싱해 실제 실행 가능한 노드 실행 라인(예: `ros2 run <pkg> <exe>`)을 추가
- 각 런치/파라미터 파일의 주요 파라미터(예: EKF/AMCL 주요 파라미터)를 README에 삽입
중 어떤 작업을 먼저 할지 알려주세요.

## 실행 가능한 노드 예 (`ros2 run`)
자동으로 스캔한 `console_scripts`/빌드 타깃을 바탕으로 핵심 `ros2 run` 예시입니다.

- `laserscan_to_point_publisher` (console_scripts)
	- 실행: `ros2 run laserscan_to_point_publisher laserscan_to_point_publisher`

- `robot_pose_publisher_ros2` (C++ executable)
	- 실행: `ros2 run robot_pose_publisher_ros2 robot_pose_publisher`

- `yahboomcar_astra` (console_scripts)
	- 실행 예(가능한 엔트리포인트 중 일부):
		- `ros2 run yahboomcar_astra colorHSV`
		- `ros2 run yahboomcar_astra colorTracker`

- `yahboomcar_bringup` (console_scripts)
	- 실행 예(유틸/드라이버 엔트리포인트 일부): `ros2 run yahboomcar_bringup Mcnamu_driver_X3`

- `yahboomcar_ctrl` (console_scripts)
	- 실행 예: `ros2 run yahboomcar_ctrl yahboom_joy`

- `yahboomcar_mediapipe` (console_scripts)
	- 실행 예(엔트리포인트 일부): `ros2 run yahboomcar_mediapipe 01_HandDetector`

참고: 위 `ros2 run <pkg> <exe>` 예시는 각 패키지의 `setup.py`의 `entry_points`(console_scripts) 또는 `CMakeLists.txt`의 `add_executable`을 기준으로 자동 생성했습니다. 필요하면 각 스크립트의 인자/환경(예: 카메라 장치, 시뮬레이션 플래그)을 README에 추가할 수 있습니다.

## AMCL / EKF 주요 파라미터 스니펫
아래는 워크스페이스에 포함된 AMCL 파라미터(`yahboomcar_multi/param/robot1_amcl_params.yaml`)와 IMU 필터 예시(`yahboomcar_bringup/param/imu_filter_param.yaml`)의 핵심 항목입니다 — 복사/붙여넣기용으로 그대로 사용 가능합니다.

### AMCL (robot1_amcl 일부)
```yaml
min_particles: 500
max_particles: 2000
max_beams: 60
update_min_d: 0.25
laser_model_type: "likelihood_field"
laser_likelihood_max_dist: 2.0
z_hit: 0.5
z_rand: 0.5
```

### IMU filter(imu_filter_madgwick) 예시
```yaml
imu_filter_madgwick:
	ros__parameters:
		fixed_frame: "base_link"
		use_mag: false
		publish_tf: false
		world_frame: "enu"
		orientation_stddev: 0.0
```

EKF(`robot_localization`)는 이와 별도로 `ekf` 파라미터 파일을 사용합니다(이 리포지토리에는 `robot_localization` 패키지의 런치가 포함되어 있으나 파라미터 파일은 외부 또는 상위 패키지에 위치할 수 있습니다). EKF 설정에서 자주 조정하는 항목:
- `frequency`, 센서별 `process_noise_*`, `measurement_noise_*`, `odom0`/`imu0` 등 입력 토픽 이름

원하시면 위 스니펫을 README 상단의 실행 가이드 근처로 이동시키고, 각 파라미터에 대한 설명(권장 범위, 영향)을 덧붙여 드리겠습니다.

## 생성된 튜닝 프로파일 (자동 생성)
아래 파일들은 이 리포지토리에서 자동 생성한 초기 튜닝 프로파일입니다 — 실제 하드웨어에서 검증 후 조정하세요.

- `src/yahboomcar_multi/param/robot1_amcl_tuned.yaml` — AMCL 초기 튜닝 (권장: `min_particles: 300`, `max_particles: 1500`, `max_beams: 40`, `update_min_d: 0.15`).
- `src/yahboomcar_multi/param/ekf_tuned.yaml` — EKF 초기 튜닝 (권장: `frequency: 40.0`, `sensor_timeout: 0.15`, 조정된 `process_noise_covariance`).
 - `src/yahboomcar_multi/param/robot1_amcl_auto_recommended.yaml` — 자동 분석으로 생성한 AMCL 권장 프로파일(시작점).
 - `src/yahboomcar_multi/param/ekf_auto_recommended.yaml` — 자동 분석으로 생성한 EKF 권장 프로파일(시작점).

사용 예시:
```bash
# AMCL (튜닝 파일 적용)
ros2 launch yahboomcar_multi robot1_amcl_launch.py params_file:=$(pwd)/src/yahboomcar_multi/param/robot1_amcl_tuned.yaml

# EKF (튜닝 파일 적용)
ros2 launch imu_ws ekf_launch.py params_file:=$(pwd)/src/yahboomcar_multi/param/ekf_tuned.yaml
```

## 개발 도구: 자동 수집 및 벤치 스크립트
작업 편의를 위해 아래 스크립트들을 추가했습니다. ROS 2 환경을 소싱한 뒤 로컬에서 실행하세요.

- `scripts/extract_cli_options.py` — 워크스페이스의 파이썬 모듈을 정적 AST 파싱하여 `parser.add_argument`로 선언된 옵션을 추출합니다.
	- 사용: `python3 scripts/extract_cli_options.py > scripts/CLI_OPTIONS.md`

- `scripts/collect_ros2_help.py` / `scripts/collect_ros2_help.ps1` — `setup.py`의 `console_scripts`를 검색하고(발견 시) `ros2 run <pkg> <exe> --ros-args --help` 출력물을 `scripts/ROS2_HELP/` 아래에 저장합니다.
	- 사용 (Unix): `python3 scripts/collect_ros2_help.py --run`
	- 사용 (Windows PowerShell): `.\tools\collect_ros2_help.ps1 -Run`
	- 안전 실행기: `scripts/collect_console_help_runner.py` — 먼저 dry-run으로 실행 계획을 확인하고 `--run`으로 실제 수집합니다. 요구: ROS2 환경을 소싱하거나 `--force`를 사용하여 강제 실행합니다.
	  - 예 (dry-run):

	    ```bash
	    python3 scripts/collect_console_help_runner.py
	    ```

	  - 예 (실행):

	    ```bash
	    python3 scripts/collect_console_help_runner.py --run --out-dir scripts/ROS2_HELP --timeout 8
	    ```

- `scripts/bench_localization.py` — ROS2 `rclpy` 노드로 `/odometry/filtered` 및 `/amcl_pose`를 일정 시간 기록하여 CSV로 저장합니다.
	- 사용: `python3 scripts/bench_localization.py --duration 30 --out logs/loc_log.csv`

이 툴들은 현장/시뮬에서 자동화된 문서화(콘솔 옵션 수집)와 로컬라이제이션 성능 벤치(로그 수집)를 돕기 위해 설계되었습니다.

