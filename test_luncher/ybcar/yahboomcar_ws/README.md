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
- 개요: EKF(`robot_localization`의 `ekf_node`)는 IMU/oders/속도(또는 휠 인코더)와 관성/관측을 융합해 안정적인 `odom` 프레임(즉, `odom->base_link`)을 제공합니다. AMCL은 맵과 레이저 스캔을 사용해 로봇의 `pose`(map frame)를 추정하고, 보통 `map->odom` 변환을 브로드캐스트하여 전체 TF 체인을 완성합니다.
- 필요한 토픽: `/scan` (LaserScan), `/odom` (오도메트리), `/imu/data` (IMU), 그리고 TF(`/tf`, `/tf_static`).
- 권장 실행 순서:
	1. 하드웨어 드라이버 및 센서(encoder/IMU/LiDAR) 노드 실행
	2. `ekf_node` 실행하여 안정적인 `/odom`을 퍼블리시
	3. `map_server` + `amcl` 실행하여 맵 기반 로컬라이제이션 수행
	4. RViz로 `map->base_link` (또는 `pose`) 확인
- 간단 실행 예:
```bash
# EKF (예: imu_ws에 있는 샘플 launch를 사용)
ros2 launch imu_ws ekf_launch.py

# map_server + AMCL (예: yahboomcar_nav 내의 런치를 사용)
ros2 launch yahboomcar_nav amcl_launch.py

# 통합된 테스트(통합 런치를 제공하는 경우)
ros2 launch test_luncher.ybcar.yahboomcar_ros2_ws integrated_map_launch.py
```
- 튜닝 포인트(간단):
	- EKF: 센서별 `process_noise` / `measurement_noise`, `frequency`, 입력 토픽의 `odom0`/`imu0` 등 설정
	- AMCL: `min_particles`/`max_particles`, `update_min_d`, `laser_max_beams`, 센서 모델 가중치
- 팁: EKF로 먼저 `odom`이 안정화되어야 AMCL이 더 빠르게 수렴합니다. 센서 캘리브레이션(특히 IMU/encoder scale, LiDAR mounting)과 TF(링크 프레임) 정의를 먼저 검증하세요.

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

