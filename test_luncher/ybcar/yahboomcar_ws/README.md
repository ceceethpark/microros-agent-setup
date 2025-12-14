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
