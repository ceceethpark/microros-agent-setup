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
이 워크스페이스의 주요 패키지들과 간단한 설명입니다.

- `laserscan_to_point_publisher`: `LaserScan`을 포인트 메시지/포인트클라우드로 변환하여 다른 노드에서 사용하도록 제공합니다.
- `robot_pose_publisher_ros2`: odom/센서 데이터를 기반으로 로봇의 전역/로컬 위치(tf/pose)를 발행합니다.
- `yahboomcar_astra`: Orbbec Astra(또는 호환 깊이 카메라) 드라이버 및 관련 노드 통합 패키지입니다.
- `yahboomcar_base_node`: 모터/엔코더 등 베이스 하드웨어 제어용 로우레벨 노드입니다.
- `yahboomcar_bringup`: 하드웨어 초기화 및 통합 런치 파일과 설정을 포함한 bringup 패키지입니다.
- `yahboomcar_ctrl`: 텔레오퍼레이션, PID 또는 로컬 제어 로직을 담은 제어 관련 패키지입니다.
- `yahboomcar_description`: URDF/xacro, 메쉬와 같은 로봇 모델 및 시각화 리소스입니다.
- `yahboomcar_laser`: 레이저 센서 드라이버 래퍼 및 레이저 관련 유틸리티입니다.
- `yahboomcar_mediapipe`: Mediapipe 기반의 영상 처리/추적 통합 노드(예: 손/얼굴/객체 추적).
- `yahboomcar_msgs`: 이 프로젝트에서 사용하는 커스텀 메시지/서비스 정의입니다.
- `yahboomcar_multi`: 다중 로봇 또는 멀티 인터페이스를 지원하는 헬퍼 노드 모음입니다.
- `yahboomcar_nav`: SLAM/Localisation/Navigation 통합(예: gmapping, AMCL, ekf 샘플/설정) 패키지입니다.
- `yahboomcar_visual`: RViz 구성, 시각화 유틸리티 및 관련 리소스입니다.
- `yahboom_app_save_map`: 맵 저장/관리용 유틸리티 또는 앱 래퍼입니다.
- `yahboom_web_savmap_interfaces`: 웹/네트워크를 통한 맵 저장 인터페이스(REST/웹소켓 등) 연동 패키지입니다.

원하시면 각 패키지의 `README.md` 또는 `package.xml`을 읽어 더 자세한 한줄 요약(저자, 라이선스, 핵심 노드)도 자동으로 추가해 드리겠습니다.
