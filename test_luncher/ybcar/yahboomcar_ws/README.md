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
