# GMapping 테스트 가이드

이 README는 `gmapping_ws`(패키지: `slam_gmapping`, 라이브러리: `openslam_gmapping`)를 빠르게 빌드하고 테스트하기 위한 최소 절차입니다.

## 전제
- ROS2 Jazzy가 설치되어 있고 `source /opt/ros/jazzy/setup.bash`가 가능한 환경.
- 라이다 드라이버가 `scan` 토픽(SensorData QoS)으로 레이저 메시지를 퍼블리시함.
- 로봇 TF 트리에서 `scan.header.frame_id` → `base_link`(또는 `base_footprint`) → `odom`이 연결되어 있어야 함.

## 빌드
작업 디렉터리로 이동 후 빌드:

```bash
source /opt/ros/jazzy/setup.bash
cd test_luncher/ybcar/gmapping_ws
colcon build --packages-select slam_gmapping openslam_gmapping
source install/setup.bash
```

## 실행 (간단)
- 단독으로 `slam_gmapping` 실행:

```bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

- 프로젝트 제공 통합 런치 (static TF 포함):

```bash
# yahboom 워크스페이스가 소스된 상태에서 실행하거나 해당 워크스페이스의 설치본을 소스하세요.
ros2 launch yahboomcar_nav map_gmapping_launch.py
```

## 실행 확인
- `scan` 토픽 확인:

```bash
ros2 topic echo /scan
```

- `map` 데이터 확인:

```bash
ros2 topic echo /map --once
ros2 topic list
```

- RViz2: `/map`(Map), `/scan`(LaserScan), TF(Frames)를 올리고 `map`, `odom`, `base_link`, `laser_frame`을 확인.

## 주요 설정 및 권장 변경
- `src/slam_gmapping/params/slam_gmapping.yaml`의 `base_frame` 값을 실제 TF에 맞게 (`base_link` 권장) 변경하세요.
- `slam_gmapping`은 `scan` 메시지의 `header.frame_id`가 TF 트리에서 해결되어야 초기화와 pose 계산이 정상 동작합니다.

## 자주 발생하는 문제
- TF lookup 실패 → 프레임 이름 불일치(예: `base_footprint` vs `base_link`) 확인
- Odometry 부재 → `odom` 프레임/퍼블리셔 확인
- 토픽 QoS 불일치 → 레이저 퍼블리셔가 SensorData QoS 또는 호환되는 QoS 사용
- 맵이 비어 있음 → 로그에서 `Failed to compute odom pose` 또는 `Unable to determine orientation of laser` 확인

## 추가 요청 가능 작업
- `slam_gmapping.yaml`의 `base_frame`을 `base_link`로 패치하여 테스트 환경에 맞추기
- 간단한 실행 스크립트(`run_gmapping.sh`) 또는 테스트 체크리스트 추가

문서를 생성하고 커밋·푸시하겠습니다. 문제가 있으면 알려주세요.