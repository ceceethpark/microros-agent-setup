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

## 기능 및 용도
- GMapping(`slam_gmapping`)은 레이저 스캔(`sensor_msgs/LaserScan`)과 오도메트리/TF를 이용해 2차원 점유격자(occupancy grid) 맵을 생성합니다.
- 주 용도: 실내/좁은 공간에서의 지도 작성(맵 생성), SLAM 기반의 실시간 위치 추정, 탐색 및 경로 계획을 위한 맵 획득.
- 요구사항: 안정적인 `scan` 토픽, `odom` 프레임/오도메트리, 그리고 `scan.header.frame_id`와 베이스 프레임(`base_link` 또는 `base_footprint`) 간 TF가 필요합니다.
- 제한사항: 센서 잡음, 잘못된 TF 또는 부정확한 오도메트리는 맵 품질 저하를 초래합니다. 파라미터(`particles`, `maxRange`, `map bounds`) 튜닝이 중요합니다.

## 관련 영상 및 참고 자료
- 공식 문서 (권장): https://wiki.ros.org/gmapping
- YouTube 검색: `ROS gmapping tutorial` → https://www.youtube.com/results?search_query=ros+gmapping+tutorial
- YouTube 검색(ROS2 관련): `ROS2 gmapping tutorial` → https://www.youtube.com/results?search_query=ros2+gmapping+tutorial

위 영상 검색 결과에서 실습형 튜토리얼(예: The Construct, ROS Tutorial 채널 등)을 선택하시면 실제 실행/파라미터 튜닝 사례를 확인하실 수 있습니다.

## Yahboom 관련 링크
- MicroROS‑Pi5 제품 페이지: https://www.yahboom.net/study/MicroROS-Pi5
- Yahboom 공식 YouTube 채널: https://www.youtube.com/channel/UCaishn63yF9Q_jKWEiaacFw
- 매뉴얼(구글 드라이브): https://drive.google.com/drive/folders/1Xd2boZ8CG8Hq_G6zRj8CqsNoueV-Ji3G
- 매핑 앱 (구글 드라이브): https://drive.google.com/drive/folders/1f25ErWgtyv9w0pSxEO0E7r575RcmiKlX
- 소스 코드 모음 (구글 드라이브): https://drive.google.com/drive/folders/16n1kfoHGunD2CuZ_Je-8tCHgshOiDSGN