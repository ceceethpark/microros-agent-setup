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

### 실행 스크립트
저장소 루트에 있는 `run_gmapping.sh`를 사용하면 환경 소싱 후 런치를 실행합니다:

```bash
./run_gmapping.sh
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

## 주행 패턴 및 권장 파라미터
- 권장 주행 패턴: 천천히 직선 주행(0.1–0.3 m/s)과 완만한 회전(0.2–0.5 rad/s)을 섞어 영역을 스트라이프(zig-zag) 형태로 커버하세요. 급가속/급회전은 피하고, 같은 지역을 여러 각도에서 스캔하도록 천천히 통과시키는 것이 좋습니다.
- 권장 파라미터(기본값에서 적용한 권장치):
	- `particles`: 60 (안정성을 위해 증가)
	- `linearUpdate`: 0.5 m (맵 업데이트 민감도 증가)
	- `angularUpdate`: 0.2 rad (~11°)
	- `map_update_interval`: 3.0 s (맵 갱신 주기 단축)
	- `temporalUpdate`: 0.5 s
	- `maxUrange`: 5.0 m

- 튜닝 팁:
	- 맵이 떨리거나 불안정하면 `particles`를 늘리고 `linearUpdate`/`angularUpdate`를 줄여 보세요.
	- 맵이 너무 느리게 업데이트되면 `map_update_interval`을 줄이거나 `linearUpdate`를 늘려 보세요.
	- 오도메트리 신뢰도가 낮으면 센서 기반(레이저) 매칭 의존도가 증가하므로 `particles` 증가가 유리합니다.

예시: 파라미터 변경 후 즉시 테스트하려면 `slam_gmapping.yaml`을 편집한 뒤 워크스페이스를 다시 빌드하거나 `source install/setup.bash` 후 런치를 재시작하세요.

## 수동 주행(텔레옵) 및 맵 저장 도구

- 텔레옵(키보드)을 이용해 수동 주행하려면 별도 터미널에서 아래를 실행하세요:

```bash
# 터미널 A: 맵퍼 실행
./run_gmapping.sh

# 터미널 B: 텔레옵 실행 (현재 터미널에서 키보드로 조종)
./run_teleop.sh
```

- 맵을 파일로 저장하려면 `nav2_map_server`의 `map_saver_cli`가 필요합니다. 설치되어 있으면 아래처럼 실행하세요:

```bash
# 저장 예시: 저장할 경로와 파일명(확장자 제외)을 지정
./map_saver.sh /home/ros2/maps/my_map
# 결과: /home/ros2/maps/my_map.pgm and /home/ros2/maps/my_map.yaml
```

참고: `map_saver_cli`는 `nav2_map_server` 패키지에 포함되어 있습니다. 설치되어 있지 않으면 `sudo apt install ros-jazzy-nav2-map-server` 또는 소스 빌드를 통해 설치하세요.

## 자주 묻는 질문 (Q&A 요약)

- Q: 이 예제는 차량이 스스로 주행하나요, 사용자가 조작하나요?
	- A: 둘 다 가능합니다. 현재 저장소는 맵 생성 노드(`slam_gmapping`)만 포함하므로, 수동 주행(리모컨/키보드/조이스틱)으로 맵을 만들 수 있습니다. 자동 탐색을 원하면 Navigation2와 탐색 노드를 추가해야 합니다.

- Q: 맵은 어디에 저장되나요? 자동 저장되나요?
	- A: 기본 소스는 맵을 생성하여 `/map`으로 발행만 합니다. 파일로 자동 저장(예: `my_map.pgm`/`my_map.yaml`)이나 `map_server`로 서빙하는 기능은 포함되어 있지 않습니다. `map_saver_cli` 또는 간단한 스크립트로 저장할 수 있습니다(예시 명령은 README에 기재됨).

- Q: gmapping이 맵을 만드는 과정은 어떻게 되나요?
	- A: 레이저 스캔과 오도메트리(TF)를 받아 스캔매칭과 파티클 필터(GridSlamProcessor)를 사용해 2D 점유격자 맵을 생성합니다. 차량이 움직이며 다양한 위치에서 스캔을 모아 맵을 확장합니다.

- Q: 이 레포지토리에서 포함된 것 / 포함되지 않은 것은?
	- A: 포함: `slam_gmapping` 래퍼(맵 생성 및 `/map` 발행), `openslam_gmapping` 라이브러리, `run_gmapping.sh`, 관련 런치/파라미터. 미포함: `map_saver` 자동 저장, `map_server` 런치, Navigation2(AMCL/planner/explore), 텔레옵 패키지(외부에서 설치 가능).

- Q: 수동 주행으로 맵을 만들려면 어떻게 하나요?
	- A: `./run_gmapping.sh`로 맵퍼 실행 후, 별도 터미널에서 `teleop_twist_keyboard`(또는 조이스틱 드라이버)를 실행해 `cmd_vel`을 전송하면 됩니다. 로봇 쪽 드라이버가 `cmd_vel`을 받아 모터를 제어하고 `odom`/TF를 발행해야 합니다.

- Q: 테스트 시 권장 주행 패턴과 기본 파라미터는?
	- A: README의 "주행 패턴 및 권장 파라미터" 섹션을 따르세요(예: 0.1–0.3 m/s, 회전 0.2–0.5 rad/s, `particles=60`, `linearUpdate=0.5`, 등).

원하시면 위 FAQ 항목을 더 늘리거나, 자동 저장 스크립트 및 텔레옵 런치 파일을 바로 추가하겠습니다.