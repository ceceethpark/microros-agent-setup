# IMU 테스트 가이드

이 README는 `imu_ws`(IMU 드라이버/브로드캐스터)를 빠르게 빌드하고 테스트하기 위한 최소 절차입니다.

## 전제
- ROS2 Jazzy가 설치되어 있고 `source /opt/ros/jazzy/setup.bash`가 가능한 환경.
- IMU(예: MPU9250, BMI088 등)가 드라이버 또는 브로드캐스터 패키지를 통해 `sensor_msgs/Imu` 메시지를 퍼블리시함.
- 필요한 경우 `imu_link` → `base_link` TF 관계가 설정되어 있거나, `robot_state_publisher`로 브로드캐스트할 수 있어야 함.

## 빌드
워크스페이스로 이동 후 빌드:

```bash
source /opt/ros/jazzy/setup.bash
cd test_luncher/ybcar/imu_ws
colcon build --packages-select <imu_driver_package>
source install/setup.bash
```

`<imu_driver_package>`는 사용 중인 IMU 드라이버 패키지명으로 교체하세요.

## 실행 (간단)
- 드라이버 단독 실행 예시(패키지/런치 파일 이름은 환경에 맞게 변경):

```bash
ros2 launch imu_driver imu_bringup.launch.py
```

- 제공된 실행 스크립트 사용:

```bash
./run_imu.sh
```

## 실행 확인
- IMU 토픽 확인:

```bash
ros2 topic echo /imu
ros2 topic hz /imu
ros2 topic list
```

- 메시지 타입/내용 확인:

```bash
ros2 topic echo /imu --once
```

- TF 확인: RViz에서 `imu_link`, `base_link`를 확인하거나 `ros2 run tf2_tools view_frames` 사용.

## 권장 설정 및 통합
- IMU 노이즈/편향 보정: 드라이버에서 캘리브레이션 기능이 있으면 먼저 설정하세요.
- 상태 추정(옵션): IMU를 오도메트리 보정이나 로봇 상태 추정에 사용하려면 `robot_localization`(`ekf_node`/`ukf_node`) 또는 `imu_tools` 계열을 사용하는 것을 권장합니다.
- 토픽 QoS: IMU 퍼블리셔의 QoS(예: `sensor_data`)와 구독 QoS를 맞추세요.

## 권장 테스트 절차
1. IMU를 고정하고 `/imu`가 안정적으로 퍼블리시되는지 확인.
2. 로봇에 장착 후 천천히 회전/기울임을 주어 센서 값(각속도, 가속도) 변화를 확인.
3. 상태 추정 노드(`robot_localization`)와 함께 동작시키며 odom/pose 추정의 안정성 검사.

## 자주 발생하는 문제
- 토픽이 안 뜸 → 드라이버가 실행 중인지, 디바이스 권한(`/dev/ttyUSB*` 등) 확인
- 데이터가 잡음/편향 심함 → 캘리브레이션 절차 확인
- TF 불일치 → `imu_link`와 `base_link` 관계 점검

## 예시 파라미터 노트
- 샘플 레이트: 100–400 Hz (IMU 모델에 따라 다름)
- 축 정렬: 드라이버에서 `frame_id`/축 정렬 파라미터 제공 시 실제 하드웨어에 맞춰 설정
- 필터/저역통과(옵션): 고주파 노이즈 제거용으로 구현된 필터 사용 권장

## 텔레옵/통합 팁
- IMU는 자체적으로 맵을 만들지는 않지만, SLAM 또는 상태 추정의 입력으로 매우 유용합니다.
- `gmapping_ws`와 결합하려면 IMU→`robot_localization`→`odom` 형태로 통합하여 더 안정적인 추정 결과를 얻을 수 있습니다.

## EKF(`robot_localization`) 통합 샘플
아래 샘플은 `imu_ws`에 간단한 `ekf_node` 런치와 파라미터 예시를 추가한 것입니다. 이 샘플은 IMU(`/imu`)와 오도메트리(`/odom`)를 받아 `odom`/`tf`를 보정합니다.

추가된 파일:
- `launch/ekf_launch.py` : `ekf_node`를 실행하는 런치 파일
- `params/ekf_params.yaml` : `ekf_node` 파라미터 예시

사용 방법(워크스페이스를 빌드/소스한 후):

```bash
# 1) 빌드 및 소스 (이미 빌드했다면 스킵)
cd test_luncher/ybcar/imu_ws
colcon build
source install/setup.bash

# 2) ekf 실행
ros2 launch imu_ws launch/ekf_launch.py
```

설정 요약/주의사항:
- `params/ekf_params.yaml`의 `odom0`과 `imu0` 토픽명을 사용 중인 시스템에 맞춰 변경하세요.
- `odom0_config`와 `imu0_config` 배열은 측정되는 상태(예: x,y,yaw 등)를 활성화합니다. 예시는 2D 주행(평면) 기반으로 설정되어 있습니다.
- `two_d_mode: true`로 설정하면 `z`/롤/피치 관련 항목이 무시되어 안정화됩니다.
- 실제 시스템에서 EKF 동작을 확인하려면 RViz와 `ros2 topic echo /odometry/filtered` 등을 사용하세요.

## 관련 리소스
- IMU 드라이버 예시: https://index.ros.org/packages/?q=imu
- `robot_localization` 패키지: https://github.com/cra-ros-pkg/robot_localization

---

원하시면 `imu_ws`에 실제 드라이버 예제(launch/params)나 `ekf` 통합 샘플을 추가로 scaffold 해 드리겠습니다.