(This file consolidates micro-ROS and micro-ROS Agent guidance from the project docs.)

# micro-ROS / micro-ROS Agent (Pi)

요약
- Agent는 ESP32(micro-ROS client)와 ROS2 네트워크를 연결하는 브리지입니다.
- ESP32 ↔ Agent: micro-XRCE-DDS over Serial (예: `/dev/ttyUSB0` @ 921600)
- Agent ↔ ROS2: 표준 DDS(RMW) 사용 — 네트워크 인터페이스와 RMW 설정이 중요합니다.

아키텍처
```
ESP32 (micro-ROS client)
    ↓ serial (/dev/ttyUSB0, 921600)
micro-ROS Agent (Pi)
    ↓ DDS (Cyclone/ Fast-DDS)
ROS2 network (other machines)
```

빠른 시작 (요약)

간단한 실행과 권장 환경 변수는 아래와 같습니다. 전체 설치 및 systemd 예시는 `3_setup.md`를 참고하세요.

```bash
# (선택) micro-ROS Agent가 빌드된 워크스페이스가 있으면 소스
source ~/microros_ws/install/setup.bash

# 권장 환경 변수
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp    # 권장
export RMW_NETWORK_INTERFACE=wlan0

# 수동 실행(디버그용)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v6
```

참고: 자세한 설치, 패키지, systemd 단위 파일 예시는 `3_setup.md`에서 다룹니다.

문제: wlan0에서 Agent가 먼저 기동되면 ESP 세션이 안 만들어지는 현상
- 원인 요약: DDS discovery(멀티캐스트/UDP)와 네트워크 인터페이스 준비 타이밍 문제
  - 무선은 링크 업, DHCP, AP 설정(클라이언트 분리, IGMP 스누핑)으로 멀티캐스트/디스커버리 패킷이 늦게 또는 차단되어 Agent가 올바르게 바인드되지 않을 수 있습니다。
  - 결과적으로 WSL 같은 NAT 환경에서는 `ros2 node list`가 비어 보이거나 discovery가 누락될 수 있습니다。

해결/완화 방법
- Agent 쪽
  - 명시적 인터페이스 바인드:
    ```bash
    export RMW_NETWORK_INTERFACE=wlan0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```
  - systemd에 `After=network-online.target`/`Wants=network-online.target` 추가
  - ExecStartPre로 `/dev/ttyUSB0`와(선택) 로그에서 `Created session` 대기 스크립트 사용

- 네트워크/AP 쪽
  - AP의 client isolation 끄기
  - IGMP 스누핑/멀티캐스트 필터링 설정 확인

- WSL/클라이언트 쪽
  - WSL은 브리지/host 네트워크 구성이 어렵다면 Pi에 SSH로 접속하여 `ros2` 명령 실행 권장
  - Docker/컨테이너 사용 시 `--network=host` 고려

- DDS 레벨 대체 방안
  - Cyclone DDS의 static discovery(정적 엔드포인트) 또는 unicast 설정을 사용하면 멀티캐스트 의존성을 제거할 수 있습니다(설정 복잡도 ↑).

간단한 디버깅 포인터

- Agent 로그: `journalctl -u micro_ros_agent -f`
- 시리얼 포트 확인: `ls -l /dev/ttyUSB0`
- RTPS 캡처(필요 시): `sudo tcpdump -i wlan0 -n udp and portrange 7400-7600 -w /tmp/rtps.pcap`

자동 로그 수집 스크립트(권장): 프로젝트 루트의 `docs/scripts/collect_logs.sh` 또는 `scripts/collect_logs.sh`를 사용해 Agent verbose, tcpdump, 시리얼 덤프를 함께 수집하세요.

추가 자료
- 심층 원인 분석 및 관찰: `4_notes.md`
- 테스트 로그 및 재현: `5_test_log.md`
