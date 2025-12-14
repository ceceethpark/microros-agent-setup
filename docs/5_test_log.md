# ROS2 테스트 로그 (요약)

이 문서는 날짜별 핵심 테스트 결과와 재현 가능한 절차를 간단히 기록합니다. 상세 분석 및 원인은 `4_notes.md`를 참고하세요.

## 2025-12-12 — 멀티 머신 통신 및 DDS 전환

- 환경: WSL(개발), Raspberry Pi (Agent), Linux PC (테스트)
- 초기 상태: 모든 머신 Fast-DDS(`rmw_fastrtps_cpp`), `ROS_DOMAIN_ID=20`
- 문제: WSL에서 `ros2 node list` 실행 시 "sequence size exceeds remaining buffer" 오류 발생 → 노드 discovery 실패(토픽 통신은 정상)
- 조치: 모든 머신을 Cyclone DDS(`rmw_cyclonedds_cpp`)로 변경
- 결과: `ros2 node list` 정상화, 멀티 머신 discovery 정상화

## 2025-12-13 — ESP32 / Agent 통신 확인

- Pi에서 `/YB_Car_Node` 확인 성공
- ESP32 ↔ Agent (serial) 연결 테스트: 시리얼 포트 `/dev/ttyUSB0` 사용, 921600 baud
- 관찰: wlan0 환경에서 Agent 기동 시 타이밍 문제로 ESP가 먼저 부팅되면 노드가 보이지 않는 현상 발견(네트워크/디스커버리 타이밍 관련)

## 주요 확인 명령(재현/분석용)

```bash
# 인터페이스 상태
ip addr show wlan0

# Agent 로그
journalctl -u micro_ros_agent -f

# RTPS 패킷 캡처
sudo tcpdump -i wlan0 -n udp and portrange 7400-7600 -w /tmp/rtps.pcap

# Agent 상세 실행(verbose)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RMW_NETWORK_INTERFACE=wlan0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v9 |& tee /tmp/agent_verbose.log
```

더 자세한 로그와 분석 결과는 본 파일의 개별 날짜 항목과 `4_notes.md`의 원인 분석 섹션을 확인하세요.
