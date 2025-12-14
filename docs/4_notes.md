# 개발 노트

이 파일은 분석과 관찰, 트러블슈팅 요약을 담는 문서입니다. 설정 관련 지침은 `3_setup.md`에, 테스트 로그는 `5_test_log.md`에 정리되어 있습니다.

ROS2 개발 및 테스트 중 발견한 중요 사항들을 기록합니다.

## 주요 개념

### ROS2 기본 구조
- **Node**: ROS2의 기본 실행 단위
- **Topic**: 노드 간 비동기 통신
- **Service**: 요청-응답 방식의 동기 통신
- **Action**: 장기 실행 작업을 위한 통신

### 주요 명령어

```bash
# Linux 버전 확인
lsb_release -a          # 상세 정보
cat /etc/os-release     # 배포판 정보
uname -a                # 커널 정보

# ROS2 버전 확인
printenv | grep ROS     # ROS 관련 환경 변수
echo $ROS_DISTRO        # ROS 배포판 이름 (humble, jazzy 등)
# 주의: ros2 --version은 지원되지 않음, $ROS_DISTRO 사용

# 노드 실행
ros2 run <package_name> <executable_name>

# 토픽 리스트 확인
ros2 topic list

# 토픽 정보 확인
ros2 topic info <topic_name>

# 노드 정보 확인
ros2 node list
ros2 node info <node_name>

# 멀티 머신 통신 확인
ros2 topic list  # 다른 머신의 토픽이 보이는지 확인
ros2 topic echo <topic_name>  # 다른 머신의 토픽 데이터 수신 확인

# DDS 설정 확인
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
ros2 doctor --report
```

## 문제 해결

### "sequence size exceeds remaining buffer" 에러
Fast-DDS의 버퍼 크기 제한으로 인한 문제

**증상:**
- `ros2 node list` 실행 시 에러 발생
- 멀티 머신 통신 시 노드 정보 교환 실패

**해결 방법:**
1. **Cyclone DDS로 변경 (가장 간단)**
   - 모든 머신에서 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
   
2. **Fast-DDS XML 설정으로 버퍼 크기 증가**
  - [3_setup.md](3_setup.md)의 Fast-DDS XML 설정 참고

3. **네트워크 최적화**
   - MTU 크기 확인: `ip link show`
   - 방화벽 규칙 확인

**참고:** 모든 머신이 동일한 DDS 미들웨어를 사용해야 합니다.

### WSL2 네트워크 제한사항

**문제:**
- WSL2에서 `ros2 node list` 작동 안 함
- 토픽 통신(`ros2 topic list`, `ros2 topic echo`)은 정상 작동

**원인:**
- WSL2의 NAT 네트워크 구조
- Multicast 기반 노드 discovery 제한
- Cyclone DDS 설정으로도 해결 안 됨

**실용적 해결 방법:**

1. **Pi 또는 Linux PC를 메인으로 사용**
   ```bash
   # WSL에서 SSH로 Pi에 접속하여 명령 실행 (권장)
   ssh ros2@192.168.0.103
   ros2 node list
   ros2 node info /YB_Car_Node
   ```

2. **WSL에서는 토픽 통신만 사용(제약 설명 및 원인/해결)**
   ```bash
   # WSL에서 가능한 것
   ros2 topic list          ✓ 작동
   ros2 topic echo /battery ✓ 작동
   ros2 topic pub /cmd_vel  ✓ 작동
   
   # WSL에서 안 되는 것 (관찰)
   ros2 node list           ✗ 안됨
   ros2 node info           ✗ 안됨
   ```

   상세 설명 — 왜 차이가 나는가 (루프백/디스커버리 관련)
   - 요지: Pi에서 Agent를 `wlan0`으로만 바인딩한 환경에서는 DDS의 discovery 패킷(일반적으로 RTPS 멀티캐스트)이 WSL의 네트워크 네임스페이스(NAT) 또는 AP의 무선 동작에 의해 차단/누락될 수 있습니다. 그 결과 WSL에서는 노드 디스커버리 정보가 제대로 수신되지 않아 `ros2 node list`가 비어 보일 수 있습니다.
   - '루프백 문제'의 의미: DDS가 시작 시 올바른 물리 인터페이스로 바인드하지 못하거나(예: 인터페이스 미준비), WSL/호스트 네트워크 경로가 멀티캐스트를 전달하지 못하면 DDS가 로컬 루프백(127.0.0.1)이나 다른 인터페이스에 바인드되어 외부 참가자와의 discovery가 이루어지지 않습니다. 이 상태에서 토픽의 일부 메시지(특정 QoS/경로를 통해 전송된 것)는 보이더라도 노드 메타데이터(discovery)가 누락될 수 있습니다.

   기술적 포인트
   - DDS Discovery는 멀티캐스트(또는 UDP 기반의 메타트래픽)를 사용하여 참가자 정보를 교환합니다. 무선 네트워크에서 멀티캐스트가 차단되거나 IGMP/스위치 설정으로 필터링되면 discovery가 실패합니다.
   - WSL2는 NAT된 네트워크 네임스페이스를 사용하므로 멀티캐스트 패킷이 전달되지 않거나 변형되어 discovery가 정상 동작하지 않을 수 있습니다.
   - 일부 DDS 구현체(또는 설정)는 데이터 전송(토픽 메시지)을 위한 경로를 설정할 수 있지만, 툴링(예: `ros2 node list`)이 기대하는 discovery 정보는 누락될 수 있습니다. 때문에 "토픽은 보이나 노드는 보이지 않는" 현상이 발생할 수 있습니다.

   권장 수정/완화 방법
   - Pi 쪽 (Agent)
     - `RMW_NETWORK_INTERFACE` 환경변수로 사용 인터페이스를 명시하세요.
       ```bash
       export RMW_NETWORK_INTERFACE=wlan0
       export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
       ```
     - Cyclone DDS를 사용한다면 `CYCLONEDDS_URI`로 바인드 주소를 명시할 수 있습니다:
       ```bash
       export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.0.88</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
       ```
     - systemd 유닛에 `After=network-online.target` 및 `Wants=network-online.target`를 추가하여 네트워크 준비를 보장하세요.

   - AP/무선 환경
     - AP의 client isolation(클라이언트 분리) 기능을 비활성화하세요.
     - IGMP 스누핑/멀티캐스트 필터링 설정을 확인하고, 가능하면 멀티캐스트 전달을 허용하세요.

   - WSL 측 대안
     - WSL에서 직접 discovery가 필요하면 WSL을 브리지 네트워크로 구성하거나(복잡), 대신 Pi에 SSH로 접속해 `ros2` 명령을 실행하세요(권장).
     - 또는 WSL에서 Docker를 쓰는 경우 `--network=host` 또는 호스트 네트워크 환경을 이용해 멀티캐스트 전달을 확보할 수 있습니다(WSL 호스트 네트워크와의 제약에 따라 다름).

   - DDS 레벨 대체 방안
     - Cyclone DDS의 static discovery(정적 엔드포인트) 또는 unicast 설정을 사용하여 discovery를 고정(정적)으로 구성하면 멀티캐스트 의존성을 제거할 수 있습니다(설정 복잡도 ↑).

   짧은 결론: WSL에서 '토픽은 보이나 노드 목록은 보이지 않는' 현상은 멀티캐스트 기반 discovery 패킷이 전달되지 않거나 Agent가 올바른 인터페이스로 바인드되지 않았기 때문에 발생합니다. 우회 방안은 `RMW_NETWORK_INTERFACE`/`CYCLONEDDS_URI`로 바인딩을 고정하거나, Pi에서 직접 명령을 실행하거나(SSH), AP/네트워크 설정을 조정하는 것입니다.

3. **개발 환경 권장 구성**
   - **WSL**: 코드 개발, 토픽 모니터링
   - **Pi/Linux PC**: 노드 실행 및 관리
   - **VS Code Remote SSH**: Pi에 직접 연결하여 개발


## 유용한 팁

### Cyclone DDS vs Fast-DDS 비교

**Cyclone DDS의 장점:**
1. **WSL 호환성**: WSL 환경에서 버퍼 크기 문제 없음
2. **간단한 설정**: 추가 XML 설정 파일 불필요
3. **메모리 효율**: 더 적은 메모리 사용
4. **안정성**: 네트워크 환경 변화에 더 견고함
5. **멀티 머신 통신**: Discovery 프로토콜이 더 안정적

**Fast-DDS의 장점:**
1. **ROS2 기본값**: 대부분의 ROS2 배포판 기본 DDS
2. **성능**: 높은 처리량이 필요한 경우 더 빠를 수 있음
3. **QoS 옵션**: 더 세밀한 QoS(Quality of Service) 설정 가능

**현재 프로젝트 상황:**
- WSL에서 Fast-DDS 버퍼 문제 발생
- 토픽 통신은 정상이지만 노드 discovery 실패
- **권장**: 모든 머신을 Cyclone DDS로 통일

**변경 방법:**
```bash
# 설치
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# 설정
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### micro-ROS와 DDS 미들웨어 호환성

**중요: Cyclone DDS로 변경해도 ESP32 micro-ROS와 호환됩니다!**

**통신 구조:**
```
ESP32 (micro-ROS) <--[micro-XRCE-DDS]--> micro-ROS Agent <--[DDS]--> ROS2 네트워크
```

**설명:**
1. **ESP32 ↔ Agent**: micro-XRCE-DDS 프로토콜 사용
   - DDS 미들웨어 선택과 **무관**
   - Serial(USB) 또는 UDP로 통신

2. **Agent ↔ ROS2**: DDS 미들웨어 사용
   - Agent는 일반 ROS2 노드처럼 작동
   - Agent가 사용하는 DDS만 변경하면 됨
   - **Cyclone DDS로 변경 가능** ✓

**결론:**
- Pi의 micro-ROS Agent만 Cyclone DDS로 실행하면 됨
- ESP32 코드는 변경 불필요
- 모든 머신(WSL, Pi, Linux PC)을 Cyclone DDS로 통일 가능

### micro-ROS Agent 통신 구조 정리

**통신 경로:**
```
ESP32 (micro-ROS Client)
    ↓
    Serial USB (/dev/ttyUSB0, 921600 baud)
    ↓
micro-ROS Agent (Pi)
    ↓
wlan0 (192.168.0.88) + Cyclone DDS
    ↓
ROS2 네트워크 (154 PC, WSL 등)
```

**핵심 포인트:**
1. **ESP32 ↔ Agent**: Serial USB 통신
   - 연결: `/dev/ttyUSB0` @ 921600 baud
   - 프로토콜: micro-XRCE-DDS
   - wlan0와 **완전히 독립적**
   
2. **Agent ↔ ROS2 네트워크**: wlan0 네트워크 통신
   - 인터페이스: wlan0 (192.168.0.88)
   - DDS: Cyclone DDS (rmw_cyclonedds_cpp)
   - Domain: ROS_DOMAIN_ID=20

3. **노드 생성 위치**: `/YB_Car_Node`
   - ESP32 micro-ROS에서 생성
   - Agent가 ROS2 네트워크로 브리지
   - 실제 실행 위치: ESP32 펌웨어

4. **Discovery 동작**:
   - Pi 로컬: `ros2 node list`가 안 보일 수 있음 (wlan0만 사용 설정)
   - 네트워크: 154 PC 등 다른 머신에서 `/YB_Car_Node` 정상 표시
   - **중요**: 다른 머신에서 보이면 정상 작동 ✅

**systemd 서비스 설정:**
```bash
# /etc/systemd/system/micro_ros_agent.service
ExecStart=ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v6
Environment="ROS_DOMAIN_ID=20"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
Environment="RMW_NETWORK_INTERFACE=wlan0"
```

# 연결 확인:
```bash
# Pi에서 확인
ls -l /dev/ttyUSB0                          # USB 연결 확인
journalctl -u micro_ros_agent -n 50         # 로그 확인 (session created 메시지)
ros2 topic list                             # 토픽 확인

# 154 PC에서 확인 (최종 검증)
ros2 node list                              # /YB_Car_Node 표시되어야 함
ros2 topic echo /battery                    # 데이터 수신 확인
```

## 관찰 요약 — wlan0 vs eth0 차이 관련
- **증상:** Pi에서 `wlan0`을 사용할 때 Agent가 먼저 기동하면 ESP32가 이미 부팅된 상태에서는 `/YB_Car_Node`가 생성되지 않음. 반면 `eth0`(유선)에서는 동일 상황에서 문제가 재현되지 않음.
- **원인(요약):** DDS(Discovery)와 네트워크 인터페이스 준비 타이밍/바인딩 차이. 무선(wlan0)은 링크 업, DHCP, 멀티캐스트 가용성, AP 설정(클라이언트 분리 등) 때문에 통신 가능 상태가 늦거나 불안정해 Agent가 올바르게 바인드되지 않거나 디스커버리 패킷이 누락되어 ESP↔Agent 핸드셰이크가 실패할 가능성이 높음. 유선(eth0)은 링크와 IP 준비가 더 빠르고 안정적이라 문제가 발생하지 않음.
- **검증 포인트:**
  - `ip addr show wlan0` / `ip route` 로 인터페이스/IP 상태 확인
  - `ss -uap | grep micro_ros_agent` 로 Agent가 바인드한 주소·포트 확인
  - `sudo tcpdump -i wlan0 udp and portrange 7400-7600` 로 RTPS/디스커버리 패킷 캡처
  - Agent를 수동으로 `-v9` 로 실행해 `Created session` 로그 타이밍 관찰
- **권장 완화:**
  - systemd 유닛에 `Wants=network-online.target` 및 `After=network-online.target` 추가하고 `network-online`을 보장
  - `RMW_NETWORK_INTERFACE=wlan0` 또는 CycloneDDS URI로 바인딩 명시
  - 이미 추가한 `ExecStartPre` 대기스크립트로 `/dev/ttyUSB0` 및(선택) session 생성 대기 강화
  - ESP 쪽에서 핸드셰이크 실패 시 주기적 재시도 로직 구현
  - AP 설정에서 multicast 차단/클라이언트 분리 여부 점검

## 원인 상세화 (기술적)

- DDS 바인딩과 인터페이스 준비 타이밍
  - Cyclone DDS 같은 DDS 구현체는 프로세스 시작 시 사용 가능한 네트워크 인터페이스에 바인드하고, 해당 주소/인터페이스로 discovery 패킷을 송수신합니다. Agent가 기동될 때 `wlan0`이 완전히 준비되지 않으면 DDS가 올바른 인터페이스에 바인드하지 못하거나 discovery 패킷이 누락될 수 있습니다.

- 무선 특유의 불안정성
  - Wi‑Fi는 링크 업/다운, DHCP 할당 지연, AP의 멀티캐스트 처리(또는 차단), 클라이언트 분리(client isolation), 전원 절약 모드 등의 영향으로 통신 가능 상태가 유선보다 늦게 안정화되는 경향이 있습니다。

- 로컬 시리얼 세션과 네트워크 디스커버리의 상호작용
  - ESP32 ↔ Agent 간의 micro-XRCE-DDS 세션은 시리얼 링크에서 성립되어야 `Created session` 메시지가 나옵니다. 이 세션 자체는 네트워크와 직접적 의존성은 없지만, Agent가 ROS2 네트워크(즉DDS)에서 자신을 제대로 광고/바인드하지 못하면 Agent 내부의 상태 기계나 ESP 쪽의 재시도 로직에 의해 세션 성립이 실패하거나 이후에 노드가 정상노출되지 않을 수 있습니다。

- 루프백/잘못된 인터페이스 바인드 가능성
  - DDS가 올바른 인터페이스로 바인드하지 못할 경우 로컬 루ープ백(127.0.0.1)이나 다른 인터페이스로 바인드되어 네트워크 상의 다른 머신과 통신이 되지 않는 현상이 발생할 수 있습니다。 이로 인해 `/YB_Car_Node`가 외부에 보이지 않는 것처럼 관찰됩니다。

## 로그 캡처 및 재현 절차 (권장)

사전: Pi에 `tcpdump`가 설치되어 있지 않으면 설치하세요。
```bash
sudo apt update
sudo apt install -y tcpdump
```

1) 인터페이스/네트워크 상태 확인
```bash
ip addr show wlan0
ip route show
nmcli device status   # NetworkManager 사용 시
```

2) RTPS/Discovery 패킷 캡처 (wlan0)
```bash
# RTPS 포트 범위(예: 7400-7600)를 캡처
sudo tcpdump -i wlan0 -n -w /tmp/rtps_wlan0.pcap udp and portrange 7400-7600
# 또는 멀티캐스트만 캡처
sudo tcpdump -i wlan0 -n -w /tmp/mcast_wlan0.pcap multicast
```

3) Agent를 매우 상세 로그로 수동 실행(별 터미널)
```bash
# ROS/Cyclone 설정을 명시하여 실행
source ~/microros_ws/install/setup.bash   # Agent가 빌드된 워크스페이스가 있을 때
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RMW_NETWORK_INTERFACE=wlan0
export ROS_DOMAIN_ID=20
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v9 |& tee /tmp/agent_verbose.log
```

4) Agent 바인드 확인(다른 터미널)
```bash
ss -uap | grep micro_ros_agent || ss -uap | grep 7400
```

5) 재현 시나리오
  - A. ESP 먼저 부팅 → Pi(Agent) 시작: `agent_verbose.log`와 `tcpdump`를 동시에 실행하여 discovery/handshake 패킷이 오가는지 비교。
  - B. Pi(Agent) 먼저 부팅 → ESP 리셋: 동일 캡처로 `Created session` 발생 시점 확인。

6) 시리얼(ESP↔Agent) 덤프(참고)
```bash
# 바이너리 데이터이므로 시간 제한을 두고 캡처
sudo timeout 10 cat /dev/ttyUSB0 > /tmp/serial.bin || true
xxd /tmp/serial.bin | head -n 200
```

7) pcap 분석(로컬)
```bash
sudo tcpdump -r /tmp/rtps_wlan0.pcap -n -vv
```

8) network-online 서비스 확인
```bash
systemctl status NetworkManager-wait-online.service
systemctl status systemd-networkd-wait-online.service
```

9) AP/라우터 점검 (가능하면)
  - AP의 client isolation(클라이언트 분리) 또는 IGMP/멀티캐스트 필터링이 활성화되어 있는지 확인。

참고: ESP가 "먼저" 부팅된 경우 세션이 생성되지 않는 가장 흔한 실무적 원인은 ESP 쪽의 재시도/타임아웃 정책(Agent 부재 시 재시도하지 않음) 또는 Agent가 시작될 때 네트워크 바인딩이 잘못되어 Agent 내부에서 외부와의 discovery 응답이 막혀 발생합니다。

추가로 원하시면 위 명령들을 바로 복사해서 Pi에서 실행하실 수 있도록, 실행 순서별 스크립트로 만들어 드리겠습니다。
