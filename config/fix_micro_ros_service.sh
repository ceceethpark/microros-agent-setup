# ==========================================
# micro-ROS Agent 서비스 Serial 모드로 변경
# ==========================================
# 아래 명령어들을 Pi 터미널에 복사-붙여넣기 하세요

# 1. 서비스 중지 및 백업
sudo systemctl stop micro_ros_agent
sudo cp /etc/systemd/system/micro_ros_agent.service /etc/systemd/system/micro_ros_agent.service.backup

# 2. Serial 모드로 서비스 파일 수정
sudo tee /etc/systemd/system/micro_ros_agent.service > /dev/null << 'EOF'
[Unit]
Description=micro-ROS Agent (serial)
After=network.target

[Service]
Type=simple
User=ros2
WorkingDirectory=/home/ros2
Environment="ROS_DOMAIN_ID=20"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
Environment="CYCLONEDDS_URI=file:///home/ros2/cyclonedds_wlan.xml"
ExecStartPre=/bin/sleep 2
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source /home/ros2/microros_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v6"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# 3. systemd 재로드 및 서비스 시작
sudo systemctl daemon-reload
sudo systemctl start micro_ros_agent

# 4. 서비스 상태 확인
echo "=========================================="
echo "서비스 상태:"
echo "=========================================="
sudo systemctl status micro_ros_agent --no-pager -l

# 5. 로그 확인 (Serial 모드 "dev: /dev/ttyUSB0" 확인)
echo ""
echo "=========================================="
echo "최근 로그:"
echo "=========================================="
journalctl -u micro_ros_agent -n 10 --no-pager

echo ""
echo "✓ 완료! 로그에서 'Serial mode => dev: /dev/ttyUSB0' 확인"
echo "실시간 로그: journalctl -u micro_ros_agent -f"
