#!/bin/bash
# Raspberry Pi .bashrc 완전 정리 스크립트
# 모든 ROS2 관련 설정을 제거하고 Cyclone DDS로 깨끗하게 재설정

echo "==================================="
echo "Raspberry Pi .bashrc 완전 정리 시작"
echo "==================================="

# 백업 생성
BACKUP_FILE=~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)
cp ~/.bashrc "$BACKUP_FILE"
echo "✓ 백업 생성: $BACKUP_FILE"

# 임시 파일 생성
TEMP_FILE=$(mktemp)

# .bashrc를 읽으면서 ROS2 섹션 전체 제거
IN_ROS_SECTION=0

while IFS= read -r line; do
    # ROS2 섹션 시작 감지
    if [[ "$line" =~ "ROS2" ]] || [[ "$line" =~ "ros/jazzy" ]] || [[ "$line" =~ "=== ROS" ]]; then
        IN_ROS_SECTION=1
        continue
    fi
    
    # ROS 관련 라인 건너뛰기
    if [[ "$line" =~ "source /opt/ros" ]] || \
       [[ "$line" =~ "ROS_DOMAIN_ID" ]] || \
       [[ "$line" =~ "RMW_IMPLEMENTATION" ]] || \
       [[ "$line" =~ "rmw_fastrtps" ]] || \
       [[ "$line" =~ "rmw_cyclonedds" ]] || \
       [[ "$line" =~ "FASTRTPS" ]] || \
       [[ "$line" =~ "ROS_LOCALHOST" ]] || \
       [[ "$line" =~ "RCUTILS_COLORIZED" ]] || \
       [[ "$line" =~ "ros2_ws" ]] || \
       [[ "$line" =~ "Distribution:" ]] || \
       [[ "$line" =~ "Domain ID:" ]] || \
       [[ "$line" =~ "Workspace:" ]] || \
       [[ "$line" =~ "ROS2 Environment" ]] || \
       [[ "$line" =~ "======" && $IN_ROS_SECTION -eq 1 ]]; then
        continue
    fi
    
    # 빈 ROS 섹션 종료
    if [[ $IN_ROS_SECTION -eq 1 ]] && [[ ! -z "$line" ]] && [[ ! "$line" =~ ^[[:space:]]*$ ]]; then
        IN_ROS_SECTION=0
    fi
    
    # 정상 라인은 유지
    echo "$line" >> "$TEMP_FILE"
done < ~/.bashrc

# 새 ROS2 설정 추가
cat >> "$TEMP_FILE" << 'EOF'

# ========================================
# ROS2 환경 설정 - Raspberry Pi (Jazzy + Cyclone DDS)
# ========================================

# ROS2 Jazzy 환경 로드
source /opt/ros/jazzy/setup.bash

# 작업 공간 설정 (있는 경우)
# source ~/ros2_ws/install/setup.bash

# ROS_DOMAIN_ID 설정 (멀티 머신 통신용)
export ROS_DOMAIN_ID=20

# Cyclone DDS 사용
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 네트워크 설정
export ROS_LOCALHOST_ONLY=0

# 화면 출력 색상
export RCUTILS_COLORIZED_OUTPUT=1

# 환경 정보 표시
echo "=== ROS2 Environment ==="
echo "Device: Raspberry Pi"
echo "Distribution: Jazzy"
echo "Domain ID: $ROS_DOMAIN_ID"
echo "RMW: $RMW_IMPLEMENTATION"
echo "IP: $(hostname -I | awk '{print $1}')"
echo "========================"

EOF

# 정리된 파일로 교체
mv "$TEMP_FILE" ~/.bashrc
chmod 644 ~/.bashrc

echo ""
echo "✓ .bashrc 정리 완료!"
echo ""
echo "제거된 내용:"
echo "  - 기존 모든 ROS2 설정"
echo "  - Fast-DDS 관련 설정"
echo "  - 중복된 환경 변수"
echo ""
echo "추가된 내용:"
echo "  - Cyclone DDS 설정"
echo "  - 깔끔한 ROS2 환경"
echo ""
echo "==================================="
echo "다음 명령어로 적용하세요:"
echo "  source ~/.bashrc"
echo "==================================="
echo ""
echo "※ 문제 발생 시 복원:"
echo "  cp $BACKUP_FILE ~/.bashrc"
echo ""
