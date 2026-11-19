#!/bin/bash
# 대시보드 "센서 활성화" 버튼: 팀 하드웨어 (아커만 차체 드라이버 + 카메라 + 라이다 + cmd_vel 변환)
# 원래 WeGo 버전 백업: /home/wego/limo_backups/dashboard_buttons_20261118_224132/
echo ""
echo "=================================================="
echo " 센서 활성화: ros2 launch jetracer_ros2 jetracer_teleop.launch.py"
echo "=================================================="
sleep 1

source /home/wego/autorace_runtime.bash
cd /home/wego/wego_ws
ros2 launch jetracer_ros2 jetracer_teleop.launch.py
