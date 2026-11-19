#!/bin/bash
# 대시보드 "차선 주행" 버튼: 미션 단독 시험 (기본: 차선 따라가기)
#   - 먼저 "센서 활성화" 버튼으로 하드웨어를 켜 둔다.
#   - TEST_MISSION=lane (기본): 차선 주행만. 차선 인식 카메라 창(simple_lane, 슬라이더 포함)이 뜬다.
#       YOLO·미션 매니저 없이 simple_lane 이 /cmd_vel 로 바로 보낸다.
#       기록: ~/simple_lane_log/<시각>_drive (LANE_RECORD=false 로 끔)
#   - TEST_MISSION=parking: 주차 표지판을 인식했다고 보고 바로 전면 T주차.
#       차를 주차 구역 앞 흰 선보다 약 1.8 m 앞 차로 가운데(횡단보도 지난 자리)에 진행 방향으로 둔다.
#   - TEST_MISSION=cone: 라바콘(M8) 시험. 차를 라바콘 입구 흰 선 앞 차로에 둔다.
#   - 리모컨 자율주행 스위치를 올리면 바로 출발한다.
#   - 종료: 이 터미널에서 Ctrl+C (YOLO 와 화면 창도 같이 꺼진다)
# 원래 WeGo 버전 백업: /home/wego/limo_backups/dashboard_buttons_20261118_224132/
#
# 바꿀 수 있는 값 (환경변수 또는 아래 기본값 수정)
#   TEST_MISSION : lane = 차선 주행 (기본) / parking = M4 주차 / cone = 라바콘 M8
#   LANE_RECORD  : true = simple_lane 프레임+상태 기록(기본) / false
#   PARK_RECORD  : true = ~/bag_m4_parking_<시각> 에 기록(기본) / false
#   M8_PLAN   : simple = 새 단순 콘 주행 cone_follow (기본) / old = 기존 m8_cone (cone_corridor)
#   M8_START  : M2 = 입구 앞 차로에서 시작(기본) / M8 = 이미 콘 안에서 시작
#   M8_RECORD : true = ~/bag_m8_<simple|local>_<시각> 에 기록(기본) / false
TEST_MISSION="${TEST_MISSION:-lane}"
LANE_RECORD="${LANE_RECORD:-true}"
PARK_RECORD="${PARK_RECORD:-true}"
M8_PLAN="${M8_PLAN:-simple}"
M8_START="${M8_START:-M2}"
M8_RECORD="${M8_RECORD:-true}"

echo ""
echo "=================================================="
if [ "$TEST_MISSION" = "lane" ]; then
echo " 차선 주행 시험: 차선 인식 카메라 창 + 레버 올리면 자동 주행, 기록 $LANE_RECORD"
echo " 차 위치: 차로 가운데, 진행 방향"
elif [ "$TEST_MISSION" = "parking" ]; then
echo " 주차(M4) 시험: 표지판 인식했다고 가정하고 바로 전면 T주차, 기록 $PARK_RECORD"
echo " 차 위치: 주차 구역 앞 흰 선보다 약 1.8 m 앞 차로 가운데"
else
echo " 라바콘 시험 ($M8_PLAN): 시작 $M8_START, 기록 $M8_RECORD"
fi
[ "$TEST_MISSION" != "lane" ] && echo " + YOLO 인식 (/yolo/debug/compressed 창)"
echo " 리모컨 자율주행 스위치를 올리면 바로 움직입니다."
echo "=================================================="
sleep 1

source /home/wego/autorace_runtime.bash
cd /home/wego/wego_ws
PARAMS="$(ros2 pkg prefix jetracer_ros2)/share/jetracer_ros2/params/params.yaml"

# 차선 주행 시험: simple_lane 하나만 (자기 창에 차선 인식 화면을 그린다). Ctrl+C 시 정지 명령 보냄
if [ "$TEST_MISSION" = "lane" ]; then
  exec ros2 launch jetracer_ros2 lane_only.launch.py rec:="$LANE_RECORD"
fi

# YOLO (표시용: 콘 주행 판단은 라이다만 사용)
ros2 run jetracer_ros2 yolo_detection_node --ros-args --params-file "$PARAMS" &
YOLO_PID=$!

# YOLO 화면 (YOLO 준비 전에는 카메라 원본을 보여 줌). rqt_image_view 는 이 압축 영상을 못 그려서 바꿈
python3 /home/wego/limo_ros2_start_shortcut/bashes/yolo_view.py &
VIEW_PID=$!

# 주차 시험이면 라이다 화면(위에서 본 모습 + 칸 판정 + 경로)도 띄움
PVIEW_PID=""
if [ "$TEST_MISSION" = "parking" ]; then
  python3 /home/wego/limo_ros2_start_shortcut/bashes/parking_view.py &
  PVIEW_PID=$!
fi

cleanup() {
  kill -INT "$YOLO_PID" 2>/dev/null
  kill -INT "$VIEW_PID" 2>/dev/null
  [ -n "$PVIEW_PID" ] && kill -INT "$PVIEW_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

if [ "$TEST_MISSION" = "parking" ]; then
  ros2 launch jetracer_ros2 m4_parking.launch.py record:="$PARK_RECORD"
elif [ "$M8_PLAN" = "old" ]; then
  ros2 launch jetracer_ros2 m8_cone.launch.py start:="$M8_START" motion:=true record:="$M8_RECORD"
else
  IN_CONE=false; [ "$M8_START" = "M8" ] && IN_CONE=true
  ros2 launch jetracer_ros2 m8_simple.launch.py start_in_cone:="$IN_CONE" record:="$M8_RECORD"
fi
