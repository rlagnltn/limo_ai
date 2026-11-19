# 신호등 + 차선주행

`line_slow/detect_line.py`를 기본 차선 검출 코드로 사용한다.
`/lane_state`의 중앙 기준 오차에 `lane_p_gain`(기본 0.01)을 곱해
각속도를 계산하고, `traffic_light_trigger.py`만 `/cmd_vel`을 발행한다.
차선 검출은 해당 코드의 기본값을 사용한다: 영상 전체 폭, ROI y=280~480,
카메라 중앙 기준(`reference_distance=0`). 기존 왼쪽 ROI 설정 파일은 사용하지 않는다.
기존 ROI/중심점/YOLO 표시 카메라 미리보기도 기본으로 열린다.
미리보기를 끄려면 실행 명령에 `show_visualization:=false`를 추가한다.

## 실행

카메라, 베이스, 기존 `testv2.engine` YOLO 검출기는 계속 실행한다.
기존 `traffic_light_trigger.py`는 Ctrl+C로 종료한 뒤 다음 명령을 실행한다.
`limo_control`, `line_slow`, 텔레옵 등 다른 `/cmd_vel` 발행 노드는 함께 실행하지 않는다.
`track_application_launch.py` 대신 아래 실행 파일을 사용한다.

```bash
source /home/wego/wego_ws/install/setup.bash
ros2 launch /home/wego/wego_ws/src/mission/traffic_lane.launch.py
```

수정한 소스 파일을 직접 실행하므로 별도 colcon 빌드는 필요 없다.
카메라는 `/camera/color/image_raw`(차선 검출)와
`/camera/color/image_raw/compressed`(YOLO/미리보기)를 모두 발행해야 한다.

속도와 조향 이득을 변경하려면:

```bash
ros2 launch /home/wego/wego_ws/src/mission/traffic_lane.launch.py \
  forward_speed:=0.2 lane_p_gain:=0.01
```

## 동작

- 차선과 YOLO 메시지가 준비되면 0.2m/s로 차선주행을 시작한다.
- 빨간불: 신뢰도 0.7 이상, 면적 3000px² 이상에서 정지한다.
  기존 `red_stop_delay_sec`를 설정하면 그 시간 동안 차선을 따라간 후 정지한다.
- 정지 중 초록불: 신뢰도 0.7 이상, 면적 2700px² 이상이면 3초간
  차선주행 후 정지한다. 두 색이 동시에 조건을 만족하면 빨간불을 우선한다.
- 기존 비상정지, 신호 재무장, YOLO 타임아웃 로직을 유지한다.
- 차선이 없거나 `/lane_state`가 0.5초 이상 끊기면
  선속도와 각속도를 모두 0으로 보낸다. 차선이 복구되면 현재 신호등 상태가
  주행을 허용하는 동안 재개한다. 초록불의 3초 타이머는 대기 중에도 진행된다.
- 미리보기의 `Lane: OK / WAIT / LOST`로 차선 입력 상태를 확인한다.
  Q/Esc 또는 Ctrl+C로 제어 노드를 종료하면 정지 명령을 보낸다.

## 검증

```bash
source /home/wego/wego_ws/install/setup.bash
/usr/bin/python3 -m unittest discover \
  -s /home/wego/wego_ws/src/mission -p test_traffic_lane.py -v
```

ROS 소켓 없이 실제 메시지와 합성 영상으로 검증한다.
실제 카메라/조향 및 ROS 토픽 연결은 로봇에서 확인해야 한다.
