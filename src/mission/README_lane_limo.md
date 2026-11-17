# LIMO에서 기존 차선/장애물 회피 실행

## 원본 로직

[lane_obstacle_avoidance.py](./lane_obstacle_avoidance.py)의 차선 추종/회피 계산은 유지하고 색상 필터를 노란색으로 변경했다.
노란색 HSV 마스크를 원근변환하고 영상을 아래부터 8개 띠로 나눠 좌우 차선 위치를 찾는다.
현재 차선의 평균 위치와 기준 위치(640 폭 기준 왼쪽 160, 오른쪽 480)의 차이로 조향한다.
전방 약 ±7도에서 장애물이 검출되면 반대 차선을 목표로 선택한다.
차선 변경 시작 조향은 오른쪽 40/왼쪽 -40이며, 목표 차선 검출 띠가 부족하면
오른쪽 33/왼쪽 -20을 유지한다. 목표 차선을 충분히 검출하면 새 차선 추종으로 전환한다.
이 숫자는 원본 모터 명령 단위이며 라디안/초가 아니다.

## 리모 연결

[lane_obstacle_avoidance_ros.py](./lane_obstacle_avoidance_ros.py)가 ROS 입출력을 담당한다.

- `/camera/color/image_raw`: BGR 변환 후 원본 좌표계인 640×480으로 리사이즈.
- `/scan`: `angle_min`, `angle_increment`, 거리 유효 범위를 사용하고, 스캔 시각의 TF로
  `base_link`로 변환한 뒤 360개 각도 칸에 재배열. 관측하지 못한 칸은 NaN 유지.
- `/cmd_vel`: 전진 속도와 `yaw_rate = speed * tan(angle) / wheelbase` 출력.
  원본의 오른쪽 양수 조향을 ROS의 왼쪽 양수 yaw로 반전.
- `~/status`: 대기/정지 사유 및 현재 차선/조향값.

카메라/스캔이 0.5초 이상 끊기거나 TF가 없거나 차선을 잃으면 정지한다.
전방이 0.2m 이내이거나 충분히 관측되지 않으면 정지한다.
목표 차선 방향의 20~90도 영역도 0.35m 이상, 절반 이상 유효 관측이어야 변경을 진행한다.
이 검사는 단순 거리 검사이며 차선 변경 전체 경로의 충돌 가능성을 보장하지는 않는다.
원본의 장애물 임계값 기본값 `10.0`은 단위 근거가 없으므로, ROS 실행 시
미터 단위의 `obstacle_distance=0.5`를 명시적으로 전달한다.

## 실행

리모의 ROS 2 환경과 워크스페이스를 source한 터미널에서 카메라, LiDAR,
로봇 베이스와 `base_link` ↔ 스캔 frame의 TF를 먼저 실행한다.
기존 `limo_control`, `line_slow`, 텔레옵 등 다른 속도 명령 노드는 함께 실행하지 않는다.

```bash
source /home/wego/wego_ws/install/setup.bash
python3 /home/wego/wego_ws/src/mission/lane_obstacle_avoidance.py --ros-args \
  -p speed:=0.05 \
  -p obstacle_distance:=0.5 \
  -p initial_lane:=left
```

별도 colcon 빌드는 필요 없다. ROS 환경에 `rclpy`, `cv_bridge`, `sensor_msgs`,
`geometry_msgs`, `std_msgs`, `tf2_ros`, OpenCV와 NumPy가 있어야 한다.
다른 토픽/차체 frame을 사용하면 `image_topic`, `scan_topic`, `cmd_vel_topic`,
`base_frame` 파라미터로 변경한다. Ackermann 모드에서는 표준 Twist를 받는
`limo_base` 드라이버를 사용해야 한다. AckermannDrive 전용 드라이버에는 직접 연결되지 않는다.

## 실차 보정이 필요한 부분

원근변환의 원본 좌표 `(200,260)` 등은 그대로다. 노란색 HSV 기본 범위는 `[15,70,70]` ~ `[40,255,255]`이며,
`-p lane_hsv_low:="[15, 70, 70]" -p lane_hsv_high:="[40, 255, 255]"`로 조정한다.
640×480 리사이즈만으로 카메라 높이/기울기 차이가 보정되지는 않는다.
차선을 안정적으로 검출하는지 실제 리모 영상으로 확인해야 한다.
`steering_scale=0.007` rad/원본 단위는 시작용 설정값이며 원본 서보 보정값은 제공되지 않았다.
`wheelbase=0.2`, `max_steering_angle=0.35` rad와 함께 실제 차체 기준으로 조정한다.
속도가 낮아도 이 원본 알고리즘의 차선 변경 완료 조건 자체는 실차 검증이 필요하다.

## 오프라인 검증

```bash
python3 -m unittest discover -s /home/wego/wego_ws/src/mission -p test_lane_limo_adapter.py
```

스캔 시작 각도/역방향/TF 회전·이동, 미관측 처리, 조향 부호·제한,
기존 차선 변경 상태 전환과 검출 함수의 실행을 검사한다.
ROS 실행 및 실제 주행은 별도로 검증해야 한다.
[cone_avoidance.py](./cone_avoidance.py)는 변경하지 않았다.

## HSV 슬라이드바로 조정

리모의 NoMachine 등 GUI 데스크톱 안의 ROS 터미널에서 실행한다.
미리보기만 확인할 때는 카메라만 있어도 되며 LiDAR/TF는 필요 없다.
다음 모드는 속도 명령을 발행하지 않는다.

```bash
python3 /home/wego/wego_ws/src/mission/lane_obstacle_avoidance_ros.py --ros-args \
  -p tune_hsv:=true -p preview_only:=true
```

H/S/V의 low/high 총 6개 슬라이더로 조정한다. H는 0~179, S/V는 0~255다.
CAMERA는 원본, MASK는 현재 색상 필터, WARPED MASK는 기존 원근변환 결과다.
MASK에서 노란 차선이 흰색으로 남고 바닥/다른 물체가 검게 사라지도록 조절한다.
low가 high를 넘으면 high를 low까지 올려 유효한 범위를 유지한다.
값은 실제 차선 검출기에 즉시 적용된다.

S 키는 현재 값을 `~/.ros/lane_hsv.yaml`에 저장한다(기존 파일은 덮어쓴다).
`hsv_save_path`로 저장 위치를 변경할 수 있다. Q/Esc 또는 창 닫기는 미리보기만 닫는다.
노드 종료는 터미널에서 Ctrl+C를 사용한다. 주행 중 창을 닫아도 주행 노드는 계속 동작한다.

저장한 값으로 주행:

```bash
python3 /home/wego/wego_ws/src/mission/lane_obstacle_avoidance_ros.py --ros-args \
  --params-file /home/wego/.ros/lane_hsv.yaml -p speed:=0.05
```

주행하면서 조절할 경우 `-p tune_hsv:=true`만 추가하고 `preview_only`는 false로 둔다.
디스플레이가 없으면 창을 열지 않고 경고한다. GUI 없는 환경에서 색상 범위는
기존 HSV 파라미터로 지정할 수 있다. 조정 값은 S로 저장하기 전까지 메모리에만 유지된다.
