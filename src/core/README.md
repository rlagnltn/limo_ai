# LIMO v3: jihyuk 구조 + 3차선 장애물 회피 수정

`jihyuk`을 기준으로 3차선 경로 계획·중앙 복귀·이후 LiDAR 해제 조건만 추가했다.
콘 구간, 일반 차선 검출 방식, 갈림길 회전 방식은 `jihyuk` 그대로다.
`v1`의 별도 합류 추적과 지그재그 로직은 포함하지 않는다.

이 패키지는 코스용 초기 구현이다. 실차 실행은 검증하지 않았다.
미션 순서·표지판 처리, 회피/갈림길/주차 계산을 구현하고 오프라인 테스트했다.
`testv2.engine` 메타데이터의 클래스 번호와 실제 연결을 확인했다.

## 실행

최초 빌드(또는 Python 수정 후):

```bash
cd ~/wego_ws
colcon build --packages-select core dl_ros2_application ydlidar_ros2_driver
source install/setup.bash
```

각 터미널에서 `source ~/wego_ws/install/setup.bash` 후 아래 4개를 실행한다.

```bash
# 터미널 1: 기존 센서/차체 브링업
ros2 launch wego teleop_launch.py

# 터미널 2: 차선 검출. 여기서는 모터 명령을 보내지 않는다.
ros2 launch core lane.launch.py

# 터미널 3: YOLO와 미션별 LiDAR 모터 시작/정지 관리.
ros2 launch core missions.launch.py

# 터미널 4: 전체 순서와 주행 제어
ros2 run core core.core
```

기존 `line_slow`, `traffic_light_trigger`, `track_application` 또는 텔레옵 제어기를
함께 실행하지 않는다. 코어만 `/cmd_vel`을 발행한다. 내부 제어기는
`/core/control_cmd`, LiDAR 노드는 `/e_stop`과 `/core/emergency_cmd`를 발행한다.
카메라/차체 입력이나 속도 후보가 끊기면 코어가 정지한다.
LiDAR가 켜져 있는 동안에는 스캔과 정지 heartbeat를 계속 요구한다.

## LiDAR 모터 사용 구간

브링업은 `auto_start: false`로 모터를 정지시킨 채 서비스 요청을 기다린다.
4번 장애물 회피와 6번 라바콘 회피 단계에서 모터/스캔을 시작한다.
3차선 단계가 끝나도 LiDAR를 유지하며, 좌·우 표지판 중 하나가 확인되고 최신 스캔으로
전방 0.8m·좌우 0.3m에 장애물이 없고 직진 경로가 충분히 관측됐을 때 해제를 허용한다.
콘 구간은 기존대로 진입 시 켜고, 콘 단계 종료 시 끈다. 종료·고장 정지 때도 모터를 정지한다.
모터 상태 전환 확인 중에는 차량이 정지한다. 켤 때는 새로운 스캔이 들어와야 회피를 시작한다.
코어 종료 후 요구 heartbeat가 끊기면 모터를 정지한다.
드라이버 노드는 재시작 요청을 받기 위해 연결된 대기 상태를 유지하며 전원 자체를 차단하지 않는다.

`/core/lidar_enabled`는 코어의 사용 요구, `/core/lidar_state`는 모터 시작/정지 응답 상태다.
실제 제어는 `/set_scan_enabled` std_srvs/SetBool 서비스가 수행한다.
수정 적용 시 1번 브링업·3번 미션 런치·4번 코어를 재실행한다.

## 미션 파일과 보정 파일

|단계|Python (`core/missions/`)|YAML (`config/`)|시작과 완료|
|---|---|---|---|
|1 방지턱 서행|mission_01_slow.py|mission_01_slow.yaml|class 5 확인 → 감속 → 지정 거리·최소 시간 경과 및 표지판 사라짐|
|2 서행 유도선|mission_02_guide.py|mission_02_guide.yaml|앞 단계 완료 → 저속 차선 추종 → 지정 거리 통과|
|3 보행자|mission_03_pedestrian.py|mission_03_pedestrian.yaml|class 4 → 정지선 또는 보정 거리에서 정지 → 실제 정지 확인 후 3초 대기 → 발진|
|4 3차선 장애물|mission_04_obstacle.py|mission_04_obstacle.yaml|3차선 지속 검출 → LiDAR로 가능한 차선 선택 → 최소 거리 주행·중앙 8cm/헤딩 0.12rad 이내·방향 표지판·유효한 전방 비움 1초 확인|
|5 갈림길|mission_05_intersection.py|mission_05_intersection.yaml|class 2 좌/class 3 우 → 진입 거리·오도메트리 회전각 → 차선 재검출 및 출구 거리|
|6 라바콘|mission_06_cone.py|mission_06_cone.yaml|class 7 → LiDAR 좌우 사이 통로 추종 → 최소 거리·표지판 및 근거리 통로 장애물 사라짐·차선 재검출 지속|
|7 횡단보도/신호등|mission_07_traffic.py|mission_07_traffic.yaml|횡단보도 또는 신호등 감지 → class 0 면적 2900px² 이상 즉시 정지 → class 1 면적 2700px² 이상 차선 복귀. 동시 검출은 빨강 우선|
|8 T자 주차|mission_08_parking.py|mission_08_parking.yaml|class 6 → 전진 정렬·정지·후진 원호·후진 직선 → 주차 영역/자세/정지 확인|

앞 단계에서 발견한 다른 표지판은 미션을 바꾸지 않는다. 단계 전환 시 검출 확인
카운터를 초기화한다. 주차 표지판은 8단계에서만 처리한다.
각 미션은 명령 후보와 완료만 반환하며 다음 미션을 직접 실행하지 않는다.
코어(`core/core.py`)와 순서 관리자(`core/course.py`)가 활성 미션을 하나만 선택한다.

공통 파일:

- `config/course.yaml`: 시작 단계, 단독 테스트 모드, YOLO 신뢰도/클래스별 픽셀 면적,
  연속 검출 프레임 수, 타임아웃, 클래스명 연결.
- `config/control.yaml`: 차선 추종 속도·게인·차체 치수·Ackermann 설정·보정 확인.
  `require_ackermann: false`가 기본이므로 보고된 motion_mode만으로 코어가 주행을 차단하지 않는다.
- `config/lane.yaml`: ROI/HLS 색상 필터·카메라→지면 호모그래피.
- `config/lidar.yaml`: LiDAR/TF frame·스캔 유효성·차체 치수·정지 거리 여유.
- `../line_slow/detect_line.py`: 차선/3차선/정지선/횡단보도 영상 처리.
- `../line_slow/lane_change.py`: 도로 방향의 5차식 경로를 오도메트리 좌표에 고정해 추적.
  차체 전체·도로 경계·장애물·관측 범위·조향 한계를 검사하며 중앙 복귀를 우선한다.
  최대 계획 범위 2.6m, 회피 속도 상한 0.15m/s.
- `../line_slow/limo_control.py`: 기존 제어기. `lane_change_curvature`, `cone_curvature`,
  `compute_control` 내 branch 모드, `parking_control`이 실제 조향 계산을 한다.
- `core/parking_path.py`: T자 주차 경로 생성.

설정 디렉터리는 소스 `~/wego_ws/src/core/config/`가 있으면 그곳을 우선 사용하고,
배포 환경에서는 설치된 `share/core/config/`를 사용한다. `CORE_CONFIG_DIR`로 바꿀 수 있다.
YAML만 수정하면 관련 터미널을 재실행한다. 소스가 있는 이 환경에서는 YAML 재빌드가 필요 없다.
CLI의 `--ros-args -p 이름:=값` 또는 마지막 `--params-file`은 기본 설정 뒤에 적용된다.

## v3 보정값 입력

두 활성화 플래그는 이미 `true`다. 전달된 2026-11-16 깊이 평면 보정 H와 640×480 해상도를 적용했다.
[보정 출처와 정확도 한계](CALIBRATION.md)를 먼저 확인한다. 현재 카메라 장착 상태와의 일치 여부는 별도 확인이 필요하다.

| 파일 | 입력·확인할 보정값 |
| --- | --- |
| `src/core/config/lane.yaml` | `ground_homography`: 전체 영상 픽셀 → base_link 지면 [전방 m, 왼쪽 m]의 3×3 행렬 9개를 행 순서로 입력. 소수점 숫자 사용 |
| 같은 파일 | `calibration_width`, `calibration_height`: 행렬 보정에 사용한 영상 해상도. 기본 640×480 |
| `src/core/config/control.yaml` | 실측 차체가 기본값과 다르면 `wheelbase`, `rear_axle_offset`, `robot_front`, `robot_rear`, `robot_width`, `max_steering_angle`을 수정 |
| `src/core/config/lidar.yaml` | 차체 전후 길이·폭은 control.yaml과 일치시킨다. LiDAR 장착 TF가 실제 위치·방향과 맞는지 확인 |

현재 H는 유효한 3×3 행렬로 입력돼 있다. 이전 보정 기준점과의 RMS 차이 약 0.325m가 있어
절대 거리 정확도를 확인한 것으로 보지는 않는다. 0 행렬·잘못된 크기·비유한 값은 주행 근거로 사용하지 않는다.
센서 영상 해상도가 보정 해상도와 다를 때도 지면 좌표를 사용하지 않는다.

값 입력 후 기존 4개 터미널 실행 방법을 그대로 사용한다. Python 변경을 반영하려면 위 빌드 명령을 수행한다.
소스 설정을 우선 읽는 기본 워크스페이스에서는 YAML만 수정한 경우 관련 노드를 재실행하면 된다.
설치본만 사용하는 환경이면 YAML 수정 후에도 다시 빌드한다.

## 실차 보정이 필요한 가정

- 기본 속도는 0.3m/s, 서행은 0.2m/s, 차선 신뢰도가 0.8 미만이면 최대 0.12m/s다.
  커브 감속·가속 제한에 따라 실제 출력은 더 낮아질 수 있다. 주차는 0.05m/s다.
- `control.yaml`의 `geometry_verified`와 `lane.yaml`의 `ground_calibrated`는 요청에 따라 true다.
  최신 기존값을 입력했지만 실제 장착 일치와 독립적인 거리 정확도를 검증한 것은 아니다.
  호모그래피는 전체 영상 픽셀 [u,v,1]을 base_link 지면 [전방m,왼쪽m,1]로 보낸다.
  `calibration_width/height`는 측정에 쓴 영상 해상도다. 기본 0 행렬은 유효한 보정이 아니다.
- 방지턱 노면 문구는 YOLO class 5가 검출해야 한다. 별도 OCR은 없다.
  유도선 전용 검출기는 없고, 방지턱 이후 `guide_distance`만큼 저속 차선 추종한다.
- 보행자 정지는 보정된 정지선이 있으면 차체 전방에서 `stop_line_margin`만큼
  떨어져 정지한다. 정지선 거리가 없으면 표지판 확인 시점부터 지정한 오도메트리 거리를 쓴다.
  거리 지정 방식은 최신 오도메트리가 있으면 `geometry_verified: false`에서도 허용한다.
  카메라 정지선 거리를 계산하는 방식은 여전히 지면 보정이 필요하다.
  이 거리는 실제 카메라 표지판 위치에 따라 보정해야 한다.
- 신호등 미션은 빨간불 박스 면적 2900px² 이상에서 즉시 정지하고, 실제 정지 후 초록불 2700px² 이상에서 차선 추종으로 복귀한다. 보행자(class 4)·서행(class 5)은 6300px² 이상, 나머지 class 2/3/6/7은 1800px² 이상을 인식 조건으로 쓴다. 면적은 (xmax-xmin)×(ymax-ymin)이며 기존 신뢰도 0.7·초록 및 일반 표지판 3프레임 확인 조건도 적용한다.
- 3차선 회피는 실제 길이 단위 차선 중심/도로 경계가 필요하다. 종료는 휴리스틱이다.
  라바콘 종료도 도로변 물체의 영향을 받을 수 있으므로 최소/최대 거리와 확인 시간을 보정한다.
- 갈림길은 표지판 방향 + 일정 곡률/오도메트리 회전각으로 주행한다.
  분기 도로 지도를 생성하거나 주행 가능 영역을 영상으로 완전히 분할하지 않는다.
- T자 주차는 표지판 확인 시 로봇 위치를 기준으로 생성한 시험 경로다.
  빈 주차 공간의 자동 측정/인식은 없다. `parking_approach/radius/reverse/box`를 실제
  주차 위치에 맞춘다. 우측이 기본이며 좌측은 side와 box의 y 범위를 함께 바꾼다.
  경로는 차량 최소 회전 반경을 검사하고, 전진→후진 전 실제 정지를 확인한다.
  주차 구간은 사용자 지정에 따라 LiDAR를 사용하지 않으며, 경로·오도메트리·주차 영역 조건으로 제어한다.
- stage timeout/경로 막힘/센서 문제는 `/core/status`와 `/mission_status`에서 확인한다.

## 미션 하나만 테스트

1~3번 터미널은 그대로 두고, 4번의 전체 코어를 종료한 뒤 실행한다.
이 실행기는 해당 미션 완료 후 정지하며 다음 단계로 넘어가지 않는다.

```bash
ros2 run core mission_03_pedestrian
ros2 run core mission_05_intersection
ros2 run core mission_08_parking
```

8개 모두 `mission_01_slow`부터 `mission_08_parking`까지 실행기가 있다.
동시에 여러 주행 실행기를 실행하지 않는다.

```bash
ros2 topic echo /core/status
ros2 topic echo /mission_status
```

## 미션 진행/완료 확인

카메라 창에 현재 미션/진행 상태, 이동 거리, 실제 속도, 8개 미션별 WAIT/RUN/DONE/FAIL,
마지막 완료 판정 이유를 표시한다. YOLO 박스 면적도 px²로 표시한다.
4번 코어 터미널에는 `[미션 1/8 방지턱 서행] 완료 판정 | ...` 형태로 판정 근거를 출력한다.

`DONE`은 코드의 완료 조건을 만족했다는 뜻이다. 실제 코스에서 정확한 위치에 정지했는지,
장애물을 접촉하지 않았는지 등을 독립적으로 확인한 결과는 아니다.
표지판을 놓친 미션은 WAIT에 머무르고 임의로 성공 처리하지 않는다.

시작·진행·완료·실패 이력은 `/core/events`에 발행하고
`~/.ros/log/mission_events_날짜_시간_PID.jsonl`에 저장한다. 경로는 코어 시작 로그에 나온다.
`/core/status`에도 전체 미션별 판정과 진행 거리가 들어 있다.

```bash
ros2 topic echo /core/events --qos-durability transient_local
```

## 테스트

```bash
source ~/wego_ws/install/setup.bash
/usr/bin/python3 -m unittest discover -s ~/wego_ws/src/core/test -v
```

ROS 노드/소켓 없이 미션 재생, 잘못된 단계의 표지판 무시, 검출 확인/만료,
신호등 정지 우선, 갈림길 방향 충돌, 종료 타임아웃, 주차 기하 및 실제 주차 제어기의
좌우 T자 경로 추종을 검사한다. 운동학 테스트는 타이어 미끄러짐·센서 오차·실제 충돌을 모델링하지 않는다.
