# LIMO: 4개 터미널로 순차 미션 수행

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
LiDAR 데이터와 LiDAR 정지 heartbeat는 4번 장애물 회피에서만 요구한다.

## LiDAR 모터 사용 구간

브링업은 `auto_start: false`로 모터를 정지시킨 채 서비스 요청을 기다린다.
현재 코드 4번 장애물 회피 미션에서만 모터/스캔을 시작한다.
6번 라바콘은 LiDAR 없이 고정 조향 동작 후 정지 상태에서 차선을 재획득한다.
나머지 구간은 모터와 스캔을 정지하고 LiDAR 장애물/미관측/비상정지 조건을 주행에 적용하지 않는다.
모터 상태 전환 확인 중에는 차량이 정지한다. 켤 때는 새로운 스캔이 들어와야 회피를 시작한다.
코어 종료 후 요구 heartbeat가 끊기면 모터를 정지한다.
드라이버 노드는 재시작 요청을 받기 위해 연결된 대기 상태를 유지하며 전원 자체를 차단하지 않는다.

`/core/lidar_enabled`는 코어의 사용 요구, `/core/lidar_state`는 모터 시작/정지 응답 상태다.
실제 제어는 `/set_scan_enabled` std_srvs/SetBool 서비스가 수행한다.
수정 적용 시 1번 브링업·3번 미션 런치·4번 코어를 재실행한다.

## 미션 파일과 보정 파일

|단계|Python (`core/missions/`)|YAML (`config/`)|시작과 완료|
|---|---|---|---|
|1 방지턱 서행|mission_01_slow.py|mission_01_slow.yaml|class 5 확인 → 감속 → 0.6m 이상·5초 이상 경과 및 표지판 0.5초 미검출|
|2 서행 유도선|mission_02_guide.py|mission_02_guide.yaml|ID 5 재인식(6300px² 이상) → 저속 차선 추종 → 1.8m 통과|
|3 보행자|mission_03_pedestrian.py|mission_03_pedestrian.yaml|class 4 → 즉시 정지 → 정지 확인 후 3초 대기 → 차선 추종 복귀 후 20초 → 4번|
|4 3차선 장애물|mission_04_obstacle.py|mission_04_obstacle.yaml|3번 후 자동 진입 → LiDAR 회피 → ID 2/3 단일 방향 확인으로 종료·5번에 방향 전달. 거리·차선수 종료 조건 없음|
|5 갈림길|mission_05_intersection.py|mission_05_intersection.yaml|class 2 좌/class 3 우 → 회전·출구 거리 → 유효한 차선 추종으로 20초 연속 전진한 뒤 8000px² 이상 class 7 동시 5개 검출 → 6번|
|6 라바콘|mission_06_cone.py|mission_06_cone.yaml|5번 완료 후 자동 진입 → 오른쪽 조향10°·0.3m/s·2초 → 왼쪽15°·0.3m/s·2초 → 정지 → 유효한 차선1초 연속 확인 → 7번. LiDAR/후진/거리 종료 없음|
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
  `require_ackermann: false`가 기본이다. 단, 바퀴 조향각을 사용하는 6번은 motion_mode=1(Ackermann)을 요구한다.
- `config/lane.yaml`: ROI/HLS 색상 필터·카메라→지면 호모그래피.
- `config/lidar.yaml`: LiDAR/TF frame·스캔 유효성·차체 치수·정지 거리 여유.
- `../line_slow/detect_line.py`: 차선/3차선/정지선/횡단보도 영상 처리.
- `../line_slow/limo_control.py`: 기존 제어기. `lane_change_curvature`, `forced_cone_control`,
  `compute_control` 내 branch 모드, `parking_control`이 실제 조향 계산을 한다.
- `core/parking_path.py`: T자 주차 경로 생성.

설정 디렉터리는 소스 `~/wego_ws/src/core/config/`가 있으면 그곳을 우선 사용하고,
배포 환경에서는 설치된 `share/core/config/`를 사용한다. `CORE_CONFIG_DIR`로 바꿀 수 있다.
YAML만 수정하면 관련 터미널을 재실행한다. 소스가 있는 이 환경에서는 YAML 재빌드가 필요 없다.
CLI의 `--ros-args -p 이름:=값` 또는 마지막 `--params-file`은 기본 설정 뒤에 적용된다.

## 실차 보정이 필요한 가정

- 기본 차선 추종 속도와 최대 속도는 0.8m/s, 차선 신뢰도가 0.8 미만이면 최대 0.2m/s다.
  일반 차선 추종의 커브 감속은 유지한다. 서행 모드는 커브·낮은 신뢰도에서도 목표 0.2m/s를 유지한다.
  출발 가속 제한과 차선 소실·입력 오류·고장·정지 조건은 계속 우선한다.
  커브 감속·가속 제한에 따라 실제 출력은 더 낮아질 수 있다. 주차는 0.05m/s다.
- `control.yaml`의 `geometry_verified`와 `lane.yaml`의 `ground_calibrated`는 기본 false다.
  실제 차체 치수와 지면 변환을 측정한 뒤 해당 값을 설정한다. 코드가 임의로 true로 바꾸지 않는다.
  호모그래피는 전체 영상 픽셀 [u,v,1]을 base_link 지면 [전방m,왼쪽m,1]로 보낸다.
  `calibration_width/height`는 측정에 쓴 영상 해상도다. 기본 0 행렬은 유효한 보정이 아니다.
- 방지턱 노면 문구는 YOLO class 5가 검출해야 한다. 별도 OCR은 없다.
  유도선 전용 검출기는 없고, 2번 단계에서 ID 5 인식 후 `guide_distance`만큼 저속 차선 추종한다.
- 보행자 미션은 표지판 확인 즉시 속도 0을 명령한다. 오도메트리로 실제 정지를
  확인한 뒤 3초 대기하고 출발한다. 정지선 탐색이나 추가 이동 거리는 사용하지 않는다.
- 신호등 미션은 빨간불 박스 면적 2900px² 이상에서 즉시 정지하고, 실제 정지 후 초록불 2700px² 이상에서 차선 추종으로 복귀한다. 보행자(class 4)·서행(class 5)은 6300px² 이상, 나머지 class 2/3/6/7은 1800px² 이상을 인식 조건으로 쓴다. 면적은 (xmax-xmin)×(ymax-ymin)이며 기존 신뢰도 0.7·초록 및 일반 표지판 3프레임 확인 조건도 적용한다.
- 3차선 회피는 실제 길이 단위 차선 중심/도로 경계가 필요하다. 종료는 갈림길 표지판(ID2/3) 확인이다.
- 6번 조향각은 가상 중앙 바퀴 기준이며 `tan(angle)/wheelbase`를 통해 cmd_vel로 변환한다.
  실제 바퀴각은 차체의 Ackermann 변환/보정과 기구에 따라 달라 실차 확인이 필요하다.
  각2초는 최종 core 출력의 해당 조향 전진 명령 시간을 누적한다(주기 약0.05초).
  입력오류/제어권상실 등으로 core가 속도0을 내보내는 동안은 누적하지 않는다.
  출발 가속 제한은 유지하며 고정 조향 구간에는 일반 차선 커브 감속/조향 변화율 제한을 적용하지 않는다.
  고정 동작 후 실제속도<=0.005m/s와 유효한 차선을1초 연속 확인한다.
  차선검출실패/미션관측공백은 확인 시간을 초기화한다. 카메라/odom/차체 freshness 검사는 계속 유지한다.
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
