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

# 터미널 2: 미션용 차선 정보와 기본 차선 주행 명령 후보
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
LiDAR 데이터와 LiDAR 정지 heartbeat는 4번 장애물 회피, 6번 라바콘, 8번 전면 주차 중에 요구한다.

기본 추종과 감속 추종에는 `line_slow`의 `DetectLine`과 `LimoControl`을 사용한다.
`DetectLine`이 발행하는 `/lane_state`를 바탕으로 `LimoControl`이 조향과 속도를 계산한다.
감속 구간에서는 기존 감속 상한을 적용하며, 최종 속도·곡률 제한과 차량 상태 검사는
코어의 제어 경로에서 적용한다. 표준 코어 브링업에서는 `/cmd_vel`을 사용한다.

## LiDAR 모터 사용 구간

브링업은 `auto_start: false`로 모터를 정지시킨 채 서비스 요청을 기다린다.
4번 장애물 회피와 6번 라바콘에 진입할 때, 8번에서는 주차 표지판이 확정된 뒤 모터/스캔을 시작한다.
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
|1 방지턱 서행|mission_01_slow.py|mission_01_slow.yaml|1번 진입 후 새 정지선 검출 → 감속 → 서행 시작 후 7초 경과|
|2 서행 유도선|mission_02_guide.py|mission_02_guide.yaml|2번 진입 후 새 정지선 검출(하단 ROI 흰색 20% 이상) → 저속 차선 추종 → 검출 지점부터 1.6m 통과|
|3 보행자|mission_03_pedestrian.py|mission_03_pedestrian.yaml|class 4(4100px² 이상) → 즉시 정지 → 정지 확인 후 4초 대기 → 차선 추종 재개 3초 후 정지선 대기|
|4 3차선 장애물|mission_04_obstacle.py|mission_04_obstacle.yaml|3번 완료 뒤 새 정지선 검출 → LiDAR 회피 → ID 2/3 단일 방향 확인(4300px² 이상)으로 종료·5번에 방향 전달. 방향 표지판 대기 시간 제한 없음|
|5 갈림길|mission_05_intersection.py|mission_05_intersection.yaml|class 2 좌/class 3 우(직접 검출 4400px² 이상) → 해당 방향 15° 고정 조향으로 0.3m/s 전진 3초 → 즉시 차선 추종 모드로 복귀 → 같은 YOLO 프레임에서 26000px² 이상 class 7을 1개 이상 검출하면 6번|
|6 라바콘|mission_06_cone.py|mission_06_cone.yaml|5번 완료 후 자동 진입 → LiDAR 양쪽 콘 줄 3스캔 확인 → 통로 중앙 추종 → 전방 콘 미검출 5스캔 확인 → 실제 정지 1초 후 일반 차선 추종 → 서행 표지판 ID 5(3500px² 이상) 검출 시 6.5번|
|6.5 라바콘 후 서행|mission_06_cone.py|control.yaml|6번 종료와 동시에 0.2m/s 이하로 서행 → 빨간불 또는 초록불 검출 시 미션 완료 및 신호를 7번으로 전달. 정지와 출발 판정은 7번에서 수행|
|7 신호등|mission_07_traffic.py|mission_07_traffic.yaml|면적 기준을 충족한 신호등(class 0 또는 1) 감지 시 정지 → 실제 정지 후 초록불만 감지되면 차선 추종 복귀. 동시 검출은 빨강 우선|
|8 T자 주차|mission_08_parking.py|mission_08_parking.yaml|class 6 → LiDAR로 A줄 빈 칸 탐색 → 전면 진입 → 완료 후 정지|

앞 단계에서 발견한 다른 표지판은 미션을 바꾸지 않는다. 단계 전환 시 검출 확인
카운터를 초기화한다. 주차 표지판은 8단계에서만 처리한다.
각 미션은 명령 후보와 완료만 반환하며 다음 미션을 직접 실행하지 않는다.
코어(`core/core.py`)와 순서 관리자(`core/course.py`)가 활성 미션을 하나만 선택한다.

공통 파일:

- `config/course.yaml`: 시작 단계, 단독 테스트 모드, YOLO 신뢰도/클래스별 픽셀 면적,
  연속 검출 프레임 수, 입력 유효 시간, 클래스명 연결.
- `config/control.yaml`: 차선 추종 속도·게인·차체 치수·Ackermann 설정·보정 확인.
  `require_ackermann: false`가 기본이다. 단, 바퀴 조향각을 사용하는 6번은 motion_mode=1(Ackermann)을 요구한다.
- `config/lane.yaml`: ROI/HLS 색상 필터·카메라→지면 호모그래피.
- `config/lidar.yaml`: LiDAR/TF frame·스캔 유효성·차체 치수·정지 거리 여유.
- `../line_slow/detect_line.py`: 차선/3차선/정지선/횡단보도 영상 처리.
- `../line_slow/limo_control.py`: 기존 제어기. `lane_change_curvature`, `cone_control`,
  `compute_control` 내 branch 모드가 갈림길 조향을 계산한다.
- `vendor/jetracer_ros2/`: 8번 전면 T주차 알고리즘과 ROS 노드 원본.

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
- 1번과 2번은 4번 진입과 같은 `detect_stopline.py`를 사용한다.
  1번 진입 뒤 새 정지선을 검출하면 7초간 서행한다. 2번 진입 뒤 새 정지선을 검출한
  위치부터 `guide_distance`만큼 저속 차선 추종한다.
- 보행자 미션은 표지판 확인 즉시 속도 0을 명령한다. 오도메트리로 실제 정지를
  확인한 뒤 4초 대기한다. 차선 추종 재개 3초 후 미션을 완료하고 정지선 감지를 켠다.
  새 정지선이 감지되면 4번으로 진입한다.
- 표지판과 신호등의 공통 면적 기준은 `config/course.yaml`에 있다. 면적은
  (xmax-xmin)×(ymax-ymin)이며 신뢰도 0.7 이상과 1프레임 확인이 기본이다.
- 3차선 회피는 실제 길이 단위 차선 중심/도로 경계가 필요하다. 종료는 갈림길 표지판(ID2/3) 확인이다.
- 6번은 실차 단독 테스트의 `ConeFollower`와 `ModeMachine`을 사용한다.
  6번 진입 후 양쪽 콘 줄을 연속 3스캔 확인할 때까지 정지한다. 전방 콘이 연속
  5스캔 검출되지 않으면 정지하고 실제 속도 절댓값 ≤0.005m/s가 1초 유지된 뒤 일반 차선 추종으로 전환한다.
  스캔이 0.3초 이상 끊기면 주행 명령을 정지한다. 미션 관측이 끊기면 정지 확인 시간을 초기화한다.
- 갈림길은 표지판 방향으로 15° 조향을 고정하고 0.3m/s 전진 명령을 총 3초 보낸다.
  이후 즉시 차선 추종 모드로 복귀하고, 차선이 없을 때는 추종기의 정지 조건을 적용한다.
- 8번은 해당 단계에서 주차 표지판 ID 6이 5000px² 이상으로 직접 검출되면 시작한다. LiDAR와 오도메트리로 A1·A2·A3 중 빈 칸을 찾아 전면 진입한다.
  원본의 ROS 노드와 주차 설정을 사용하고, 코어는 Ackermann 모드와 주행 입력을 확인한 뒤 주차 명령을 최종 `/cmd_vel`로 전달한다.
  주차 상태와 상세 판정은 `/mission/parking/state`, `/mission/parking/debug`에서 확인한다.
- 미션별 경과 시간과 코스 전체 시간에 상한을 두지 않는다. 경로 막힘, 센서 문제, 주행 진척 없음은
  `/core/status`와 `/mission_status`에서 확인한다.

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
마지막 완료 판정 이유를 표시한다. 6.5번도 별도 상태로 표시되며, YOLO 박스 면적은 px²로 표시한다.
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
신호등 정지 우선, 갈림길 방향 충돌, 미션 종료 트리거, 주차 기하 및 실제 주차 제어기의
좌우 T자 경로 추종을 검사한다. 운동학 테스트는 타이어 미끄러짐·센서 오차·실제 충돌을 모델링하지 않는다.
