# M4 전면 T주차 코드

주차 표지판을 인식한 뒤부터 A줄의 빈 칸에 **전면 진입**으로 주차하고 정지한다. **아커만 모드**용이다.

```
 차로 →  →  →  →  →  →  →
       ▣┌────┬────┬────┐      ▣ = P 표지판 (A1 왼쪽 위 모서리 바깥)
        │ A1 │ A2 │ A3 │      ← 이 중 빈 칸 하나가 목표, 나머지 두 칸엔 차
        ├────┼────┼────┤
        │ B1 │ B2 │ B3 │      ← 차가 있어도 됨
        └────┴────┴────┘      칸 0.51 x 0.6 m, 차로 폭 0.5 m
```

## 실차 결과 (2026-11-19)

- **A2 빈칸 (A1·A3에 장애물):** 실차로 주차에 성공했다.
- 그 전 실차 기록 4번을 bag으로 재생해 봤다. 지금 코드는 4번 모두 A2를 골랐다.
- **아직 실차로 안 해 본 경우:**
  - A1 빈칸: 회전할 때 P 표지판과의 간격이 계산상 1~2 cm로 가장 빡빡하다.
  - A3 빈칸
  - 대회 흐름: 신호등 미션이 끝난 뒤 약 2 m 떨어진 곳에서 출발하는 경우

## 동작 원리

차선 인식은 쓰지 않는다. 라이다와 odom만 쓴다.

1. **시작:** 레버를 올리거나 표지판 신호를 받는 순간의 차 방향을 차로 방향으로 고정한다. 이후 odom으로 그 직선을 따라 서행한다.
2. **칸 위치 찾기 (라이다):** 오른쪽 A줄 영역을 2D 격자로 만들어 장애물 덩어리를 찾는다.
   - 덩어리 두 개 간격이 약 1.02 m(칸 2개)이면 A1+A3에 차 → **목표 A2**
   - 간격이 약 0.51 m(칸 1개)이면 A1+A2 또는 A2+A3. **P 표지판**이 보이는 위치로 구분한다.
   - 같은 판단이 0.6초 이상 유지돼야 목표를 확정한다.
3. **회전:** 목표 칸 중심에서 회전반경(0.5 m)만큼 앞 지점부터 우회전한다. 경로는 "직선 → 1/4 원 → 칸 안쪽 직선"이고, Pure Pursuit로 따라간다.
4. **정지:** 정해진 깊이에 도달하거나, 앞 물체(B칸 차)와 6 cm 이내가 되면 정지하고 `complete`를 보낸다.
5. **안전:** 지금 조향으로 10 cm 더 갔을 때 차체에 라이다 점이 들어오면 정지한다. 차 두 대를 끝내 못 찾으면 정지하고 `failed`를 보낸다.

## 파일

| 파일 | 내용 |
|---|---|
| `jetracer_ros2/jetracer_ros2/parking_logic.py` | **핵심 로직.** ROS 없이 동작한다 (상태 기계, 라이다 격자, 칸 위치 찾기, 경로) |
| `jetracer_ros2/jetracer_ros2/mission_parking_node.py` | ROS 노드. 토픽을 로직에 넘기고 명령을 발행한다 |
| `jetracer_ros2/jetracer_ros2/mission_manager_node.py` | **수정본.** 변경 내용은 `mission_manager_node.diff` 참고 |
| `jetracer_ros2/launch/m4_parking.launch.py` | 주차 단독 시험: 차선 + 정지선 + 주차를 띄우고, 레버를 올리면 바로 시작 |
| `jetracer_ros2/params/params.yaml` | `mission_parking_node` 항목을 교체했다 (나머지는 기존과 같음) |
| `jetracer_ros2/test/sim_parking.py` | 시뮬레이터 (트랙, 주차된 차, 라이다 레이캐스트, 아커만 운동) |
| `jetracer_ros2/test/test_parking_logic.py` | pytest 17개 |
| `jetracer_ros2/test/replay_parking_bag.py` | 실차 bag을 지금 로직에 넣어 판단 과정을 다시 본다 (디버깅용) |
| `dashboard_bashes/parking_view.py` | 확인용 화면: 카메라 + 차선 인식 + 라이다 + 칸 판정 + 경로 |
| `dashboard_bashes/lane_detection.sh` | 대시보드 '차선 주행' 버튼 스크립트 (`TEST_MISSION=parking`이 기본) |

`setup.py`는 바꾸지 않았다. 기존에 등록된 `mission_parking_node` 실행 파일을 그대로 쓴다.

## 매니저 연동 (대회 흐름)

- **입력:**
  - `mission/parking/start` (Bool): 매니저가 `parking_sign`을 받으면 true를 보낸다
  - `/odom`, `/scan`
  - (보조) `inference/cmd_vel`, `mission/stop_line_distance`
- **출력:**
  - `mission/parking/cmd_vel`: 매니저가 M4 동안 차로 넘긴다
  - `mission/parking/state`: `complete`이면 매니저가 정지를 유지한다
  - `mission/parking/debug`: JSON
- **매니저 수정 내용:** YOLO `parking_sign` 이벤트는 처음 확정될 때 한 번만 온다. 그런데 표지판이 신호등(M1) 구간에서 먼저 보일 수 있다. 그래서 M1·M4 중에 본 표지판을 기억해 두고, M4 동안 1초마다 start=true를 다시 보내게 했다.

## 실행

```bash
bash ~/build_autorace.sh --packages-select jetracer_ros2

# 단독 시험 (하드웨어 launch 먼저): 레버 올리면 시작, 내리면 처음으로 돌아감
ros2 launch jetracer_ros2 m4_parking.launch.py
python3 ~/limo_ros2_start_shortcut/bashes/parking_view.py     # 확인 화면 (선택)

# 테스트 / bag 재생
cd ~/wego_ws/src/jetracer_ros2
python3 -m pytest -q test/test_parking_logic.py
python3 test/replay_parking_bag.py ~/bag_m4_parking_<시각>
```

단독 시험 때 차는 **A1에서 0.5~1 m 앞, 차로 가운데에 차로와 평행하게** 놓는다. 출발 방향을 차로 방향으로 쓰기 때문이다.

## 실차에서 맞출 값 (`params.yaml`)

| 파라미터 | 기본값 | 역할 |
|---|---|---|
| `turn_radius_m` | 0.50 | 회전 시작 위치 = 칸 중심 − 이 값 |
| `target_x_offset_m` | −0.02 | 최종 위치 좌우 보정 |
| `park_rear_y_m` | 0.22 | 칸 안으로 들어가는 깊이 |
| `line_to_grid_m`, `start_to_grid_m` | 0.47, 2.30 | 흰 선 / 출발점 기준 칸 위치 추정 (보조, 차 두 대를 못 찾을 때만 의미 있음) |

## 알려진 한계

- 칸에서 2 m 넘게 떨어져 출발하면 출발 방향 오차가 쌓인다. 시뮬레이션에서 2°부터 실패가 생겼다. 대회 흐름에서는 차로 방향 보정이 더 필요하다.
- A줄에 장애물이 두 개 있어야 칸 위치를 잡는다.
- 차선 인식(`simple_lane`) 자체의 문제는 고치지 않았다. 주차는 차선 인식을 쓰지 않도록 우회했다.
