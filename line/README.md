# LIMO 차선 주행 (simple_lane) 이식 묶음

2026-11-19 기준. 대시보드 "센서 활성화" → "차선 주행" 버튼을 누르면 차선 인식 카메라 창이 뜨고,
리모컨 자율주행 레버를 올리면 차선을 따라 자동 주행한다.

## 들어 있는 파일

| 묶음 안 경로 | 받는 차에 둘 곳 | 설명 |
|---|---|---|
| `jetracer_ros2/jetracer_ros2/simple_lane.py` | `~/wego_ws/src/jetracer_ros2/jetracer_ros2/` | 차선 인식 + 주행 (핵심) |
| `jetracer_ros2/jetracer_ros2/three_lane.py`, `three_lane_simple.py` | 같은 곳 | 3차선 구간 (simple_lane 이 불러옴) |
| `jetracer_ros2/jetracer_ros2/m8_observers.py`, `stop_line_detector.py` | 같은 곳 | 주행 모드에서 불러옴 |
| `jetracer_ros2/launch/lane_only.launch.py` | `~/wego_ws/src/jetracer_ros2/launch/` | 차선 주행만 실행 |
| `jetracer_ros2/launch/jetracer_teleop.launch.py` | 같은 곳 | 센서 활성화 (드라이버·카메라·라이다·`simple_lane bridge` 변환 노드) |
| `tune/simple_lane_tune_v9.json` | `~/simple_lane_tune_v9.json` | 슬라이더 튜닝 값 (없으면 코드 기본값으로 달림) |
| `dashboard_bashes/lane_detection.sh` | `~/limo_ros2_start_shortcut/bashes/` | "차선 주행" 버튼 (기본 TEST_MISSION=lane) |
| `dashboard_bashes/limo_bringup_without_tf.sh` | 같은 곳 | "센서 활성화" 버튼 |
| `dashboard_bashes/autorace_runtime.bash` | `~/autorace_runtime.bash` | 위 버튼들이 source 하는 환경 설정 |
| `limo_base/src/limo_ackerman_driver.cpp` | `~/wego_ws/src/limo_ros2/limo_base/src/` | 레버로 자율주행 진입 수정본 |
| `limo_base/limo_ackerman_driver.diff` | (참고) | 원본 WeGo 드라이버 대비 바뀐 부분 |

`setup.py` 의 console_scripts 에 아래 줄이 있어야 한다 (없으면 추가):
```
'simple_lane = jetracer_ros2.simple_lane:main',
```

## 설치 (받는 차)
1. 받는 차의 같은 파일을 먼저 백업한다.
2. 위 표대로 복사한다.
3. 빌드:
   ```
   cd ~/wego_ws
   colcon build --packages-select jetracer_ros2 limo_base
   ```
   (드라이버를 바꾸지 않으려면 `limo_base` 는 빼도 된다. 단 레버를 올려도 control_mode 가 1(코드 제어)이
   안 되어 차가 안 움직이면 드라이버 수정본이 필요하다.)

## 꼭 차마다 확인할 것
- **카메라 BEV 보정** (`simple_lane.py` 의 `BEV_SRC`, `BEV_GROUND`): 이 값은 보낸 차 카메라 장착 각도 기준.
  카메라 각도가 조금만 달라도 차로 폭·중심이 어긋나 한쪽 선에 붙는다.
  직선 차로 가운데에 차를 두고 `ros2 run jetracer_ros2 simple_lane calib` 로 확인
  (양쪽 선 간격 약 43cm, 차가 가운데면 중심 0cm 근처가 정상).
- **조향 배율** (`BRIDGE_GAIN = 1.3`): 보낸 차 실측값. 커브에서 너무 덜/더 꺾이면 `simple_lane turncal` 로 확인.
- **색 범위** (튜닝 창 h_min/h_max/s_min/v_min): 조명이 다르면 노란 선 마스크가 달라진다. 슬라이더로 맞추면 자동 저장.

## 사용
1. 차를 차로 가운데에 진행 방향으로 둔다.
2. 대시보드 "센서 활성화" → "차선 주행".
3. 창 두 개: `simple_lane`(카메라 + 차선 인식 + 위에서 본 화면), `simple_lane_tune`(슬라이더).
4. 리모컨 자율주행 레버를 올리면 출발, 내리면 정지.
5. 종료: 터미널에서 Ctrl+C (정지 명령 보내고 기록 저장).
6. 기록: `~/simple_lane_log/<시각>_drive` (프레임 + log.csv). 끄려면 `LANE_RECORD=false`.
   다시 보기: `ros2 run jetracer_ros2 simple_lane replay ~/simple_lane_log/<폴더>`

## 2026-11-19 수정 내용
- 오른쪽 커브에서 오른쪽 선에 붙던 문제: 두 선이 안 맞아 한쪽을 버린 뒤, 남은 선을 잘못 잰 폭(60~64cm)의
  절반으로 띄우던 것을 직선에서 기억한 폭(약 35~43cm)으로 띄우게 수정 (`LaneEngine.process`).
- 카메라 창이 슬라이더에 가려 안 보이던 문제: 영상 창과 슬라이더 창 분리.
- "차선 주행" 버튼 기본을 차선 주행 시험으로 (`lane_detection.sh`, 주차/라바콘은 `TEST_MISSION=parking|cone`,
  이 경우 주차·라바콘 파일이 따로 필요).
- 아직 실차 재검증 전: 오른쪽 커브 수정은 기록 재생으로만 확인. 오른쪽(흰) 선 인식이 약한 문제는 남아 있음.
