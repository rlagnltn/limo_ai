# limo_ai v3

`jihyuk` 구조에 수정한 3차선 장애물 회피만 추가한 브랜치입니다.
콘 구간은 기존 동작을 유지합니다. 보정 플래그 두 개는 `true`로 설정했습니다.

[보정값 입력과 실행 방법](src/core/README.md)

```bash
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash

# 각각 별도 터미널에서 실행
ros2 launch wego teleop_launch.py
ros2 launch core lane.launch.py
ros2 launch core missions.launch.py
ros2 run core core.core
```

`src/core/config/lane.yaml`에 전달된 2026-11-16 보정 행렬과 640×480 해상도를 적용했습니다.
[보정 출처와 정확도 한계](src/core/CALIBRATION.md)를 확인하세요. 같은 카메라 장착 상태가 필요합니다.
실측 차체 치수가 기본값과 다르면 `control.yaml`과 `lidar.yaml`도 맞춰야 합니다.
최신 행렬의 절대 거리 정확도와 차체 실측은 별도 확인이 필요합니다.
