# limo_ai

#통합
#source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
ros2 launch core bringup.launch.py

# 1번 터미널

#ros2 launch wego teleop_launch.py

# 2번 터미널

#ros2 launch core lane.launch.py

# 3번 터미널

#ros2 launch core missions.launch.py

# 4번 터미널

# 각 미션에 대해서 개별적으로 테스트할때는

#ros2 run core mission_미션번호_미션이름

# 전체 진행할때는

#ros2 run core core.core
