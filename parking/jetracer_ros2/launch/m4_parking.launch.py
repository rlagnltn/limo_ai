"""주차(M4) 단독 시험: 주차 표지판을 이미 인식했다고 보고 바로 전면 T주차.

하드웨어(jetracer_teleop.launch.py, 대시보드 '센서 활성화')는 따로 켠다. 매니저는 쓰지 않는다:
  simple_lane --(inference/cmd_vel)--> mission_parking_node --(/cmd_vel)--> bridge -> /limo/ack_cmd
  stop_line_node --(mission/stop_line_distance)--> mission_parking_node   (주차 구역 앞 흰 선 = 칸 위치 기준)

차 놓는 곳: 주차 구역 앞 흰 선보다 약 1.8 m 앞 차로 가운데 (횡단보도 지난 자리, M1 이 끝나는 곳), 진행 방향.
리모컨 자율주행 스위치를 올리면 움직인다. 칸 안에 들어가 멈추면 끝.
다시 하려면 레버를 내리고(주차 노드가 처음으로 돌아감) 차를 출발 위치에 놓은 뒤 다시 올린다.

  ros2 launch jetracer_ros2 m4_parking.launch.py
  인자: record:=false
"""
import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag = os.path.expanduser('~/bag_m4_parking_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    topics = ['/scan', '/odom', '/limo_status', '/camera/color/image_raw/compressed',
              '/camera/color/camera_info', '/inference/cmd_vel', '/cmd_vel', '/limo/ack_cmd',
              '/mission/stop_line_distance', '/mission/parking/state', '/mission/parking/debug']
    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('bag', default_value=bag),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag'), *topics],
                       output='screen', condition=IfCondition(LaunchConfiguration('record'))),
        Node(package='jetracer_ros2', executable='simple_lane', output='screen', emulate_tty=True,
             arguments=['drive', '--topic', 'inference/cmd_vel']),
        Node(package='jetracer_ros2', executable='stop_line_node', output='screen'),
        # params.yaml 은 노드가 스스로 읽고, 아래 값은 ros-args 로 덮어쓴다
        Node(package='jetracer_ros2', executable='mission_parking_node', output='screen',
             emulate_tty=True,
             arguments=['--ros-args', '-p', 'output_topic:=/cmd_vel', '-p', 'auto_start:=true', '-p', 'reset_on_manual:=true',
                        '-p', 'max_time_sec:=600.0']),
    ])
