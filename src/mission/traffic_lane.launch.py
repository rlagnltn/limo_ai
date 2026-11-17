"""Source 파일로 차선 검출 + 신호등 제어 실행 (별도 colcon 빌드 불필요).

카메라, YOLO(testv2.engine), 베이스는 별도로 실행한다.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mission = Path(__file__).resolve().parent
    lane_source = mission.parent / 'line_slow/detect_line.py'
    return LaunchDescription([
        DeclareLaunchArgument('forward_speed', default_value='0.2'),
        DeclareLaunchArgument('lane_p_gain', default_value='0.01'),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument('show_visualization', default_value='true'),
        DeclareLaunchArgument('startup_debug', default_value='false'),
        ExecuteProcess(
            cmd=['/usr/bin/python3', '-u', str(mission / 'trace_startup.py'),
                 LaunchConfiguration('startup_debug'), str(lane_source), '--ros-args',
                 '-p', ['show_visualization:=', LaunchConfiguration('show_visualization')]],
            output='screen'),
        ExecuteProcess(
            cmd=['/usr/bin/python3', '-u', str(mission / 'trace_startup.py'),
                 LaunchConfiguration('startup_debug'), str(mission / 'traffic_light_trigger.py'),
                 '--ros-args', '-p', ['forward_speed:=', LaunchConfiguration('forward_speed')],
                 '-p', ['lane_p_gain:=', LaunchConfiguration('lane_p_gain')],
                 '-p', ['show_window:=', LaunchConfiguration('show_window')]],
            output='screen'),
    ])
