from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from core.configuration import config_files


def generate_launch_description():
    configs = [str(path) for path in config_files()]
    yolo_args = [item for path in configs for item in ('--params-file', path)]
    return LaunchDescription([
        DeclareLaunchArgument('yolo_python', default_value='/home/wego/miniconda3/envs/dl_env/bin/python3'),
        DeclareLaunchArgument('weight', default_value='/home/wego/wego_ws/src/dl_ros2_application/weight/testv2.engine'),
        Node(package='core', executable='perception', output='screen',
             remappings=[('cmd_vel', '/core/emergency_cmd')]),
        ExecuteProcess(cmd=[LaunchConfiguration('yolo_python'), '-u', '-c',
            'from dl_ros2_application.yolo_object_detect import main; main()',
            '--ros-args', *yolo_args, '-p', ['model_weight_path:=', LaunchConfiguration('weight')]],
            output='screen'),
    ])
