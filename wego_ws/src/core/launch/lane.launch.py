from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='core', executable='lane', output='screen',
             parameters=[{'show_visualization': False}]),
    ])
