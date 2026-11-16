"""Show live YOLO detections while the robot is driven manually."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_weight_path',
            default_value=(
                '/home/wego/wego_ws/src/dl_ros2_application/'
                'weight/testv2.engine'
            ),
            description='YOLO model used for the live preview.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw/compressed',
            description='Compressed camera image topic.',
        ),
        DeclareLaunchArgument(
            'viewer', default_value='true',
            description='Open rqt_image_view on /yolo.',
        ),
        Node(
            package='dl_ros2_application',
            executable='yolo_object_detect',
            name='yolo_object_detect_node',
            output='screen',
            parameters=[{
                'model_weight_path': LaunchConfiguration('model_weight_path'),
            }],
            remappings=[(
                '/camera/color/image_raw/compressed',
                LaunchConfiguration('image_topic'),
            )],
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='yolo_preview_viewer',
            arguments=['/yolo'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('viewer')),
        ),
    ])
