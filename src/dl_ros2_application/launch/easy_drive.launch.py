"""Start required hardware, original driving nodes and one combined dashboard."""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, filename):
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


def inspect_graph():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    context = Context()
    rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('easy_drive_startup_check', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        return ({name for name in node.get_node_names()},
                {topic: node.count_publishers(topic) for topic in (
                    '/limo_status', '/camera/color/image_raw',
                    '/camera/color/image_raw/compressed', '/scan')})
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def configure(context):
    mode = LaunchConfiguration('bringup').perform(context)
    drive = LaunchConfiguration('drive').perform(context).lower() == 'true'
    names, publishers = inspect_graph()
    duplicate = names.intersection({'dl_control_node', 'trt_track_detect_node',
                                    'yolo_object_detect_node', 'yolo_object_detection_node',
                                    'drive_dashboard', 'left_lane_tracker',
                                    'detect_line', 'limo_control', 'limo_e_stop'})
    if duplicate:
        message = '이미 실행 중인 주행/인식 터미널을 Ctrl+C로 종료한 뒤 다시 실행하세요: ' + ', '.join(sorted(duplicate))
        return [LogInfo(msg=message), EmitEvent(event=Shutdown(reason=message))]

    hardware = []
    base = bool(publishers['/limo_status'])
    camera = bool(publishers['/camera/color/image_raw'] or
                  publishers['/camera/color/image_raw/compressed'])
    lidar = bool(publishers['/scan'])
    if not drive:
        if mode != 'false' and not camera:
            hardware.append(include('orbbec_camera', 'astra_stereo_u3.launch.py'))
    elif mode == 'true' or (mode == 'auto' and not any((base, camera, lidar))):
        hardware.append(include('wego', 'teleop_launch.py'))
    elif mode == 'auto':
        if not base:
            hardware.append(include('limo_base', 'limo_base.launch.py'))
        if not camera:
            hardware.append(include('orbbec_camera', 'astra_stereo_u3.launch.py'))
        if not lidar:
            hardware.append(include('ydlidar_ros2_driver', 'ydlidar.launch.py'))

    tracking = [include('limo_ros2_application', 'track_application_launch.py')] if drive else []
    weights = '/home/wego/wego_ws/src/dl_ros2_application/weight'
    nodes = [
        Node(package='dl_ros2_application', executable='yolo_object_detect',
             name='yolo_object_detect_node', output='screen', respawn=True, respawn_delay=3.0,
             parameters=[{'model_weight_path': os.path.join(weights, 'testv2.pt'),
                          'device': 'cuda:0', 'image_size': 416}]),
        Node(package='dl_ros2_application', executable='drive_dashboard',
             name='drive_dashboard', output='screen',
             parameters=[{'display': ParameterValue(LaunchConfiguration('display'), value_type=bool)}]),
    ]
    shutting_down = {'requested': False}

    def stop_once(event, launch_context):
        # Preserve shutdown on a driving node or dashboard exit; YOLO can respawn.
        if event.action is not nodes[-1] and not (
                drive and isinstance(event.action, Node)
                and event.action.node_name in {'detect_line', 'limo_control', 'limo_e_stop'}):
            return []
        if shutting_down['requested'] or launch_context.is_shutdown:
            return []
        shutting_down['requested'] = True
        return [EmitEvent(event=Shutdown(reason='주행 화면 또는 주행 노드 종료'))]

    # An inference crash must leave the camera and lane-based driving running.
    handlers = [RegisterEventHandler(OnProcessExit(on_exit=stop_once))]
    message = 'LIMO: 자율주행과 통합 화면 시작' if drive else 'LIMO: 카메라와 객체 탐지 화면 시작 (주행 제어 없음)'
    return handlers + hardware + [LogInfo(msg=message)] + tracking + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drive', default_value='true', choices=['true', 'false'],
                              description='Enable driving; false opens camera and detection only.'),
        DeclareLaunchArgument('bringup', default_value='auto', choices=['auto', 'true', 'false'],
                              description='Automatically start missing vehicle, camera and lidar drivers.'),
        DeclareLaunchArgument('display', default_value='true',
                              description='Open the combined driving dashboard window.'),
        OpaqueFunction(function=configure),
    ])
