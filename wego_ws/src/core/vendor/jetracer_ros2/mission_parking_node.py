"""M4 전면 T주차 노드 (아커만, 주차 표지판 인식 이후 담당).

로직은 parking_logic.ForwardParking (ROS 없이 test/sim_parking.py 로 시뮬레이션 가능).

입력
  mission/parking/start        Bool     true = 시작 (매니저가 parking_sign 인식 시 보냄), false = 대기로 초기화
  /odom                        Odometry 이동 거리 / 회전 각도
  /scan                        LaserScan 빈 칸 판정, 충돌 방지, 칸 안쪽 정지
  inference/cmd_vel            Twist    차선 노드 조향 (주차 구역 앞까지 차선 따라가기)
  mission/stop_line_distance   Float32  주차 구역 앞 흰 선 위치 (칸 위치 기준점)
출력
  mission/parking/cmd_vel      Twist    매니저가 M4 동안 차로 그대로 넘김
  mission/parking/state        String   상태 (complete = 주차 완료 -> 매니저가 정지 유지)
  mission/parking/debug        String   JSON (칸 판정, 목표 칸, 차로 좌표 위치) 2 Hz

사용법 (단독 시험 시 매니저 없이 /cmd_vel 로 직접):
  ros2 launch jetracer_ros2 m4_parking.launch.py     차선 + 정지선 + 주차, 바로 시작 (대시보드 버튼)
  ros2 run jetracer_ros2 mission_parking_node
  ros2 run jetracer_ros2 mission_parking_node --ros-args -p output_topic:=/cmd_vel
  ros2 topic pub --once /mission/parking/start std_msgs/msg/Bool '{data: true}'
"""

import json
import math
import os
import sys
from dataclasses import fields

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from jetracer_ros2.parking_logic import ForwardParking, ParkingConfig

try:
    from limo_msgs.msg import LimoStatus
except ImportError:  # limo_msgs 없으면 레버 연동만 빠짐
    LimoStatus = None
CONTROL_COMMANDED = 1    # /limo_status control_mode: 1 = 코드 제어 (0 대기, 3 리모컨)

TOPICS = {
    'output_topic': 'mission/parking/cmd_vel',
    'odom_topic': '/odom',
    'scan_topic': '/scan',
    'lane_command_topic': 'inference/cmd_vel',
    'stop_line_topic': 'mission/stop_line_distance',
    'auto_start': False,     # true = 표지판 신호 없이 odom/라이다가 들어오면 바로 시작 (단독 주차 시험용)
    'reset_on_manual': False,  # true = 리모컨 레버로 코드 제어(1)에서 벗어나면 처음으로, 다시 올리면 그 자리에서 새로 시작
}


class MissionParkingNode(Node):
    def __init__(self):
        super().__init__('mission_parking_node')
        for name, value in TOPICS.items():
            self.declare_parameter(name, value)
        cfg = ParkingConfig()
        for f in fields(ParkingConfig):
            self.declare_parameter(f.name, getattr(cfg, f.name))
            setattr(cfg, f.name, type(getattr(cfg, f.name))(self.get_parameter(f.name).value))
        self.logic = ForwardParking(cfg)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.cmd_pub = self.create_publisher(Twist, p('output_topic'), 10)
        self.state_pub = self.create_publisher(String, 'mission/parking/state', 10)
        self.debug_pub = self.create_publisher(String, 'mission/parking/debug', 10)
        self.create_subscription(Bool, 'mission/parking/start', self._start_cb, 10)
        self.create_subscription(Odometry, p('odom_topic'), self._odom_cb, 20)
        self.create_subscription(LaserScan, p('scan_topic'), self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, p('lane_command_topic'), self._lane_cb, 10)
        self.create_subscription(Float32, p('stop_line_topic'), self._line_cb, 10)
        self._control_mode = None
        if LimoStatus is not None and p('reset_on_manual'):
            self.create_subscription(LimoStatus, '/limo_status', self._status_cb, 10)

        self._last_state = None
        self._last_state_pub = 0.0
        self._last_debug = 0.0
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'전면 T주차 노드 준비 (출력 {p("output_topic")}, 회전반경 {cfg.turn_radius_m} m, '
            f'칸 {cfg.cell_width}x{cfg.cell_depth} m). mission/parking/start 대기')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _start_cb(self, msg: Bool):
        if msg.data:
            if self.logic.state == 'WAIT':
                if self.logic.pose is None:
                    self.get_logger().warn('odom 아직 없음: 받으면 바로 시작', throttle_duration_sec=1.0)
                    self._pending_start = True
                    return
                self.logic.start(self._now())
                self.get_logger().info('주차 시작 (주차 표지판)')
        else:
            if self.logic.state != 'WAIT':
                self.get_logger().info('주차 대기로 초기화')
            self.logic.reset()
            self._pending_start = False

    _pending_start = False
    _auto_started = False

    def _status_cb(self, msg):
        mode = int(msg.control_mode)
        prev, self._control_mode = self._control_mode, mode
        if prev == CONTROL_COMMANDED and mode != CONTROL_COMMANDED and self.logic.state != 'WAIT':
            self.logic.reset()
            self._auto_started = False
            self._last_state = None
            self.get_logger().info('리모컨 레버로 코드 제어 해제 -> 주차 처음으로 (다시 올리면 그 자리에서 새로 시작)')

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.logic.on_odom(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, self._now())
        if self._pending_start and self.logic.state == 'WAIT':
            self._pending_start = False
            self.logic.start(self._now())
            self.get_logger().info('주차 시작 (주차 표지판, odom 수신 후)')

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        self.logic.on_scan(angles, np.asarray(msg.ranges, dtype=float), msg.range_min, msg.range_max,
                           self._now())

    def _lane_cb(self, msg: Twist):
        self.logic.on_lane_cmd(msg.linear.x, msg.angular.z, self._now())

    def _line_cb(self, msg: Float32):
        if msg.data >= 0.0:
            self.logic.on_stop_line(float(msg.data), self._now())

    def _tick(self):
        now = self._now()
        lever_ok = (not self.get_parameter('reset_on_manual').value or self._control_mode is None
                    or self._control_mode == CONTROL_COMMANDED)
        if (self.logic.state == 'WAIT' and self.get_parameter('auto_start').value
                and not self._auto_started and self.logic.pose is not None
                and self.logic.scan_t is not None and lever_ok):
            self._auto_started = True
            self.logic.start(now)
            self.get_logger().info('주차 시작 (auto_start: 표지판 인식했다고 가정)')
        if self.logic.state in ('WAIT',):
            return                                   # 시작 전에는 아무것도 보내지 않음
        v, w, state = self.logic.step(now)
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

        state_text = {'DONE': 'complete', 'FAILED': 'failed'}.get(state, state.lower())
        if state != self._last_state:
            d = self.logic.debug()
            self.get_logger().info(f'=== 주차 상태: {state} {self.logic.reason} | {json.dumps(d)}')
            self._last_state = state
            self._last_state_pub = 0.0
        if now - self._last_state_pub >= 1.0:          # 매니저가 놓치지 않게 1초마다 다시
            self.state_pub.publish(String(data=state_text))
            self._last_state_pub = now
        if now - self._last_debug >= 0.5:
            self.debug_pub.publish(String(data=json.dumps(self.logic.debug())))
            self._last_debug = now

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    argv = sys.argv if args is None else args
    ros_args = ['--ros-args']
    try:
        params = os.path.join(get_package_share_directory('jetracer_ros2'), 'params', 'params.yaml')
        if os.path.exists(params):
            ros_args += ['--params-file', params]
    except Exception:  # noqa: BLE001 - 패키지 설치 전 단독 실행
        pass
    if '--ros-args' in argv:
        ros_args += argv[argv.index('--ros-args') + 1:]
    rclpy.init(args=[argv[0]] + ros_args)
    node = None
    try:
        node = MissionParkingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            for _ in range(5):
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
