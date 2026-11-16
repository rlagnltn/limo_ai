import rclpy
import signal
import math
import time
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool, String, Float64MultiArray
from sensor_msgs.msg import Imu
from dl_ros2_msgs.msg import BoundingBoxes
from limo_msgs.msg import LimoStatus
from limo_ros2_application.lane_geometry import CorridorMeasurement

class LimoControl(Node):
    def __init__(self):
        super().__init__('limo_control')
        
        # Setting for publisher of cmd velocity
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Setting for subscriber of e_stop and lane_detection
        self.e_stop_subscription = self.create_subscription(
            Bool,
            'e_stop',
            self.e_stop_callback,
            10)
        self.distance_subscription = self.create_subscription(
            Int32,
            'distance_y',
            self.distance_callback,
            10)
        
        # prevent from warning
        self.e_stop_subscription
        self.distance_subscription
        
        # flag and input value of twisting
        self.e_stop_flag = True
        self.control_mode = None
        self.gap = 0
        self.v_marking = False
        self.lane_curve = False
        self.seen_signs = set()
        self.slow_until = None
        self.pedestrian_until = None
        self.turn_direction = None
        self.turn_timer = None
        self.lane_side = 'left'
        self.merge_tracking = False
        self.merge_subscription = None
        self.zigzag_active = False
        self.zigzag_lane = None
        self.zigzag_lane_received = None
        self.imu_yaw = None
        self.imu_yaw_rate = None
        self.imu_received = None
        self.zigzag_target_yaw = None
        self.zigzag_aligned_since = None

        # parameter for default speed, p_gain for twist
        self.declare_parameter('default_speed', 0.2)
        self.declare_parameter('p_gain', 0.02)

        self.default_speed =self.get_parameter('default_speed')
        self.p_gain =self.get_parameter('p_gain')
        self.turn_angle = self.declare_parameter('turn_steering_angle_deg', 30.0).value
        self.turn_duration = self.declare_parameter('turn_duration_sec', 2.0).value
        self.imu_timeout = self.declare_parameter('imu_timeout_sec', 0.5).value
        self.zigzag_lane_timeout = self.declare_parameter('zigzag_lane_timeout_sec', 0.5).value
        self.zigzag_speed_scale = self.declare_parameter('zigzag_speed_scale', 0.5).value
        self.alignment_confirm = self.declare_parameter('zigzag_alignment_confirm_sec', 0.5).value
        self.alignment_max_tilt = math.radians(
            self.declare_parameter('zigzag_alignment_max_tilt_deg', 5.0).value)
        self.alignment_max_center = self.declare_parameter('zigzag_alignment_max_center_error_px', 20.0).value
        self.alignment_max_yaw_rate = math.radians(
            self.declare_parameter('zigzag_alignment_max_yaw_rate_deg_sec', 5.0).value)
        self.alignment_max_bend = self.declare_parameter('zigzag_alignment_max_bend_px', 12.0).value
        self.heading_p_gain = self.declare_parameter('zigzag_heading_p_gain', 8.0).value
        self.heading_d_gain = self.declare_parameter('zigzag_heading_d_gain', 1.0).value
        self.alignment_center_gain = self.declare_parameter('zigzag_alignment_center_gain', 0.02).value
        self.zigzag_boundary_margin = self.declare_parameter('zigzag_boundary_margin_px', 80.0).value
        self.zigzag_boundary_gain = self.declare_parameter('zigzag_boundary_gain', 0.02).value
        self.zigzag_max_steering = math.radians(
            self.declare_parameter('zigzag_max_steering_angle_deg', 15.0).value)
        imu_topic = self.declare_parameter('imu_topic', '/imu').value
        self.imu_subscription = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.zigzag_subscription = self.create_subscription(
            Float64MultiArray, 'zigzag_lane_state', self.zigzag_callback, 1)
        self.lane_publisher = self.create_publisher(
            String, 'lane_side', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.lane_publisher.publish(String(data=self.lane_side))
        self.merge_publisher = self.create_publisher(
            Bool, 'merge_tracking', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.merge_publisher.publish(Bool(data=False))
        self.sign_min_area = self.declare_parameter('sign_min_area_px2', 4200).value
        self.slow_sign_duration = self.declare_parameter('slow_sign_duration_sec', 8.0).value
        self.pedestrian_stop_duration = self.declare_parameter('pedestrian_stop_duration_sec', 3.0).value
        self.sign_subscription = self.create_subscription(
            BoundingBoxes, 'yolo_bounding_boxes', self.sign_callback, 10)
        self.v_subscription = self.create_subscription(
            Bool, 'v_marking', self.v_callback, 10)
        self.curve_subscription = self.create_subscription(
            Bool, 'lane_curve', self.curve_callback, 10)
        self.status_subscription = self.create_subscription(
            LimoStatus, 'limo_status', self.status_callback, qos_profile_sensor_data)

    def e_stop_callback(self, msg):
        self.e_stop_flag = msg.data
    
    def distance_callback(self, msg):
        self.gap = msg.data

    def status_callback(self, msg):
        self.control_mode = msg.control_mode

    def v_callback(self, msg):
        self.v_marking = msg.data

    def curve_callback(self, msg):
        self.lane_curve = msg.data

    def reset_zigzag_heading(self):
        self.zigzag_target_yaw = None
        self.zigzag_aligned_since = None

    def imu_callback(self, msg):
        q = msg.orientation
        values = (q.x, q.y, q.z, q.w, msg.angular_velocity.z)
        self.imu_received = time.monotonic()
        if (msg.orientation_covariance[0] < 0 or msg.angular_velocity_covariance[0] < 0
                or not all(math.isfinite(value) for value in values)):
            self.imu_yaw = self.imu_yaw_rate = None
            self.zigzag_aligned_since = None
            return
        norm = math.hypot(*values[:4])
        if norm < 1e-6:
            self.imu_yaw = self.imu_yaw_rate = None
            self.zigzag_aligned_since = None
            return
        x, y, z, w = (value / norm for value in values[:4])
        self.imu_yaw = math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))
        self.imu_yaw_rate = msg.angular_velocity.z

    def zigzag_callback(self, msg):
        # See detect_line: mode and geometry arrive together from the same image.
        self.zigzag_lane_received = time.monotonic()
        if len(msg.data) != 7 or not all(math.isfinite(value) for value in msg.data):
            self.zigzag_lane = None
            self.zigzag_aligned_since = None
            return
        active = bool(msg.data[0])
        # A section-exit packet cannot release a sensor-loss stop on its own.
        if not active and self.zigzag_active:
            active = (not msg.data[1] or self.imu_yaw is None
                      or self.imu_received is None
                      or time.monotonic() - self.imu_received > self.imu_timeout)
        if active != self.zigzag_active:
            self.reset_zigzag_heading()
        self.zigzag_active = active
        self.zigzag_lane = CorridorMeasurement(*msg.data[2:]) if active and msg.data[1] else None
        if self.zigzag_lane is None:
            self.zigzag_aligned_since = None

    def zigzag_command(self, now, speed):
        lane = self.zigzag_lane
        if (lane is None or self.imu_yaw is None or self.imu_yaw_rate is None
                or self.imu_received is None or now - self.imu_received > self.imu_timeout
                or self.zigzag_lane_received is None
                or now - self.zigzag_lane_received > self.zigzag_lane_timeout):
            # Keep the target, but stop until both sensors are available again.
            self.zigzag_aligned_since = None
            return 0., 0.

        if self.zigzag_target_yaw is None:
            aligned = (abs(lane.tilt_rad) <= self.alignment_max_tilt
                       and abs(lane.center_error_px) <= self.alignment_max_center
                       and lane.bend_px <= self.alignment_max_bend
                       and abs(self.imu_yaw_rate) <= self.alignment_max_yaw_rate
                       and not self.e_stop_flag and not self.pedestrian_stopping())
            if aligned:
                if self.zigzag_aligned_since is None:
                    self.zigzag_aligned_since = now
                if now - self.zigzag_aligned_since >= self.alignment_confirm:
                    self.zigzag_target_yaw = self.imu_yaw
            else:
                self.zigzag_aligned_since = None

        if self.zigzag_target_yaw is None:
            # Align with the averaged corridor axis; never reuse curve steering.
            curvature = (self.heading_p_gain * lane.tilt_rad
                         + self.alignment_center_gain * lane.center_error_px
                         - self.heading_d_gain * self.imu_yaw_rate)
        else:
            error = self.zigzag_target_yaw - self.imu_yaw
            error = math.atan2(math.sin(error), math.cos(error))
            curvature = self.heading_p_gain * error - self.heading_d_gain * self.imu_yaw_rate

        # Paint wiggles do not steer the vehicle unless a boundary is near.
        curvature += self.zigzag_boundary_gain * (
            max(0., self.zigzag_boundary_margin - lane.right_clearance_px)
            - max(0., self.zigzag_boundary_margin - lane.left_clearance_px))
        # Scaling yaw rate with speed preserves steering angle during slowdown.
        radius = 0.2 / math.tan(self.zigzag_max_steering) + 0.172 / 2.
        angular = speed * max(-1. / radius, min(1. / radius, curvature))
        return speed, angular

    def pedestrian_stopping(self):
        return self.pedestrian_until is not None and time.monotonic() < self.pedestrian_until

    def set_lane_side(self, side):
        self.reset_zigzag_heading()
        self.lane_side = side
        self.gap = 0
        self.lane_curve = False
        self.lane_publisher.publish(String(data=side))

    def stop_merge_tracking(self):
        self.merge_tracking = False
        self.merge_publisher.publish(Bool(data=False))
        if self.merge_subscription is not None:
            self.destroy_subscription(self.merge_subscription)
            self.merge_subscription = None

    def merge_callback(self, msg):
        if self.merge_tracking and msg.data:
            self.stop_merge_tracking()
            self.set_lane_side('left')

    def sign_callback(self, msg):
        self.seen_signs.intersection_update(box.class_name for box in msg.bounding_boxes)
        for box in msg.bounding_boxes:
            if box.class_name != 'pedestrian_sign':
                continue
            area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
            if (area >= self.sign_min_area and box.class_name not in self.seen_signs
                    and not self.e_stop_flag and self.control_mode not in (None, 3)):
                self.pedestrian_until = time.monotonic() + self.pedestrian_stop_duration
                self.seen_signs.add(box.class_name)
                self.timer_callback()
        for box in msg.bounding_boxes:
            if box.class_name != 'slow_sign':
                continue
            area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
            if (area >= self.sign_min_area and box.class_name not in self.seen_signs
                    and not self.e_stop_flag and not self.pedestrian_stopping()
                    and self.control_mode not in (None, 3)):
                self.slow_until = time.monotonic() + self.slow_sign_duration
                self.seen_signs.add(box.class_name)
        for box in msg.bounding_boxes:
            if box.class_name not in ('left_sign', 'right_sign'):
                continue
            area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
            if area < self.sign_min_area or box.class_name in self.seen_signs:
                continue
            direction = 'left' if box.class_name == 'left_sign' else 'right'
            if self.start_turn(direction):
                self.seen_signs.add(box.class_name)
                break

    def start_turn(self, direction):
        if (self.turn_direction is not None or self.e_stop_flag or self.pedestrian_stopping()
                or self.control_mode in (None, 3)):
            return False
        self.stop_merge_tracking()
        self.set_lane_side('left')
        self.turn_direction = direction
        # Disconnect lane input only while the triggered turn is active.
        self.destroy_subscription(self.distance_subscription)
        self.distance_subscription = None
        self.gap = 0
        self.turn_timer = self.create_timer(self.turn_duration, self.finish_turn)
        self.timer_callback()
        return True

    def finish_turn(self):
        direction = self.turn_direction
        self.destroy_timer(self.turn_timer)
        self.turn_timer = None
        self.turn_direction = None
        self.merge_tracking = True
        self.merge_subscription = self.create_subscription(
            Bool, 'merge_complete', self.merge_callback, 10)
        self.set_lane_side(direction)
        self.merge_publisher.publish(Bool(data=True))
        self.distance_subscription = self.create_subscription(
            Int32, 'distance_y', self.distance_callback, 10)
        self.timer_callback()

    def timer_callback(self):
        # set the limo speed
        msg = Twist()
        msg.linear.x = self.default_speed.value
        slow_active = self.slow_until is not None and time.monotonic() < self.slow_until
        if slow_active:
            msg.linear.x *= 0.5
        if self.turn_direction is not None:
            # Invert limo_driver's inner-wheel-angle conversion (metres/radians).
            radius = 0.2 / math.tan(math.radians(self.turn_angle)) + 0.172 / 2
            direction = 1 if self.turn_direction == 'left' else -1
            msg.angular.z = direction * msg.linear.x / radius
        elif self.zigzag_active and not self.merge_tracking:
            speed = min(msg.linear.x, self.default_speed.value * self.zigzag_speed_scale)
            msg.linear.x, msg.angular.z = self.zigzag_command(time.monotonic(), speed)
        elif not self.v_marking or self.merge_tracking:
            msg.angular.z = self.gap * self.p_gain.value
            if slow_active:
                msg.angular.z *= 0.5
            if self.lane_curve and not slow_active:
                msg.linear.x *= 0.5
                msg.angular.z *= 0.5

        # Physical obstacle stop and pedestrian stop both override driving.
        if self.e_stop_flag or self.pedestrian_stopping():
            msg.linear.x = 0.0
            msg.angular.z = 0.0
                
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    limo_control = LimoControl()

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(limo_control, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        limo_control.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
