"""LiDAR validity, robot-frame obstacles and a fail-closed stop heartbeat."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_of(q):
    values = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(v * v for v in values))
    if not all(math.isfinite(v) for v in values) or norm < 1e-6:
        raise ValueError('Invalid quaternion')
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def decode_scan(scan, transform, limit, inf_is_free=False):
    """Keep invalid rays unknown; return transformed XY hits and coverage."""
    if (not scan.ranges or len(scan.ranges) > 10000 or not math.isfinite(scan.angle_increment) or
            scan.angle_increment == 0 or not math.isfinite(scan.angle_min) or
            not math.isfinite(scan.range_min) or scan.range_min < 0 or
            not math.isfinite(scan.range_max) or scan.range_max <= scan.range_min):
        raise ValueError('Invalid scan geometry')
    t, q = transform.translation, transform.rotation
    norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if norm < 1e-6 or not all(math.isfinite(v) for v in
                            (norm, t.x, t.y, t.z, q.x, q.y, q.z, q.w)):
        raise ValueError('Invalid laser transform')
    x, y, z, w = (v/norm for v in (q.x, q.y, q.z, q.w))
    r00, r01 = 1-2*(y*y+z*z), 2*(x*y-z*w)
    r10, r11 = 2*(x*y+z*w), 1-2*(x*x+z*z)
    points, total, observed = [], [0]*36, [0]*36
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i*scan.angle_increment
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = r00*c+r01*s, r10*c+r11*s
        bearing = math.atan2(dy, dx)
        index = min(35, int((bearing+math.pi)*36/(2*math.pi)))
        total[index] += 1
        finite = math.isfinite(distance) and max(0.01, scan.range_min) <= distance <= scan.range_max
        if finite or (inf_is_free and distance == math.inf):
            observed[index] += 1
        if finite and distance <= limit:
            points.append([t.x+distance*dx, t.y+distance*dy])
    coverage = [n/d if d >= 2 else 0.0 for n, d in zip(observed, total)]
    return points, coverage


class LimoEStop(Node):
    def __init__(self):
        super().__init__('limo_e_stop')
        defaults = dict(base_frame='base_link', scan_timeout=0.5,
                        sensor_max_age=0.5, controller_timeout=0.5, obstacle_range=3.0,
                        robot_front=0.18, robot_rear=0.18, robot_width=0.20,
                        safety_margin=0.04, reaction_time=0.25,
                        braking_deceleration=0.4, max_speed=0.35,
                        clear_scans=3, inf_is_free=False,
                        min_scan_valid_fraction=0.3, min_front_coverage=0.05)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if any(not math.isfinite(self.cfg[k]) or self.cfg[k] <= 0 for k in
               ('scan_timeout', 'sensor_max_age', 'controller_timeout', 'obstacle_range',
                'robot_front', 'robot_rear', 'robot_width', 'braking_deceleration',
                'max_speed', 'reaction_time')):
            raise ValueError('Invalid obstacle/vehicle configuration')
        if not 0 <= self.cfg['safety_margin'] <= 0.2 or self.cfg['clear_scans'] < 1:
            raise ValueError('Invalid stop hysteresis/margin')
        if not (0 < self.cfg['min_scan_valid_fraction'] <= 1 and
                0 < self.cfg['min_front_coverage'] <= 1):
            raise ValueError('Invalid scan health thresholds')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(Bool, 'e_stop', 10)
        self.state_publisher = self.create_publisher(String, 'obstacle_state', 10)
        # Emergency zeros only; the course controller is the sole nonzero writer.
        self.stop_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.controller_subscription = self.create_subscription(
            String, 'mission_status', self.controller_callback, 10)
        self.last_controller = None
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)
        self.last_scan = None
        self.last_odom = None
        self.velocity = 0.0
        self.stop = True
        self.clear_count = 0
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        velocity = msg.twist.twist.linear.x
        if math.isfinite(velocity):
            self.velocity = velocity
            self.last_odom = time.monotonic()

    def publish_stop(self):
        msg = Bool()
        msg.data = self.stop
        self.publisher_.publish(msg)
        if self.stop:
            self.stop_publisher.publish(Twist())

    def controller_callback(self, msg):
        try:
            status = json.loads(msg.data)
            if status.get('version') == 1 and math.isfinite(status['speed']):
                self.last_controller = time.monotonic()
        except (ValueError, KeyError, TypeError, AttributeError):
            pass

    def controller_alive(self, now):
        return self.last_controller is not None and now-self.last_controller <= self.cfg['controller_timeout']

    def laser_callback(self, msg):
        now = time.monotonic()
        state = dict(valid=False, reason='scan_or_transform_unavailable')
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9 - stamp
            if stamp and (age > self.cfg['sensor_max_age'] or age < -0.1):
                raise ValueError('Stale laser scan')
            if not msg.header.frame_id:
                raise ValueError('Missing laser frame')
            transform = self.tf_buffer.lookup_transform(
                self.cfg['base_frame'], msg.header.frame_id, Time()).transform
            points, coverage = decode_scan(
                msg, transform, self.cfg['obstacle_range'], self.cfg['inf_is_free'])
            # Packet health is distinct from complete observation of a planned path.
            # Retain sparse/unknown bins, rather than discarding real obstacle hits.
            good = sum(math.isfinite(r) and max(0.01, msg.range_min) <= r <= msg.range_max
                       for r in msg.ranges)
            scan_health = good/len(msg.ranges)
            front_coverage = sum(coverage[16:20])/4
            valid = (scan_health >= self.cfg['min_scan_valid_fraction'] and
                     front_coverage >= self.cfg['min_front_coverage'])
            speed = (abs(self.velocity) if self.last_odom is not None and
                     now-self.last_odom < self.cfg['scan_timeout'] else self.cfg['max_speed'])
            clearance = (self.cfg['safety_margin'] + speed*self.cfg['reaction_time'] +
                         speed*speed/(2*self.cfg['braking_deceleration']))
            half_width = self.cfg['robot_width']/2+self.cfg['safety_margin']
            reverse = self.last_odom is not None and now-self.last_odom < 0.5 and self.velocity < -0.01
            front = self.cfg['robot_front']+(self.cfg['safety_margin'] if reverse else clearance)
            rear = self.cfg['robot_rear']+(clearance if reverse else self.cfg['safety_margin'])
            blocked = any(-rear < x < front and abs(y) < half_width for x, y in points)
            self.clear_count = self.clear_count+1 if valid and not blocked else 0
            self.stop = (not valid or blocked or self.clear_count < self.cfg['clear_scans'] or
                         not self.controller_alive(now))
            self.last_scan = now
            state = dict(valid=valid, reason='blocked' if blocked else ('clear' if valid else 'insufficient_scan'),
                         frame=self.cfg['base_frame'], points=points, coverage=coverage,
                         scan_health=scan_health, front_coverage=front_coverage,
                         sensor_origin=[transform.translation.x, transform.translation.y],
                         range_limit=min(msg.range_max, self.cfg['obstacle_range']))
        except (TransformException, ValueError, TypeError, OverflowError) as error:
            self.stop = True
            self.clear_count = 0
            state['reason'] = str(error)
        output = String()
        output.data = json.dumps(state, allow_nan=False, separators=(',', ':'))
        self.state_publisher.publish(output)
        self.publish_stop()

    def timer_callback(self):
        now = time.monotonic()
        if (self.last_scan is None or now-self.last_scan > self.cfg['scan_timeout'] or
                not self.controller_alive(now)):
            self.stop = True
            self.clear_count = 0
        self.publish_stop()

    def destroy_node(self):
        self.stop = True
        self.publish_stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LimoEStop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
