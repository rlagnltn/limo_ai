#!/usr/bin/env python3
"""Original line_slow lane detection, steering and safety in one executable.

Only YOLO speed-trigger handling changes: a matching class and pixel box area
cap forward speed for exactly 10 seconds from the initial detection. Continued
visibility never extends the deadline. Disappearance rearms the next trigger.
Original acceleration and safety checks remain in force after expiry.

Run in the same ROS environment with the original sensor/TF/calibration setup:
  python3 line_slow.py --ros-args -p target_class:=slow_sign \
      -p min_box_area:=3000.0 -p target_speed:=0.05

This starts detect_line, limo_control and limo_e_stop together. Do not also run
those nodes from track_application_launch.py. Parameter files for their original
node names remain usable. Camera, YOLO and base drivers run separately.
The original detector/controller methods are retained below to preserve behavior.
"""

from rclpy.executors import SingleThreadedExecutor
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
                        robot_front=0.18, robot_rear=0.18, robot_width=0.22,
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



import rclpy
from rclpy.node import Node 

import cv2
import numpy as np
import os
import time
import json
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from dl_ros2_msgs.msg import BoundingBoxes

class DetectLine(Node):
    def __init__(self):
        # CV bridge
        self.br = CvBridge()

        # Subscribe camera data
        super().__init__('detect_line')
        self.subscription = self.create_subscription(
                            Image,
                            '/camera/color/image_raw', 
                            self.image_callback, 
                            rclpy.qos.qos_profile_sensor_data)
        self.subscription # to prevent from warning
        self.yolo_boxes = []
        self.yolo_received_at = None
        self.yolo_subscription = self.create_subscription(
            BoundingBoxes, '/yolo_bounding_boxes', self.yolo_callback, 10)

        # Publish result (offset between reference distance and real distance)
        self.dis_publisher = self.create_publisher(Int32, 'distance_y', 10)
        self.lane_publisher = self.create_publisher(String, 'lane_state', 10)
        self.controller_status = {}
        self.controller_received_at = None
        self.status_subscription = self.create_subscription(
            String, 'mission_status', self.status_callback, 10)

        # Publish Image for debugging
        self.debug_publisher = self.create_publisher(Image, 'debug_image', 10)
        self.visualization_publisher = self.create_publisher(Image, 'lane_visualization', 10)
        self.declare_parameter('show_visualization', True)
        self.show_visualization = self.get_parameter('show_visualization').value
        self.visualization_window_name = 'LIMO camera - ROI and lane centre'
        self.visualization_window_ready = False
        if self.show_visualization and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            self.show_visualization = False
            self.get_logger().warning(
                'No desktop display: launch this node from a terminal in the '
                'robot NoMachine desktop to open the camera preview.')
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # Parameters (For Masking Lane, For the reference distance of lane, ROI)
        self.declare_parameter('roi_x_l', 0)
        self.declare_parameter('roi_x_h', 0)  # 0 uses the full image width.
        self.declare_parameter('roi_y_l', 280)
        self.declare_parameter('roi_y_h', 480)
        
        self.roi_x_l=self.get_parameter('roi_x_l')
        self.roi_x_h=self.get_parameter('roi_x_h')
        self.roi_y_l=self.get_parameter('roi_y_l')
        self.roi_y_h=self.get_parameter('roi_y_h')

        self.declare_parameter('lane_h_l', 0)
        self.declare_parameter('lane_l_l', 90)
        self.declare_parameter('lane_s_l', 100)
        self.declare_parameter('lane_h_h', 60)
        self.declare_parameter('lane_l_h', 220)
        self.declare_parameter('lane_s_h', 255)

        lane_h_l=self.get_parameter('lane_h_l')
        lane_l_l=self.get_parameter('lane_l_l')
        lane_s_l=self.get_parameter('lane_s_l')
        lane_h_h=self.get_parameter('lane_h_h')
        lane_l_h=self.get_parameter('lane_l_h')
        lane_s_h=self.get_parameter('lane_s_h')
        
        self.yellow_lane_low = np.array([lane_h_l.value,
                                         lane_l_l.value,
                                         lane_s_l.value])
        self.yellow_lane_high = np.array([lane_h_h.value,
                                          lane_l_h.value, 
                                          lane_s_h.value])
        
        # 0 selects the camera centre; positive values are ROI-relative pixels.
        self.declare_parameter('reference_distance', 0)
        self.reference_distance = self.get_parameter('reference_distance')
        self.declare_parameter('lane_width_ratio', 0.75)
        self.declare_parameter('left_only_center_bias', 0.0)
        self.lane_geometry = None
        self.lane_center = None
        self.lane_width = None
        self.boundary_samples = []
        self.width_profile = None
        self.left_trace = self.right_trace = None
        self.trace_received_at = None
        self.lane_confidence = 0.0
        self.lane_heading_error = 0.0
        self.declare_parameter('ground_calibrated', False)
        self.declare_parameter('ground_homography', [0.0]*9)
        self.declare_parameter('calibration_width', 640)
        self.declare_parameter('calibration_height', 480)

        # Parameter For debugging
        # 0: ROI
        # 1: Masking
        # 2: Detected boundaries and lane centre
        self.declare_parameter('debug_image_num', 2) #default 2
        self.debug_sequence = self.get_parameter('debug_image_num')

        # to syncronize subscriber and publisher
        self.sub_flag = False
        # Keep the last valid steering offset while the lane is out of the ROI.
        self.last_distance_to_ref = 0
        # Extra offset magnitude on loss; 0 restores the last-offset hold behavior.
        self.declare_parameter('lost_lane_offset_boost', 0)
    
    def yolo_callback(self, msg):
        self.yolo_boxes = list(msg.bounding_boxes)
        self.yolo_received_at = time.monotonic()

    def status_callback(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict):
                self.controller_status = status
                self.controller_received_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def ground_point(self, x, y):
        if not self.get_parameter('ground_calibrated').value:
            return None
        if (self.image_.shape[1] != self.get_parameter('calibration_width').value or
                self.image_.shape[0] != self.get_parameter('calibration_height').value):
            return None
        matrix = np.asarray(self.get_parameter('ground_homography').value, dtype=float)
        if matrix.size != 9 or not np.isfinite(matrix).all():
            return None
        matrix = matrix.reshape(3, 3)
        if abs(np.linalg.det(matrix)) < 1e-12:
            return None
        point = matrix @ np.array([x, y, 1.0])
        if abs(point[2]) < 1e-9:
            return None
        gx, gy = point[:2]/point[2]
        return [float(gx), float(gy)] if 0 < gx < 5 and abs(gy) < 3 else None

    @staticmethod
    def longitudinal_white_mask(white):
        """Keep elongated, predominantly longitudinal markings as divider candidates."""
        count, labels, stats, _ = cv2.connectedComponentsWithStats(white, connectivity=8)
        keep = np.zeros(count, dtype=np.uint8)
        for label in range(1, count):
            x, y, width, height, area = stats[label]
            if area < 12 or height < 8:
                continue
            yy, xx = np.nonzero(labels[y:y+height, x:x+width] == label)
            points = np.column_stack((xx, yy)).astype(np.float64)
            covariance = np.cov(points, rowvar=False)
            values, vectors = np.linalg.eigh(covariance)
            axis = vectors[:, 1]
            if values[1] >= 6.25 * max(values[0], 0.25) and abs(axis[1]) >= 0.35:
                keep[label] = 255
        return keep[labels]

    def road_state(self, hls, reference_x, cx, left, right, row, x0, y0):
        """White dividers are considered only inside two measured yellow edges."""
        state = dict(version=1, valid=cx is not None, width=self.roi_.shape[1],
                     offset=0, confidence=0.0, lane_count=0, centers=[],
                     metric_valid=False, ground_centers=[], heading_error=0.0,
                     stop_distance=None)
        if cx is None:
            return state
        state['confidence'] = self.lane_confidence
        state['heading_error'] = self.lane_heading_error
        state['lane_count'], state['centers'] = 1, [float(cx)]
        white = cv2.inRange(hls, np.array([0, 180, 0]), np.array([180, 255, 80]))
        if left is not None and right is not None and len(self.boundary_samples) >= 5:
            samples = self.boundary_samples
            divider_mask = self.longitudinal_white_mask(white)
            fractions = []
            for l, r, y in samples:
                lo, hi = max(0, int(l)), min(white.shape[1], int(r)+1)
                # Broad white paint across the road, including zebra stripes,
                # must not supply evidence for longitudinal dividers.
                if hi <= lo or np.count_nonzero(white[y, lo:hi]) / (hi-lo) >= 0.25:
                    continue
                edges = np.diff(np.pad((divider_mask[y] > 0).astype(np.int8), (1, 1)))
                for start, end in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
                    fraction = ((start+end-1)/2-l)/(r-l)
                    if 2 <= end-start <= 0.08*(r-l) and 0.12 < fraction < 0.88:
                        fractions.append((float(fraction), y))
            dividers = []
            for fraction, _ in sorted(fractions):
                group = [(f, y) for f, y in fractions if abs(f-fraction) < 0.045]
                if len(set(y for _, y in group)) >= 5:
                    position = float(np.median([f for f, _ in group]))
                    if not any(abs(position-v) < 0.12 for v in dividers):
                        dividers.append(position)
            dividers.sort()
            if len(dividers) == 2:
                splits = [0.0]+dividers+[1.0]
                gaps = np.diff(splits)
                if np.min(gaps) >= 0.18 and np.max(gaps) <= 0.48:
                    positions = [float(left+(right-left)*(a+b)/2)
                                 for a, b in zip(splits, splits[1:])]
                    state['lane_count'], state['centers'] = 3, positions
                    cx = min(positions, key=lambda x: abs(x-reference_x))
            ordered = sorted(samples, key=lambda sample: sample[2])
            far = (ordered[0][0]+ordered[0][1])/2
            near = (ordered[-1][0]+ordered[-1][1])/2
            state['heading_error'] = float(np.clip((near-far)/max(1.0, right-left), -1, 1))
            left_ground = self.ground_point(x0+left, y0+row)
            right_ground = self.ground_point(x0+right, y0+row)
            ground = [self.ground_point(x0+center, y0+row) for center in state['centers']]
            if left_ground and right_ground and all(p is not None for p in ground):
                if left_ground[1] > right_ground[1]:
                    state.update(metric_valid=True, ground_centers=ground,
                                 ground_bounds=[left_ground[1], right_ground[1]])
            # A continuous transverse white stripe; zebra gaps are not joined.
            stop_rows = []
            for y in range(white.shape[0]):
                lo, hi = max(0, int(left)), min(white.shape[1], int(right)+1)
                bits = white[y, lo:hi] > 0
                edges = np.diff(np.pad(bits.astype(np.int8), (1, 1)))
                lengths = np.flatnonzero(edges == -1)-np.flatnonzero(edges == 1)
                if len(lengths) and max(lengths) >= 0.65*(right-left):
                    stop_rows.append(y)
            bands = np.split(stop_rows, np.flatnonzero(np.diff(stop_rows) > 1)+1) if stop_rows else []
            bands = [band for band in bands if 2 <= len(band) <= white.shape[0]*0.35]
            if bands:
                point = self.ground_point(x0+(left+right)/2, y0+int(bands[-1][-1]))
                if point:
                    state['stop_distance'] = point[0]
        state['offset'] = int(round(reference_x-cx))
        return state

    @staticmethod
    def fit_boundary(points, image_width):
        """Fit x(y), rejecting isolated row outliers without weighting paint width."""
        if len(points) < 3:
            return None, []
        values = np.asarray(points, dtype=float)
        if np.ptp(values[:, 1]) < 6:
            return None, []
        fit = np.polyfit(values[:, 1], values[:, 0], 1)
        residual = np.abs(values[:, 0]-np.polyval(fit, values[:, 1]))
        keep = residual <= max(4.0, image_width*0.025)
        values = values[keep]
        if len(values) < 3 or np.ptp(values[:, 1]) < 6:
            return None, []
        return np.polyfit(values[:, 1], values[:, 0], 1), values.tolist()

    def detect_lane_center(self, mask, reference_x):
        """Track boundary shapes and per-row lane width through partial curves."""
        height, width = mask.shape
        self.boundary_samples = []
        self.lane_confidence = 0.0
        self.lane_heading_error = 0.0
        if self.lane_geometry != mask.shape:
            self.lane_geometry = mask.shape
            self.lane_center = None
            self.lane_width = None
            self.width_profile = None
            self.left_trace = self.right_trace = None
            self.trace_received_at = None
            self.last_distance_to_ref = 0
        if height < 8 or width < 16:
            return None, None, None, 0, 'LOST'

        # Tiny disconnected specks must not become a "short boundary".
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        keep = np.zeros(count, dtype=bool)
        for label in range(1, count):
            keep[label] = (stats[label, cv2.CC_STAT_AREA] >= max(16, width*0.03) and
                           stats[label, cv2.CC_STAT_HEIGHT] >= 6)
        mask = (keep[labels].astype(np.uint8)*255)

        now = time.monotonic()
        fresh_trace = (self.trace_received_at is not None and
                       now-self.trace_received_at <= 1.0)
        nominal_ratio = float(np.clip(
            self.get_parameter('lane_width_ratio').value, 0.25, 0.98))
        nominal_width = self.lane_width if self.lane_width is not None else width*nominal_ratio
        mode = self.controller_status.get('mode') if (
            self.controller_received_at is not None and
            now-self.controller_received_at < 0.5) else None

        def expected_width(y):
            value = (np.polyval(self.width_profile, y) if self.width_profile is not None
                     else nominal_width)
            return float(np.clip(value, width*0.18, width*1.8))

        def expected_edges(y):
            center = reference_x if self.lane_center is None else self.lane_center
            if fresh_trace and self.left_trace is not None and self.right_trace is not None:
                return (float(np.polyval(self.left_trace, y)),
                        float(np.polyval(self.right_trace, y)))
            return center-expected_width(y)/2, center+expected_width(y)/2

        # Sample occupied rows: a short visible fragment must not fall between a
        # fixed set of scan rows. Keep at most 40 rows for bounded processing.
        occupied = np.flatnonzero(np.count_nonzero(mask, axis=1) >= 2)
        occupied = occupied[(occupied >= height//10) & (occupied < height*19//20)]
        if len(occupied) > 40:
            occupied = occupied[np.linspace(0, len(occupied)-1, 40, dtype=int)]
        paired, singles = [], {'left': [], 'right': []}
        for y in occupied:
            edges = np.diff(np.pad((mask[y] > 0).astype(np.int8), (1, 1)))
            starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
            centers = [(start+end-1)/2.0 for start, end in zip(starts, ends)
                       if 2 <= end-start <= max(3, width*0.18)]
            predicted_left, predicted_right = expected_edges(y)
            predicted_center = (predicted_left+predicted_right)/2
            if mode in ('branch_a', 'branch_b'):
                predicted_center = reference_x+(-0.18 if mode == 'branch_a' else 0.18)*width
            candidates = []
            for i, left in enumerate(centers):
                for right in centers[i+1:]:
                    if width*0.18 <= right-left <= width*0.98:
                        score = (abs((left+right)/2-predicted_center) +
                                 0.25*abs(right-left-expected_width(y)))
                        candidates.append((score, left, right))
            if candidates:
                _, left, right = min(candidates)
                paired.append((float(left), float(right), int(y)))
            observations = []
            for x in centers:
                for side, predicted in (('left', predicted_left), ('right', predicted_right)):
                    error = abs(x-predicted)
                    if error <= max(width*0.12, expected_width(y)*0.6):
                        observations.append((error, side, x))
            if observations:
                _, side, x = min(observations)
                singles[side].append((float(x), int(y)))

        left_fit, left_points = self.fit_boundary([(l, y) for l, _, y in paired], width)
        right_fit, right_points = self.fit_boundary([(r, y) for _, r, y in paired], width)
        if left_fit is not None and right_fit is not None:
            common = set(p[1] for p in left_points) & set(p[1] for p in right_points)
            valid_pairs = [p for p in paired if p[2] in common]
        else:
            valid_pairs = []
        if len(valid_pairs) >= 3 and np.ptp([p[2] for p in valid_pairs]) >= 6:
            self.boundary_samples = valid_pairs
            y = int(np.median([p[2] for p in valid_pairs]))
            left, right = float(np.polyval(left_fit, y)), float(np.polyval(right_fit, y))
            if right <= left:
                return None, None, None, y, 'LOST - invalid boundary ordering'
            profile = right_fit-left_fit
            if self.width_profile is None:
                self.width_profile = profile
            elif np.ptp([p[2] for p in valid_pairs]) >= max(20, height*0.2):
                self.width_profile = 0.65*self.width_profile+0.35*profile
            self.left_trace, self.right_trace = left_fit, right_fit
            center_fit = (left_fit+right_fit)/2
            self.lane_width = right-left
            span = np.ptp([p[2] for p in valid_pairs])
            self.lane_confidence = float(np.clip(span/(height*0.5), 0.65, 1.0))
            status = 'BOTH BOUNDARIES'
        else:
            choices = []
            for side, points in singles.items():
                fit, points = self.fit_boundary(points, width)
                if fit is not None:
                    # A marking clipped by the image edge can span many rows
                    # while revealing little of the road's direction. Prefer
                    # an unclipped boundary when both have valid fits.
                    edge = width * 0.025
                    clipped = sum(x < edge or x >= width-edge for x, _ in points) / len(points)
                    score = len(points) * (1.0 - 0.8*clipped)
                    choices.append((score, side, fit, points))
            if not choices:
                return None, None, None, height//2, 'LOST - awaiting boundary recovery'
            _, side, fit, points = max(choices, key=lambda candidate: candidate[0])
            y = int(np.median([p[1] for p in points]))
            span = np.ptp([p[1] for p in points])
            previous = self.left_trace if side == 'left' else self.right_trace
            if fresh_trace and previous is not None and span < height*0.2:
                value = float(np.polyval(fit, y))
                slope = 0.75*previous[0]+0.25*fit[0]
                fit = np.array([slope, value-slope*y])
            profile = (self.width_profile if self.width_profile is not None else
                       np.array([0.0, nominal_width]))
            if side == 'left':
                self.left_trace, self.right_trace = fit, fit+profile
                bias = float(np.clip(self.get_parameter('left_only_center_bias').value, 0.0, 0.15))
                center_fit = fit+profile*(0.5-bias)
                left, right = float(np.polyval(fit, y)), None
                status = 'LEFT ONLY - tracked curve centre'
            else:
                self.right_trace, self.left_trace = fit, fit-profile
                center_fit = fit-profile/2
                left, right = None, float(np.polyval(fit, y))
                status = 'RIGHT ONLY - tracked curve centre'
            self.lane_confidence = float(np.clip(span/(height*0.6)*0.6, 0.35, 0.65))

        self.trace_received_at = now
        center = float(np.clip(np.polyval(center_fit, y), 0, width-1))
        near, far = height*0.85, height*0.25
        self.lane_heading_error = float(np.clip(
            (np.polyval(center_fit, near)-np.polyval(center_fit, far)) /
            max(1.0, expected_width(y)), -1, 1))
        self.lane_center = center
        return center, left, right, y, status

    def timer_callback(self):
        if self.sub_flag:     
            if self.debug_sequence.value == 0:
                self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.roi_,'bgr8'))
            elif self.debug_sequence.value == 1:
                self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.mask_yellow,'mono8'))
            else:    
                self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.image_,'bgr8'))
            self.visualization_publisher.publish(
                self.br.cv2_to_imgmsg(self.visualization_image, 'bgr8'))
        if self.show_visualization:
            try:
                if not self.visualization_window_ready:
                    cv2.namedWindow(self.visualization_window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(self.visualization_window_name, 1100, 600)
                    self.visualization_window_ready = True
                if self.sub_flag:
                    display = self.visualization_image
                else:
                    display = np.zeros((480, 800, 3), dtype=np.uint8)
                    cv2.putText(display, 'Waiting for valid camera image...', (40, 220),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                cv2.imshow(self.visualization_window_name, display)
                cv2.waitKey(1)
            except cv2.error as error:
                self.show_visualization = False
                self.get_logger().warning('Visualization window unavailable: ' + str(error))
          
    def image_callback(self, msg):
        try:
            stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
            if stamp is not None and (stamp.sec or stamp.nanosec):
                age = self.get_clock().now().nanoseconds*1e-9-(stamp.sec+stamp.nanosec*1e-9)
                if age > 0.5 or age < -0.1:
                    raise ValueError('Stale camera frame')
            self.process_image(msg)
        except (CvBridgeError, cv2.error, ValueError, TypeError, IndexError):
            state = String()
            state.data = json.dumps(dict(version=1, valid=False, width=640, offset=0,
                                         centers=[], ground_centers=[], reason='invalid_camera_frame'))
            self.lane_publisher.publish(state)
            self.sub_flag = False

    def process_image(self, msg):
        # convert opencv Mat type to image msg type
        self.image_ = self.br.imgmsg_to_cv2(msg, 'bgr8')
        
        # Use both sides of the image, clipping the ROI to the actual frame.
        height, width = self.image_.shape[:2]
        x0 = int(np.clip(self.roi_x_l.value, 0, width - 1))
        y0 = int(np.clip(self.roi_y_l.value, 0, height - 1))
        x1 = self.roi_x_h.value if self.roi_x_h.value > 0 else width
        x1 = int(np.clip(x1, x0 + 1, width))
        y1 = int(np.clip(self.roi_y_h.value, y0 + 1, height))
        self.roi_ = self.image_[y0:y1, x0:x1]

        # Masking the lane
        hls = cv2.cvtColor(self.roi_, cv2.COLOR_BGR2HLS)
        self.mask_yellow = cv2.inRange(hls, self.yellow_lane_low,
                                            self.yellow_lane_high)
        
        reference_x = float(self.reference_distance.value)
        if reference_x <= 0:
            reference_x = width / 2.0 - x0
        reference_x = float(np.clip(reference_x, 0, self.roi_.shape[1] - 1))
        cx, left_x, right_x, cy, status = self.detect_lane_center(
            self.mask_yellow, reference_x)
        lane_state = self.road_state(hls, reference_x, cx, left_x, right_x, cy, x0, y0)
        if lane_state['valid'] and lane_state['lane_count'] == 3:
            cx = min(lane_state['centers'], key=lambda x: abs(x-reference_x))
        detected = cx is not None
        if detected:
            distance_to_ref = int(round(reference_x - cx))
            self.last_distance_to_ref = distance_to_ref
        else: # Tighten the last turn without accumulating correction across frames.
            distance_to_ref = self.last_distance_to_ref
            boost = max(0, self.get_parameter('lost_lane_offset_boost').value)
            if distance_to_ref > 0:
                distance_to_ref += boost
            elif distance_to_ref < 0:
                distance_to_ref -= boost

        # Keep the existing controller topic and steering sign.
        dis = Int32()
        dis.data = distance_to_ref
        self.dis_publisher.publish(dis)
        lane_state['offset'] = distance_to_ref
        state_msg = String()
        state_msg.data = json.dumps(lane_state, allow_nan=False, separators=(',', ':'))
        self.lane_publisher.publish(state_msg)

        # Draw on a copy after detection; visualization never changes the mask or steering.
        overlay = self.image_.copy()
        cv2.rectangle(overlay, (x0, y0), (x1 - 1, y1 - 1), (255, 255, 0), 2)
        reference_pixel = x0 + int(round(reference_x))
        cv2.line(overlay, (reference_pixel, y0), (reference_pixel, y1 - 1), (0, 255, 0), 2)
        cv2.putText(overlay, 'ROI x:%d-%d y:%d-%d' % (x0, x1, y0, y1),
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(overlay, 'ROI', (x0 + 8, min(y0 + 25, y1 - 1)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
        for center in lane_state['centers']:
            cv2.circle(overlay, (x0+int(center), y0+cy), 5, (0, 255, 255), -1)
        if (self.controller_received_at is not None and
                time.monotonic()-self.controller_received_at < 0.5):
            target = self.controller_status.get('target_lane')
            if isinstance(target, int) and 0 <= target < len(lane_state['centers']):
                target_x = x0+int(lane_state['centers'][target])
                cv2.circle(overlay, (target_x, y0+cy), 12, (0, 165, 255), 2)
                cv2.putText(overlay, 'TARGET', (max(x0, target_x-35), y0+cy-18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        if detected:
            point_y = y0 + cy
            for boundary_x in (left_x, right_x):
                if boundary_x is not None:
                    cv2.circle(overlay, (x0 + int(round(boundary_x)), point_y),
                               7, (255, 0, 0), -1)
            if left_x is not None and right_x is not None:
                cv2.line(overlay, (x0 + int(round(left_x)), point_y),
                         (x0 + int(round(right_x)), point_y), (0, 255, 255), 2)
            cv2.circle(overlay, (x0 + int(round(cx)), point_y), 7, (0, 0, 255), -1)
            self.image_ = overlay.copy()
        color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(overlay, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.putText(overlay, 'Steering offset: %d px' % distance_to_ref,
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        mask_panel = np.zeros_like(overlay)
        mask_roi = mask_panel[y0:y1, x0:x1]
        mask_roi[:] = cv2.cvtColor(self.mask_yellow, cv2.COLOR_GRAY2BGR)
        cv2.putText(mask_panel, 'Detected lane pixels (ROI)', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        fresh = (self.yolo_received_at is not None and
                 time.monotonic() - self.yolo_received_at < 2.0)
        boxes = self.yolo_boxes if fresh else []
        for box in boxes:
            cv2.rectangle(overlay, (box.xmin, box.ymin),
                          (box.xmax, box.ymax), (255, 0, 255), 2)
            cv2.putText(overlay, '%s %.0f%%' % (box.class_name, box.probability * 100),
                        (max(0, box.xmin), max(85, box.ymin - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        lines = ['YOLO: latest detections' if fresh else 'YOLO: waiting / stale']
        control_fresh = (self.controller_received_at is not None and
                         time.monotonic()-self.controller_received_at < 0.5)
        control = self.controller_status if control_fresh else {}
        lines += ['Mission: %s / %s' % (control.get('mode', 'waiting'), control.get('phase', '-')),
                  'Control: %s' % str(control.get('reason', 'controller unavailable'))[:90],
                  'Speed: %.2f m/s | lanes: %d | metric calibration: %s' % (
                      control.get('speed', 0.0), lane_state['lane_count'], lane_state['metric_valid'])]
        lines += ['%s: %.0f%%' % (box.class_name, box.probability * 100)
                  for box in boxes]
        if fresh and not boxes:
            lines.append('No objects detected')
        # A separate footer keeps class names clear of the lane mask.
        footer = np.zeros((30 * len(lines) + 10, overlay.shape[1] * 2, 3), dtype=np.uint8)
        for index, line in enumerate(lines):
            cv2.putText(footer, line, (10, 25 + index * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        self.visualization_image = np.hstack((overlay, mask_panel))
        self.visualization_image = np.vstack((self.visualization_image, footer))

        self.sub_flag = True

    def destroy_node(self):
        if self.visualization_window_ready:
            try:
                cv2.destroyWindow(self.visualization_window_name)
            except cv2.error:
                pass  # The user may already have closed the preview window.
        return super().destroy_node()


"""Non-DL controller; the safety node may additionally publish emergency zeros.

mission_command (std_msgs/String, JSON) accepts mode=follow/slow/guide/
obstacles/branch_a/branch_b/cones/pedestrian/traffic/parking/hold.
YOLO trigger applies a 10-second speed cap without changing lane control.
Other signs and signals use mission_command. {"signal":"green"} releases a traffic stop.
Parking requires a measured path and parking_box in the entry robot frame.
Geometry/calibration gates must be verified on the real robot before enabling
metric manoeuvres. Never use elapsed time alone as a parking/stop distance.
"""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from dl_ros2_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int32, String
from limo_msgs.msg import LimoStatus


def finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Expected a finite number')
    return float(value)


def local_point(pose, x, y):
    dx, dy = x-pose[0], y-pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c*dx+s*dy, -s*dx+c*dy


def world_point(pose, x, y):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0]+c*x-s*y, pose[1]+s*x+c*y


class LimoControl(Node):
    def __init__(self):
        super().__init__('limo_control')
        defaults = dict(enabled=True, default_speed=0.3, max_speed=0.35,
                        slow_speed=0.15, uncertain_speed=0.10, parking_speed=0.08,
                        p_gain=0.003, d_gain=0.0002, heading_gain=0.25,
                        wheelbase=0.2, max_steering_angle=0.35,
                        robot_front=0.18, robot_rear=0.18, robot_width=0.22,
                        rear_axle_offset=0.1, safety_margin=0.04,
                        acceleration=0.25, deceleration=0.4, reaction_time=0.25,
                        curvature_rate=2.0, sensor_timeout=0.5, vehicle_status_timeout=2.5,
                        lane_hold_time=0.8,
                        lane_hold_distance=0.1, lane_hold_speed=0.08,
                        lookahead=0.8, avoidance_horizon=1.2, lane_change_distance=1.2,
                        guide_distance=1.677, stop_tolerance=0.02,
                        stop_search_distance=1.5, stopped_speed=0.005,
                        stop_confirm_time=0.2,
                        pedestrian_wait=3.2, mission_timeout=15.0, lap_timeout=600.0,
                        geometry_verified=False, auto_obstacle_mode=True,
                        require_path_coverage_for_lane_follow=True,
                        allow_sparse_scan_crawl=False, sparse_scan_speed=0.05,
                        command_control_mode=1, command_mode_settle=0.2,
                        require_ackermann=True, base_frame='base_link',
                        odom_frame='odom', initial_mode='follow')
        defaults.update(target_class='slow_sign', min_box_area=3000.0,
                        min_confidence=0.5, target_speed=0.05)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if (not math.isfinite(self.cfg['min_box_area']) or self.cfg['min_box_area'] < 0 or
                not math.isfinite(self.cfg['target_speed']) or self.cfg['target_speed'] < 0 or
                not 0 <= self.cfg['min_confidence'] <= 1):
            raise ValueError('Invalid speed trigger parameters')
        self.speed_trigger_until = 0.0
        self.speed_trigger_visible = False
        for k in ('default_speed', 'max_speed', 'slow_speed', 'uncertain_speed',
                  'parking_speed', 'wheelbase', 'max_steering_angle', 'robot_front',
                  'robot_rear', 'robot_width', 'acceleration', 'deceleration',
                  'sensor_timeout', 'vehicle_status_timeout', 'lookahead', 'avoidance_horizon', 'lap_timeout',
                  'curvature_rate', 'lane_change_distance', 'guide_distance',
                  'stop_tolerance', 'stop_search_distance', 'stopped_speed',
                  'stop_confirm_time', 'reaction_time'):
            if not math.isfinite(self.cfg[k]) or self.cfg[k] <= 0:
                raise ValueError('Invalid control parameter: '+k)
        if not 0 < self.cfg['max_steering_angle'] < 0.5:
            raise ValueError('Steering limit must respect the Ackermann mechanism')
        if (not 0 < self.cfg['p_gain'] <= 0.02 or self.cfg['d_gain'] < 0 or
                self.cfg['heading_gain'] < 0):
            raise ValueError('Invalid steering gains')
        if self.cfg['initial_mode'] not in ('follow', 'hold'):
            raise ValueError('Start in follow/hold; arm other missions after sensors are ready')
        if not 3 <= self.cfg['pedestrian_wait'] < self.cfg['mission_timeout'] <= 15:
            raise ValueError('Invalid mission waiting times')
        if (not 0 <= self.cfg['rear_axle_offset'] <= self.cfg['robot_rear'] or
                not 0 <= self.cfg['safety_margin'] <= 0.2 or
                not 0 <= self.cfg['lane_hold_time'] <= 1.0 or
                not 0 < self.cfg['lane_hold_distance'] <= 0.15 or
                not 0 < self.cfg['lane_hold_speed'] <= 0.1):
            raise ValueError('Invalid geometry or lane hold interval')
        if not 0 < self.cfg['sparse_scan_speed'] <= 0.05:
            raise ValueError('Sparse-return test crawl must not exceed 0.05 m/s')
        if (type(self.cfg['command_control_mode']) is not int or
                not 0 <= self.cfg['command_control_mode'] <= 255 or
                not 0 <= self.cfg['command_mode_settle'] <= 1.0):
            raise ValueError('Invalid command-control mode configuration')
        self.max_curvature = math.tan(self.cfg['max_steering_angle'])/self.cfg['wheelbase']
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'mission_status', 10)
        self.subscriptions_ = [
            self.create_subscription(Bool, 'e_stop', self.e_stop_callback, 10),
            self.create_subscription(String, 'lane_state', self.lane_callback, 10),
            self.create_subscription(String, 'obstacle_state', self.obstacle_callback, 10),
            self.create_subscription(String, 'mission_command', self.command_callback, 10),
            self.create_subscription(BoundingBoxes, '/yolo_bounding_boxes', self.yolo_callback, 10),
            self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile_sensor_data),
            self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile_sensor_data),
            self.create_subscription(LimoStatus, 'limo_status', self.vehicle_callback, 10),
            # Compatibility only: distance_y alone never authorizes motion.
            self.create_subscription(Int32, 'distance_y', self.distance_callback, 10),
        ]
        self.lane = {}
        self.obstacles = {}
        self.received = {}
        self.e_stop_flag = True
        self.pose = None
        self.odom_speed = 0.0
        self.travel = 0.0
        self.vehicle_ok = False
        self.vehicle_control_mode = None
        self.command_ready_since = None
        self.odom_fault = False
        self.last_valid_lane = None
        self.last_valid_lane_travel = 0.0
        self.last_tracking_curvature = 0.0
        self.filtered_error = 0.0
        self.previous_error = 0.0
        self.last_speed = 0.0
        self.last_curvature = 0.0
        self.last_tick = time.monotonic()
        self.started_at = None
        self.mode = 'hold'
        self.phase = 'idle'
        self.reason = 'initializing'
        self.hold_reason = 'initial_hold' if self.cfg['initial_mode'] == 'hold' else ''
        self.entered_at = self.last_tick
        self.phase_travel = 0.0
        self.phase_heading = 0.0
        self.blocked_since = None
        self.stop_target = None
        self.stopped_at = None
        self.mission_distance = None
        self.stationary_since = None
        self.signal = ('red', 0.0)
        self.slow_until = 0.0
        self.slow_sign_used = False
        self.slow_sign_until = None
        self.target_lane = None
        self.multi_frames = 0
        self.command_ids = deque(maxlen=100)
        self.parking_path = []
        self.parking_box = None
        self.parking_origin = None
        self.parking_index = 0
        self.progress_at, self.progress_travel = self.last_tick, self.travel
        self._enter(self.cfg['initial_mode'], self.last_tick)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def _enter(self, mode, now):
        self.slow_sign_until = None
        if mode != 'hold':
            self.hold_reason = ''
        self.mode, self.phase = mode, 'running'
        self.entered_at, self.phase_travel = now, self.travel
        self.phase_heading = self.pose[2] if self.pose else 0.0
        self.blocked_since = None
        self.target_lane = None
        self.stop_target = None
        self.stopped_at = None
        self.mission_distance = None
        self.stationary_since = None
        self.last_tracking_curvature = self.last_curvature
        self.progress_at, self.progress_travel = now, self.travel
        if mode in ('pedestrian', 'traffic'):
            self.phase = 'approach_stop'
        elif mode == 'hold':
            self.phase = 'stopped'

    def enter_hold(self, reason, now):
        self._enter('hold', now)
        self.hold_reason = reason
        self.get_logger().warning('Automatic hold: '+reason)

    def distance_callback(self, msg):
        pass  # Rich lane_state validity and freshness are mandatory.

    def e_stop_callback(self, msg):
        self.e_stop_flag = bool(msg.data)
        self.received['e_stop'] = time.monotonic()
        if self.e_stop_flag:
            self.publish_motion(0.0, 0.0)

    def lane_callback(self, msg):
        now = time.monotonic()
        try:
            lane = json.loads(msg.data)
            if not isinstance(lane, dict) or lane.get('version') != 1:
                raise ValueError('Invalid lane schema')
            width = finite_number(lane['width'])
            error = finite_number(lane['offset'])
            if width < 16 or abs(error) > width:
                raise ValueError('Invalid lane geometry')
            if type(lane.get('valid')) is not bool or lane.get('lane_count', 0) not in (0, 1, 3):
                raise ValueError('Invalid lane status')
            if (not 0 <= finite_number(lane.get('confidence', 0.0)) <= 1 or
                    abs(finite_number(lane.get('heading_error', 0.0))) > 1):
                raise ValueError('Invalid lane confidence/heading')
            for key in ('centers', 'ground_centers'):
                values = lane.get(key, [])
                if not isinstance(values, list) or len(values) > 3:
                    raise ValueError('Invalid lane candidates')
                if key == 'centers':
                    for value in values:
                        if not 0 <= finite_number(value) < width:
                            raise ValueError('Lane centre outside image')
                else:
                    for point in values:
                        if len(point) != 2 or not 0 < finite_number(point[0]) < 5 or abs(finite_number(point[1])) > 3:
                            raise ValueError('Invalid metric lane centre')
            self.lane = lane
            self.received['lane'] = now
            if lane.get('valid') is True:
                self.last_valid_lane = now
                self.last_valid_lane_travel = self.travel
            if lane.get('lane_count') == 3 and lane.get('valid') is True:
                self.multi_frames = min(10, self.multi_frames+1)
            else:
                self.multi_frames = 0
        except (ValueError, KeyError, TypeError, OverflowError):
            self.lane = {}
            self.received.pop('lane', None)

    def obstacle_callback(self, msg):
        try:
            state = json.loads(msg.data)
            if state.get('valid') is not True or state.get('frame') != self.cfg['base_frame']:
                raise ValueError('Invalid obstacle frame/data')
            points, coverage = state['points'], state['coverage']
            if (not 0 < finite_number(state['range_limit']) <= 10 or
                    len(state['sensor_origin']) != 2 or
                    any(abs(finite_number(v)) > 3 for v in state['sensor_origin'])):
                raise ValueError('Invalid scan limits/origin')
            if len(points) > 10000 or len(coverage) != 36:
                raise ValueError('Invalid obstacle packet')
            for point in points:
                if len(point) != 2 or any(abs(finite_number(v)) > 10 for v in point):
                    raise ValueError('Invalid obstacle point')
            for value in coverage:
                if not 0 <= finite_number(value) <= 1:
                    raise ValueError('Invalid coverage')
            self.obstacles = state
            self.obstacle_error = ''
            self.received['obstacles'] = time.monotonic()
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as error:
            self.obstacles = {}
            self.received.pop('obstacles', None)
            try:
                self.obstacle_error = str(state.get('reason', str(error)))[:180]
            except (AttributeError, UnboundLocalError):
                self.obstacle_error = str(error)

    def odom_callback(self, msg):
        now = time.monotonic()
        try:
            if msg.header.frame_id != self.cfg['odom_frame'] or msg.child_frame_id != self.cfg['base_frame']:
                raise ValueError('Odometry frames do not match configuration')
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if stamp and (age > self.cfg['sensor_timeout'] or age < -0.1):
                raise ValueError('Stale odometry')
            pose = (finite_number(msg.pose.pose.position.x),
                    finite_number(msg.pose.pose.position.y), yaw_of(msg.pose.pose.orientation))
            velocity = finite_number(msg.twist.twist.linear.x)
            if self.pose is not None:
                dt = now-self.received.get('odom', now)
                distance = math.hypot(pose[0]-self.pose[0], pose[1]-self.pose[1])
                if distance > self.cfg['max_speed']*max(dt, 0.02)*4+0.1:
                    self.odom_fault = True
                else:
                    self.travel += distance
            self.pose, self.odom_speed = pose, velocity
            self.received['odom'] = now
        except (ValueError, TypeError):
            self.received.pop('odom', None)

    def vehicle_callback(self, msg):
        now = time.monotonic()
        self.vehicle_ok = (msg.error_code == 0 and
                           (not self.cfg['require_ackermann'] or msg.motion_mode == 1))
        self.vehicle_control_mode = int(msg.control_mode)
        if self.vehicle_control_mode == self.cfg['command_control_mode']:
            if self.command_ready_since is None:
                self.command_ready_since = now
        else:
            self.command_ready_since = None
        self.received['vehicle'] = now
        if not self.vehicle_ok or self.command_ready_since is None:
            self.publish_motion(0.0, 0.0)
            self.last_tracking_curvature = 0.0

    def imu_callback(self, msg):
        pitch_rate = msg.angular_velocity.y
        if math.isfinite(pitch_rate) and abs(pitch_rate) > 0.25:
            # This is a bump fallback, not a replacement for slowing BEFORE a sign.
            self.slow_until = time.monotonic()+2.0

    def yolo_callback(self, msg):
        now = time.monotonic()
        visible = any(
            box.class_name == self.cfg['target_class']
            and math.isfinite(box.probability)
            and box.probability >= self.cfg['min_confidence']
            and box.xmax > box.xmin and box.ymax > box.ymin
            and (box.xmax-box.xmin)*(box.ymax-box.ymin) >= self.cfg['min_box_area']
            for box in msg.bounding_boxes)
        # A rising edge starts one fixed interval. Repeated detections never
        # extend it; a new interval requires disappearance and reappearance.
        if (visible and not self.speed_trigger_visible and self.mode == 'follow'
                and now >= self.speed_trigger_until):
            self.speed_trigger_until = now + 10.0
            self.get_logger().info('Speed trigger: reducing speed for 10 seconds')
        self.speed_trigger_visible = visible

    def command_callback(self, msg):
        now = time.monotonic()
        try:
            command = json.loads(msg.data)
            if not isinstance(command, dict):
                raise ValueError('Expected command object')
            if 'signal' in command:
                if command['signal'] not in ('red', 'yellow', 'green'):
                    raise ValueError('Invalid traffic signal')
                self.signal = (command['signal'], now)
                return
            mode = command['mode']
            if mode not in ('follow', 'slow', 'guide', 'obstacles', 'branch_a',
                            'branch_b', 'cones', 'pedestrian', 'traffic', 'parking', 'hold'):
                raise ValueError('Unknown mission')
            identifier = command.get('id')
            if identifier is not None:
                if not isinstance(identifier, str) or len(identifier) > 100:
                    raise ValueError('Invalid command id')
                if identifier in self.command_ids:
                    return
            elif mode == self.mode and not any(command.get(k) is True for k in
                                               ('reset_odom_fault', 'reset_lap')):
                return  # Repeated detections must not restart waits or manoeuvres.
            parking = None
            if mode == 'parking':
                if not self.cfg['geometry_verified'] or not self.fresh('odom', now):
                    raise ValueError('Parking requires verified geometry and odometry')
                points, box = command['path'], command['parking_box']
                if not isinstance(points, list) or not 2 <= len(points) <= 100 or len(box) != 4:
                    raise ValueError('Provide measured parking path and rectangle')
                box = [finite_number(v) for v in box]
                if box[1]-box[0] < self.cfg['robot_front']+self.cfg['robot_rear'] or box[3]-box[2] < self.cfg['robot_width']:
                    raise ValueError('Parking box too small')
                path = []
                for point in points:
                    if len(point) != 4:
                        raise ValueError('Path points are [x,y,yaw,signed_speed]')
                    x, y, yaw, speed = [finite_number(v) for v in point]
                    if math.hypot(x, y) > 5 or abs(speed) > self.cfg['parking_speed'] or abs(speed) < 0.02:
                        raise ValueError('Parking waypoint/speed outside limits')
                    wx, wy = world_point(self.pose, x, y)
                    path.append((wx, wy, wrap_angle(yaw+self.pose[2]), speed))
                parking = (path, box, self.pose)
            if mode == 'guide' and not self.fresh('odom', now):
                raise ValueError('Heading hold requires odometry')
            distance = command.get('distance')
            if distance is not None:
                distance = finite_number(distance)
                if not 0 < distance <= 5:
                    raise ValueError('Distance must be in metres, 0 < d <= 5')
                if mode in ('pedestrian', 'traffic') and not self.cfg['geometry_verified']:
                    raise ValueError('Rear-axle stopping needs verified geometry')
            self._enter(mode, now)
            if identifier is not None:
                self.command_ids.append(identifier)
            self.mission_distance = distance
            if mode == 'hold':
                self.hold_reason = 'operator_hold'
                self.publish_motion(0.0, 0.0)
            if parking:
                self.parking_path, self.parking_box, self.parking_origin = parking
                self.parking_index = 0
            if mode in ('pedestrian', 'traffic') and distance is not None:
                self.stop_target = self.travel+distance
            if command.get('reset_odom_fault') is True and mode == 'hold':
                self.odom_fault = False
            if command.get('reset_lap') is True and mode == 'hold':
                self.started_at = None
                self.slow_sign_used = False
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            self.get_logger().warning('Mission command rejected: '+str(error))

    def fresh(self, name, now):
        timeout = self.cfg['vehicle_status_timeout'] if name == 'vehicle' else self.cfg['sensor_timeout']
        return name in self.received and 0 <= now-self.received[name] <= timeout

    def steering_curvature(self, now, dt):
        width = float(self.lane['width'])
        error = float(self.lane['offset'])*640.0/width
        self.filtered_error = 0.65*self.filtered_error+0.35*error
        derivative = clamp((self.filtered_error-self.previous_error)/dt, -500.0, 500.0)
        self.previous_error = self.filtered_error
        heading = clamp(float(self.lane.get('heading_error', 0.0)), -1.0, 1.0)
        angle = clamp(self.cfg['p_gain']*self.filtered_error+self.cfg['d_gain']*derivative+
                      self.cfg['heading_gain']*heading,
                      -self.cfg['max_steering_angle'], self.cfg['max_steering_angle'])
        return math.tan(angle)/self.cfg['wheelbase']

    def curve_clear(self, curvature, horizon, direction=1, road_bounds=None, check_coverage=True):
        """Check the swept rectangle, including overhangs; unknown space is blocked."""
        for index in range(21):
            distance = direction*horizon*index/20
            angle = curvature*distance
            if abs(curvature) < 1e-6:
                x, y = distance, 0.0
            else:
                offset = self.cfg['rear_axle_offset']
                x = math.sin(angle)/curvature+offset*(math.cos(angle)-1)
                y = (1-math.cos(angle))/curvature+offset*math.sin(angle)
            if not self.footprint_clear(x, y, angle, direction, road_bounds, check_coverage and index > 2):
                return False
        return True

    def footprint_clear(self, x, y, angle, direction=1, road_bounds=None, check_coverage=True):
        half = self.cfg['robot_width']/2+self.cfg['safety_margin']
        front = self.cfg['robot_front']+self.cfg['safety_margin']
        rear = self.cfg['robot_rear']+self.cfg['safety_margin']
        c, s = math.cos(angle), math.sin(angle)
        if road_bounds is not None:
            for dx, dy in ((front, half), (front, -half), (-rear, half), (-rear, -half)):
                py = y+s*dx+c*dy
                if py > road_bounds[0] or py < road_bounds[1]:
                    return False
        if check_coverage:
            origin = self.obstacles.get('sensor_origin', [0.0, 0.0])
            coverage = self.obstacles.get('coverage', [0.0]*36)
            edge = front if direction > 0 else -rear
            for lateral in (-half, 0.0, half):
                px, py = x+c*edge-s*lateral, y+s*edge+c*lateral
                dx, dy = px-origin[0], py-origin[1]
                ray = min(35, int((math.atan2(dy, dx)+math.pi)*36/(2*math.pi)))
                if math.hypot(dx, dy) > self.obstacles.get('range_limit', 0.0) or coverage[ray] < 0.6:
                    return False
        for ox, oy in self.obstacles.get('points', []):
            dx, dy = ox-x, oy-y
            if -rear <= c*dx+s*dy <= front and abs(-s*dx+c*dy) <= half:
                return False
        return True

    def change_path(self, lateral, length, bounds):
        """A quintic S-curve settles parallel to the adjacent straight lane."""
        for index in range(31):
            u = index/30
            x = u*length
            y = lateral*(10*u**3-15*u**4+6*u**5)
            slope = lateral*(30*u*u-60*u**3+30*u**4)/length
            second = lateral*(60*u-180*u*u+120*u**3)/(length*length)
            curvature = second/(1+slope*slope)**1.5
            if abs(curvature) > self.max_curvature:
                return False
            if not self.footprint_clear(x, y, math.atan(slope), road_bounds=bounds,
                                        check_coverage=index > 2):
                return False
        return True

    def lane_change_curvature(self, baseline):
        if (not self.cfg['geometry_verified'] or self.lane.get('metric_valid') is not True or
                self.lane.get('lane_count') != 3):
            return None, 'lane_change_needs_calibration'
        centers = self.lane.get('ground_centers', [])
        bounds = self.lane.get('ground_bounds')
        if len(centers) != 3 or not isinstance(bounds, list) or len(bounds) != 2:
            return None, 'three_lane_geometry_unavailable'
        bounds = [finite_number(v) for v in bounds]
        if abs(float(self.lane.get('heading_error', 0.0))) > 0.2:
            return None, 'three_lane_change_requires_straight_corridor'
        current = min(range(3), key=lambda i: abs(centers[i][1]))
        # Hold the selected lane until it has been reached; avoid frame-to-frame toggling.
        choices = [self.target_lane] if self.target_lane is not None else [current, current-1, current+1]
        candidates = []
        for i in choices:
            if i is None or not 0 <= i < 3:
                continue
            _, y = centers[i]
            length = max(self.cfg['lane_change_distance'], abs(y)*2.5)
            look = min(self.cfg['lookahead'], length)
            u = look/length
            target_y = y*(10*u**3-15*u**4+6*u**5)
            x = look+self.cfg['rear_axle_offset']
            k = 2*target_y/(x*x+target_y*target_y)
            if abs(k) > self.max_curvature:
                continue
            if self.change_path(y, length, bounds):
                candidates.append((abs(i-current)*0.5+abs(k-baseline)*0.1, i, k))
        if not candidates:
            self.target_lane = None
            return None, 'no_clear_lane_change_path'
        _, selected, curvature = min(candidates)
        self.target_lane = selected
        if selected == current and abs(centers[selected][1]) < 0.06:
            self.target_lane = None
        return curvature, 'tracking_lane_%d' % selected

    def cone_curvature(self):
        # This mode is explicitly armed: objects bounding the gap are assumed cones.
        points = self.obstacles.get('points', [])
        near = [(x, y) for x, y in points if 0.3 < x < 1.5 and abs(y) < 1.5]
        left = [p for p in near if p[1] > 0.08]
        right = [p for p in near if p[1] < -0.08]
        if not left or not right:
            return None, 'cone_corridor_not_observed'
        lp = min(left, key=lambda p: p[1])
        rp = max(right, key=lambda p: p[1])
        bounds = (lp[1], rp[1])
        if bounds[0]-bounds[1] < self.cfg['robot_width']+2*self.cfg['safety_margin']:
            return None, 'cone_gap_too_narrow'
        x, y = max(0.5, (lp[0]+rp[0])/2), (lp[1]+rp[1])/2
        desired = 2*y/((x+self.cfg['rear_axle_offset'])**2+y*y)
        candidates = [desired]+[self.max_curvature*i/8 for i in range(-8, 9)]
        clear = [k for k in candidates if abs(k) <= self.max_curvature and
                 self.curve_clear(k, self.cfg['avoidance_horizon'], road_bounds=bounds)]
        if not clear:
            return None, 'cone_path_blocked'
        return min(clear, key=lambda k: abs(k-desired)+0.2*abs(k-self.last_curvature)), 'cone_gap'

    def parking_control(self):
        if self.parking_origin is None or not self.parking_path:
            return 0.0, 0.0, 'parking_path_missing'
        goal = self.parking_path[self.parking_index]
        x, y = local_point(self.pose, goal[0], goal[1])
        distance = math.hypot(x, y)
        if distance <= 0.03:
            if self.parking_index < len(self.parking_path)-1:
                self.parking_index += 1
                return self.parking_control()
            yaw_error = abs(wrap_angle(goal[2]-self.pose[2]))
            inside = True
            for dx in (-self.cfg['robot_rear'], self.cfg['robot_front']):
                for dy in (-self.cfg['robot_width']/2, self.cfg['robot_width']/2):
                    wx, wy = world_point(self.pose, dx, dy)
                    bx, by = local_point(self.parking_origin, wx, wy)
                    box = self.parking_box
                    inside = inside and box[0]+0.005 < bx < box[1]-0.005 and box[2]+0.005 < by < box[3]-0.005
            if inside and yaw_error < 0.12 and abs(self.odom_speed) <= self.cfg['stopped_speed']:
                self.phase = 'complete'
                return 0.0, 0.0, 'parked_four_corners_inside'
            return 0.0, 0.0, 'parking_alignment_or_boundary_error'
        direction = 1 if goal[3] > 0 else -1
        if direction*x < -0.03:
            return 0.0, 0.0, 'parking_waypoint_behind_travel_direction'
        x += self.cfg['rear_axle_offset']
        k = 2*y/max(0.01, x*x+y*y)
        if abs(k) > self.max_curvature:
            return 0.0, 0.0, 'parking_path_exceeds_turning_radius'
        speed = direction*min(abs(goal[3]), math.sqrt(2*self.cfg['deceleration']*distance))
        if not self.curve_clear(k, min(0.6, distance+0.1), direction):
            return 0.0, 0.0, 'parking_path_blocked_or_unobserved'
        return speed, k, 'parking_waypoint_%d' % self.parking_index

    def compute_control(self, now, dt):
        if not self.get_parameter('enabled').value:
            return 0.0, 0.0, 'disabled_by_parameter'
        if self.mode == 'hold':
            return 0.0, 0.0, self.hold_reason or 'held'
        if not self.fresh('vehicle', now):
            return 0.0, 0.0, 'vehicle_stale_or_missing'
        if self.vehicle_control_mode != self.cfg['command_control_mode']:
            # RC/manual ownership is an intentional pause, not a failed mission.
            self.blocked_since = None
            self.progress_at, self.progress_travel = now, self.travel
            return 0.0, 0.0, 'waiting_for_command_mode(%s)' % self.vehicle_control_mode
        if (self.command_ready_since is None or
                now-self.command_ready_since < self.cfg['command_mode_settle']):
            return 0.0, 0.0, 'command_mode_settling'
        for sensor in ('e_stop', 'obstacles', 'odom', 'vehicle', 'lane'):
            if not self.fresh(sensor, now):
                if sensor == 'obstacles' and getattr(self, 'obstacle_error', ''):
                    return 0.0, 0.0, 'obstacles_invalid: '+self.obstacle_error
                return 0.0, 0.0, sensor+'_stale_or_missing'
        if self.e_stop_flag or not self.vehicle_ok or self.odom_fault:
            return 0.0, 0.0, 'emergency_stop_or_vehicle_fault'
        if self.started_at is not None and now-self.started_at >= self.cfg['lap_timeout']:
            self.enter_hold('lap_time_limit', now)
            return 0.0, 0.0, 'lap_time_limit'
        travelled = self.travel-self.phase_travel
        distance = getattr(self, 'mission_distance', None)
        if (distance is not None and travelled >= distance and
                self.mode in ('slow', 'branch_a', 'branch_b', 'cones', 'obstacles')):
            self._enter('follow', now)
        if self.phase == 'complete':
            return 0.0, 0.0, 'mission_complete'
        if self.mode == 'parking':
            return self.parking_control()
        if self.mode in ('pedestrian', 'traffic'):
            if self.phase == 'waiting':
                if abs(self.odom_speed) > self.cfg['stopped_speed']:
                    self.phase = 'approach_stop'
                    self.stopped_at = self.stationary_since = None
                    return 0.0, 0.0, 'motion_during_required_stop'
                if self.stopped_at is None:
                    return 0.0, 0.0, 'stop_not_confirmed'
                waiting = now-self.stopped_at
                if waiting >= self.cfg['mission_timeout']:
                    self.phase = 'failed'
                    return 0.0, 0.0, 'mission_wait_timeout'
                release = (waiting >= self.cfg['pedestrian_wait'] if self.mode == 'pedestrian'
                           else self.signal[0] == 'green' and now-self.signal[1] < 1.0)
                if not release:
                    return 0.0, 0.0, 'waiting_'+self.mode
                self._enter('follow', now)
                return self.compute_control(now, dt)
            elif self.phase == 'failed':
                return 0.0, 0.0, 'mission_failed'
            if self.stop_target is None:
                distance = self.lane.get('stop_distance')
                if not self.cfg['geometry_verified'] or self.lane.get('metric_valid') is not True:
                    return 0.0, 0.0, 'stop_line_metric_calibration_or_detection_required'
                if distance is not None:
                    self.stop_target = self.travel+finite_number(distance)+self.cfg['rear_axle_offset']
                elif travelled >= self.cfg['stop_search_distance']:
                    self.phase = 'failed'
                    return 0.0, 0.0, 'stop_line_not_found_within_search_distance'
            if self.stop_target is not None:
                remaining = self.stop_target-self.travel
                if remaining <= self.cfg['stop_tolerance']:
                    if remaining < -0.05:
                        self.phase = 'failed'
                        return 0.0, 0.0, 'rear_axle_stop_overshoot'
                    if abs(self.odom_speed) <= self.cfg['stopped_speed']:
                        if self.stationary_since is None:
                            self.stationary_since = now
                        if now-self.stationary_since >= self.cfg['stop_confirm_time']:
                            self.phase, self.stopped_at = 'waiting', now
                    else:
                        self.stationary_since = None
                    return 0.0, 0.0, 'stopping_at_rear_axle_target'
        if self.mode == 'guide':
            length = getattr(self, 'mission_distance', None) or self.cfg['guide_distance']
            if travelled >= length:
                self._enter('follow', now)
            else:
                error = wrap_angle(self.phase_heading-self.pose[2])
                return min(self.cfg['slow_speed'], self.cfg['default_speed']/2, 0.2), clamp(
                    2.0*error, -self.max_curvature, self.max_curvature), 'holding_entry_heading'
        if self.mode == 'cones':
            curvature, reason = self.cone_curvature()
            return (0.0, 0.0, reason) if curvature is None else (
                min(self.cfg['slow_speed'], self.cfg['default_speed']/2, 0.2), curvature, reason)
        if self.lane.get('valid') is not True:
            if self.lane.get('reason') == 'invalid_camera_frame':
                return 0.0, 0.0, 'camera_frame_invalid'
            if (self.last_valid_lane is None or
                    (self.command_ready_since is not None and
                     self.last_valid_lane < self.command_ready_since) or
                    now-self.last_valid_lane > self.cfg['lane_hold_time'] or
                    self.travel-self.last_valid_lane_travel >= self.cfg['lane_hold_distance']):
                return 0.0, 0.0, 'lane_lost'
            return (min(self.cfg['lane_hold_speed'], self.cfg['default_speed']),
                    self.last_tracking_curvature, 'brief_lane_loss_curve_hold')
        baseline = self.steering_curvature(now, dt)
        speed = min(self.cfg['default_speed'], self.cfg['max_speed'])
        speed /= 1+0.9*abs(baseline)
        if float(self.lane.get('confidence', 0.0)) < 0.8:
            speed = min(speed, self.cfg['uncertain_speed'])
        if self.slow_sign_until is not None:
            speed = min(speed, 0.2)
        elif self.mode in ('slow', 'branch_a', 'branch_b', 'pedestrian', 'traffic') or now < self.slow_until:
            speed = min(speed, self.cfg['slow_speed'], self.cfg['default_speed']/2, 0.2)
        if self.mode in ('pedestrian', 'traffic') and self.stop_target is None:
            speed = min(speed, 0.08)
        if self.stop_target is not None:
            remaining = max(0.0, self.stop_target-self.travel-self.cfg['stop_tolerance'])
            speed = min(speed, math.sqrt(2*self.cfg['deceleration']*remaining))
        if self.mode == 'follow' and self.cfg['auto_obstacle_mode'] and self.multi_frames >= 4:
            self._enter('obstacles', now)
        if self.mode == 'obstacles':
            curvature, reason = self.lane_change_curvature(baseline)
            if curvature is None:
                # Until calibrated, keep the current lane only if the full path is clear.
                if self.lane.get('lane_count') == 1 and self.curve_clear(baseline, self.cfg['lookahead']):
                    self._enter('follow', now)
                else:
                    return 0.0, 0.0, reason
            else:
                return min(speed, self.cfg['slow_speed']), curvature, reason
        return speed, baseline, 'lane_follow'

    def publish_motion(self, speed, curvature):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(speed*curvature) if abs(speed) > 1e-6 else 0.0
        self.publisher_.publish(msg)
        self.last_speed, self.last_curvature = speed, curvature

    def timer_callback(self):
        now = time.monotonic()
        dt = clamp(now-self.last_tick, 0.01, 0.2)
        self.last_tick = now
        try:
            if self.slow_sign_until is not None and now >= self.slow_sign_until:
                self._enter('follow', now)
                self.get_logger().info('Speed-bump mission complete: resuming lane following')
            speed, curvature, reason = self.compute_control(now, dt)
            # Keep the original steering curvature and all stop decisions.
            # Expiry automatically restores the speed selected by the original
            # controller (including its existing acceleration/safety limits).
            if 0 < speed and now < self.speed_trigger_until:
                speed = min(speed, self.cfg['target_speed'])
            if not all(math.isfinite(v) for v in (speed, curvature)):
                raise ValueError('Nonfinite control output')
            speed = clamp(speed, -self.cfg['parking_speed'], self.cfg['max_speed'])
            curvature = clamp(curvature, -self.max_curvature, self.max_curvature)
            # Optional single-lane policy: retain measured-hit/braking checks,
            # but do not mistake missing returns for a measured obstacle.
            # Three-lane detections require coverage immediately, even before
            # auto_obstacle_mode has accumulated enough frames to switch modes.
            partial_lane_scan = (
                not self.cfg['require_path_coverage_for_lane_follow'] and speed > 0 and
                self.mode in ('follow', 'slow') and
                self.lane.get('valid') is True and self.lane.get('lane_count') == 1 and
                self.multi_frames == 0)
            sparse = (self.cfg['allow_sparse_scan_crawl'] and speed > 0 and
                      self.mode in ('follow', 'slow', 'guide', 'branch_a', 'branch_b', 'pedestrian', 'traffic') and
                      self.lane.get('lane_count') != 3 and self.multi_frames == 0 and
                      (self.lane.get('valid') is True or reason == 'brief_lane_loss_curve_hold') and
                      self.obstacles.get('valid') is True and
                      self.obstacles.get('scan_health', 0.0) >= 0.3 and
                      self.obstacles.get('front_coverage', 0.0) >= 0.05)
            if speed != 0:
                if self.last_speed*speed < 0 or self.odom_speed*speed < -0.0004:
                    speed, curvature, reason = 0.0, 0.0, 'direction_change_wait_for_stop'
            if speed != 0:
                curvature = clamp(curvature, self.last_curvature-self.cfg['curvature_rate']*dt,
                                  self.last_curvature+self.cfg['curvature_rate']*dt)
                # Slowing down is immediate; acceleration is limited.
                if abs(speed) > abs(self.last_speed):
                    speed = math.copysign(min(abs(speed), abs(self.last_speed)+self.cfg['acceleration']*dt), speed)
                braking_speed = max(abs(speed), abs(self.odom_speed))
                horizon = (braking_speed*self.cfg['reaction_time'] +
                           braking_speed*braking_speed/(2*self.cfg['deceleration'])+0.12)
                direction = 1 if speed > 0 else -1
                if not self.curve_clear(curvature, horizon, direction):
                    # Test the actual swept footprint, not just four central bins.
                    # A measured collision must never be treated as missing coverage.
                    if not self.curve_clear(curvature, horizon, direction, check_coverage=False):
                        speed, curvature, reason = 0.0, 0.0, 'swept_path_obstacle'
                    elif partial_lane_scan:
                        # compute_control has already checked sensor health,
                        # freshness, vehicle ownership, lane validity and e-stop.
                        # Unknown space remains unknown; only measured hits are
                        # checked here. No speed, turn or acceleration limits change.
                        reason = 'partial_scan: '+reason
                    elif sparse:
                        speed = min(speed, self.cfg['sparse_scan_speed'])
                        reason = 'sparse_scan_crawl: '+reason
                    else:
                        speed, curvature, reason = 0.0, 0.0, 'swept_path_unobserved'
            if speed == 0:
                curvature = 0.0
                if (self.mode != 'hold' and self.phase not in ('complete', 'waiting') and
                        (self.started_at is not None or self.mode != 'follow') and
                        not reason.startswith('waiting_for_command_mode')):
                    if self.blocked_since is None:
                        self.blocked_since = now
                    elif now-self.blocked_since > self.cfg['mission_timeout']:
                        self.enter_hold('no_progress_timeout: '+reason, now)
                        reason = self.hold_reason
            else:
                self.blocked_since = None
                if self.started_at is None:
                    self.started_at = now
                    self.progress_at = now
                if self.travel-self.progress_travel >= 0.01:
                    self.progress_at, self.progress_travel = now, self.travel
                elif now-self.progress_at > self.cfg['mission_timeout']:
                    self.enter_hold('odometry_no_progress', now)
                    speed, curvature, reason = 0.0, 0.0, 'odometry_no_progress'
            self.publish_motion(speed, curvature)
            if (speed > 0 and self.lane.get('valid') is True and
                    self.mode in ('follow', 'slow', 'branch_a', 'branch_b')):
                self.last_tracking_curvature = curvature
            self.reason = reason
        except (ValueError, KeyError, TypeError, OverflowError, ZeroDivisionError) as error:
            self.publish_motion(0.0, 0.0)
            self.reason = 'invalid_state: '+str(error)
        status = String()
        status.data = json.dumps(dict(version=1, mode=self.mode, phase=self.phase, reason=self.reason,
                                     speed=self.last_speed, curvature=self.last_curvature,
                                     target_lane=self.target_lane,
                                     hold_reason=self.hold_reason,
                                     control_mode=self.vehicle_control_mode,
                                     odometry_speed=self.odom_speed,
                                     travel=self.travel,
                                     geometry_verified=self.cfg['geometry_verified']),
                                 allow_nan=False, separators=(',', ':'))
        self.status_publisher.publish(status)

    def destroy_node(self):
        self.publish_motion(0.0, 0.0)
        return super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        for cls in (DetectLine, LimoEStop, LimoControl):
            node = cls()
            nodes.append(node)
            executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in reversed(nodes):
            if rclpy.ok():
                node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
