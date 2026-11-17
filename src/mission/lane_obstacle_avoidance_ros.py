#!/usr/bin/env python3
"""ROS 2 I/O adapter for the unchanged Xycar lane/avoidance algorithm.

Laser ranges and obstacle_distance use metres. Images are resized to the
original 640x480 coordinate system. Camera mounting/perspective still needs
on-robot verification; resizing is not camera calibration.
"""
import math
import os
from pathlib import Path
import time

import cv2
import numpy as np

from lane_obstacle_avoidance import LaneObstacleAvoidance


def scan_bins(scan, yaw=0.0, translation=(0.0, 0.0)):
    """Rebin valid returns into 360 degrees: 0 forward, 90 robot-left.

    Missing/unobserved bins stay NaN, never zero or falsely clear space.
    Transform the laser origin and heading to base_link before indexing.
    """
    bins = [float('nan')] * 360
    if not math.isfinite(scan.angle_increment) or scan.angle_increment == 0:
        return bins
    if not math.isfinite(scan.angle_min):
        return bins
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max or distance <= 0:
            continue
        angle = scan.angle_min + i * scan.angle_increment + yaw
        x = distance * math.cos(angle) + translation[0]
        y = distance * math.sin(angle) + translation[1]
        radius = math.hypot(x, y)
        index = round(math.degrees(math.atan2(y, x))) % 360
        if not math.isfinite(bins[index]) or radius < bins[index]:
            bins[index] = radius
    return bins


def twist_values(steering, speed, wheelbase, max_angle, steering_scale):
    """Original positive steering means right; ROS positive yaw means left.

    steering_scale converts one original motor-command unit into radians.
    This scale is configurable because the original servo calibration is absent.
    """
    angle = max(-max_angle, min(max_angle, -steering * steering_scale))
    return float(speed), float(speed * math.tan(angle) / wheelbase)


def sector_clear(bins, indices, clearance):
    samples = [bins[i % 360] for i in indices]
    valid = [v for v in samples if math.isfinite(v) and v > 0]
    return len(valid) >= len(samples) * 0.5 and min(valid) > clearance


def save_hsv(path, low, high):
    """Write a ROS parameter file that can be passed to --params-file."""
    target = Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        'lane_obstacle_avoidance:\n  ros__parameters:\n'
        f'    lane_hsv_low: {list(low)}\n'
        f'    lane_hsv_high: {list(high)}\n')
    return target


class HSVTuner:
    WINDOW = 'Lane HSV sliders'
    PREVIEW = 'Lane preview: camera | mask | warped mask'
    CHANNELS = (('H', 179), ('S', 255), ('V', 255))

    def __init__(self, algorithm):
        self.algorithm = algorithm
        self.closed = False
        cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WINDOW, 640, 320)
        cv2.namedWindow(self.PREVIEW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.PREVIEW, 1440, 420)
        for i, (channel, maximum) in enumerate(self.CHANNELS):
            for side, values in (('low', algorithm.lane_hsv_low),
                                 ('high', algorithm.lane_hsv_high)):
                cv2.createTrackbar(channel + ' ' + side, self.WINDOW,
                                   values[i], maximum, lambda _: None)

    def update(self, image):
        if self.closed:
            return -1
        if any(cv2.getWindowProperty(w, cv2.WND_PROP_VISIBLE) < 1
               for w in (self.WINDOW, self.PREVIEW)):
            self.close()
            return -1
        low, high = [], []
        for channel, _ in self.CHANNELS:
            lower = cv2.getTrackbarPos(channel + ' low', self.WINDOW)
            upper = cv2.getTrackbarPos(channel + ' high', self.WINDOW)
            if lower > upper:
                upper = lower
                cv2.setTrackbarPos(channel + ' high', self.WINDOW, upper)
            low.append(lower)
            high.append(upper)
        self.algorithm.lane_hsv_low = tuple(low)
        self.algorithm.lane_hsv_high = tuple(high)
        if image is None:
            canvas = np.zeros((240, 900, 3), np.uint8)
            cv2.putText(canvas, 'Waiting for camera...', (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        else:
            mask = self.algorithm.color_mask(image)
            warped = self.algorithm.warp_mask(mask)
            panels = [image.copy(), cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
                      cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)]
            for panel, label in zip(panels, ('CAMERA', 'MASK: white = selected', 'WARPED MASK')):
                cv2.putText(panel, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 0, 255), 2)
            canvas = np.hstack(panels)
        footer = np.zeros((70, canvas.shape[1], 3), np.uint8)
        cv2.putText(footer, f'HSV low={low} high={high} | S: save | Q/Esc: close preview',
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.imshow(self.PREVIEW, np.vstack((canvas, footer)))
        key = cv2.waitKey(1) & 0xff
        if key in (27, ord('q')):
            self.close()
        return key

    def close(self):
        self.closed = True
        for window in (self.WINDOW, self.PREVIEW):
            try:
                cv2.destroyWindow(window)
            except cv2.error:
                pass


def main(args=None):
    # Keep algorithm and adapter helpers importable without a ROS installation.
    import rclpy
    from cv_bridge import CvBridge, CvBridgeError
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import Image, LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    class LaneAvoidanceNode(Node):
        def __init__(self):
            super().__init__('lane_obstacle_avoidance')
            defaults = dict(image_topic='/camera/color/image_raw', scan_topic='/scan',
                            cmd_vel_topic='/cmd_vel', base_frame='base_link',
                            speed=0.05, obstacle_distance=0.5, stop_distance=0.2,
                            side_clearance=0.35, sensor_timeout=0.5,
                            wheelbase=0.2, max_steering_angle=0.35,
                            steering_scale=0.007, initial_lane='left',
                            lane_hsv_low=[15, 70, 70], lane_hsv_high=[40, 255, 255],
                            tune_hsv=False, preview_only=False,
                            hsv_save_path='~/.ros/lane_hsv.yaml')
            self.cfg = c = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            for key in ('speed', 'obstacle_distance', 'stop_distance', 'side_clearance',
                        'sensor_timeout', 'wheelbase', 'max_steering_angle', 'steering_scale'):
                if not math.isfinite(c[key]) or c[key] <= 0:
                    raise ValueError(f'{key} must be finite and positive')
            if c['stop_distance'] >= c['obstacle_distance'] or c['max_steering_angle'] >= math.pi / 2:
                raise ValueError('Invalid obstacle distances or steering limit')
            self.algorithm = LaneObstacleAvoidance(
                obstacle_distance=c['obstacle_distance'],
                lane_hsv_low=c['lane_hsv_low'], lane_hsv_high=c['lane_hsv_high'])
            self.algorithm.reset(c['initial_lane'])
            self.tuner = None
            if c['tune_hsv']:
                if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                    self.get_logger().warning('HSV sliders require a desktop display (e.g. NoMachine terminal)')
                else:
                    try:
                        self.tuner = HSVTuner(self.algorithm)
                    except cv2.error as exc:
                        self.get_logger().warning('HSV window unavailable: ' + str(exc))
            self.bridge = CvBridge()
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.image = self.scan = None
            self.image_at = self.scan_at = None
            self.last_status = None
            self.publisher = self.create_publisher(Twist, c['cmd_vel_topic'], 10)
            self.status = self.create_publisher(String, '~/status', 10)
            self.subscriptions_ = [
                self.create_subscription(Image, c['image_topic'], self.image_callback, qos_profile_sensor_data),
                self.create_subscription(LaserScan, c['scan_topic'], self.scan_callback, qos_profile_sensor_data),
            ]
            self.timer = self.create_timer(0.05, self.tick)
            self.get_logger().info('LIMO adapter ready; waiting for camera, scan and TF')

        def stamp_valid(self, msg):
            stamp = msg.header.stamp
            if not (stamp.sec or stamp.nanosec):
                return True
            age = self.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9)
            return -0.1 <= age <= self.cfg['sensor_timeout']

        def image_callback(self, msg):
            self.image = None
            if not self.stamp_valid(msg):
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.image = cv2.resize(frame, (640, 480))
                self.image_at = time.monotonic()
            except (CvBridgeError, cv2.error, ValueError) as exc:
                self.report('invalid_camera: ' + str(exc))

        def scan_callback(self, msg):
            self.scan = msg if self.stamp_valid(msg) else None
            self.scan_at = time.monotonic()

        def report(self, reason):
            if reason != self.last_status:
                msg = String()
                msg.data = reason
                self.status.publish(msg)
                self.get_logger().info(reason)
                self.last_status = reason

        def tick(self):
            if self.tuner is not None and not self.tuner.closed:
                try:
                    key = self.tuner.update(self.image)
                    self.cfg['lane_hsv_low'] = list(self.algorithm.lane_hsv_low)
                    self.cfg['lane_hsv_high'] = list(self.algorithm.lane_hsv_high)
                    if key in (ord('s'), ord('S')):
                        path = save_hsv(self.cfg['hsv_save_path'],
                                        self.algorithm.lane_hsv_low, self.algorithm.lane_hsv_high)
                        self.get_logger().info('Saved HSV parameters: ' + str(path))
                except OSError as exc:
                    self.get_logger().error('Cannot save HSV parameters: ' + str(exc))
                except cv2.error as exc:
                    self.tuner.close()
                    self.get_logger().warning('HSV preview closed: ' + str(exc))
            if self.cfg['preview_only']:
                self.report('preview_only: no motor commands are published')
                return
            command = Twist()
            reason = 'waiting_for_fresh_sensors'
            now, c = time.monotonic(), self.cfg
            if (self.image is not None and self.scan is not None
                    and now - self.image_at <= c['sensor_timeout']
                    and now - self.scan_at <= c['sensor_timeout']):
                try:
                    yaw, translation = 0.0, (0.0, 0.0)
                    if not self.scan.header.frame_id:
                        raise ValueError('scan has no frame_id')
                    if self.scan.header.frame_id != c['base_frame']:
                        tf = self.tf.lookup_transform(c['base_frame'], self.scan.header.frame_id,
                                                      Time.from_msg(self.scan.header.stamp)).transform
                        q = tf.rotation
                        # This is a planar sensor adapter; reject tilted scans.
                        if abs(q.x) > 0.05 or abs(q.y) > 0.05:
                            raise ValueError('LiDAR is not planar in base_frame')
                        yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
                        translation = (tf.translation.x, tf.translation.y)
                    bins = scan_bins(self.scan, yaw, translation)
                    if not sector_clear(bins, range(-7, 8), c['stop_distance']):
                        reason = 'front_blocked_or_unobserved'
                    else:
                        previous = self.algorithm.lane, self.algorithm.target_lane
                        result = self.algorithm.update(self.image, bins)
                        if result is None:
                            reason = 'lane_or_scan_unavailable'
                        else:
                            side = result.target_lane
                            indices = range(20, 91) if side == 'left' else range(-90, -19)
                            if side and not sector_clear(bins, indices, c['side_clearance']):
                                self.algorithm.lane, self.algorithm.target_lane = previous
                                reason = 'target_lane_blocked_or_unobserved'
                            else:
                                command.linear.x, command.angular.z = twist_values(
                                    result.steering, c['speed'], c['wheelbase'],
                                    c['max_steering_angle'], c['steering_scale'])
                                reason = f'lane={result.lane}, target={side}, steering={result.steering:.2f}'
                except (TransformException, ValueError, cv2.error, OverflowError) as exc:
                    reason = 'input_unavailable: ' + str(exc)
            self.publisher.publish(command)
            self.report(reason)

    rclpy.init(args=args)
    node = None
    try:
        node = LaneAvoidanceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.tuner is not None:
                node.tuner.close()
            if rclpy.ok() and not node.cfg['preview_only']:
                node.publisher.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
