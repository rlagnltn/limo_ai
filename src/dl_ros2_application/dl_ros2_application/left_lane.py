"""HLS left-lane tracking, with two-boundary center tracking for zigzags."""
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import cv2
import numpy as np
import yaml
from limo_ros2_application.lane_geometry import LaneTracker, MergeTracker

CONFIG_PATH = '/home/wego/wego_ws/src/limo_ros2_application/params/detect_line.yaml'


@dataclass(frozen=True)
class LaneConfig:
    roi_x_l: int = 0
    roi_x_h: int = 320
    roi_y_l: int = 400
    roi_y_h: int = 480
    reference_distance: int = 170
    low: tuple = (0, 90, 100)
    high: tuple = (60, 220, 255)
    min_pixels: int = 20
    merge_confirm_sec: float = 0.5
    merge_width_tolerance_px: float = 20.0
    merge_boundary_margin_px: int = 60
    zigzag_min_segment_px: int = 12
    zigzag_min_reversals: int = 2
    zigzag_lookahead_px: int = 60
    zigzag_exit_delay_sec: float = 0.5
    zigzag_smoothing_sec: float = 0.25
    zigzag_max_center_speed_px_sec: float = 120.0
    zigzag_center_reference_x: int = 320
    zigzag_center_band_px: int = 40
    zigzag_boundary_margin_px: int = 60
    zigzag_center_deadband_px: float = 2.0
    zigzag_center_hold_sec: float = 0.3

    def make_tracker(self):
        return LaneTracker(self.zigzag_min_segment_px, self.zigzag_min_reversals,
                           self.zigzag_exit_delay_sec, self.zigzag_smoothing_sec,
                           self.zigzag_max_center_speed_px_sec, self.min_pixels,
                           center_band_px=self.zigzag_center_band_px,
                           boundary_margin_px=self.zigzag_boundary_margin_px,
                           center_deadband_px=self.zigzag_center_deadband_px,
                           center_hold_sec=self.zigzag_center_hold_sec)

    def make_merge_tracker(self):
        return MergeTracker(self.min_pixels, self.merge_boundary_margin_px,
                            self.merge_confirm_sec, self.merge_width_tolerance_px)


@lru_cache(maxsize=4)
def load_lane_config(path=CONFIG_PATH):
    document = yaml.safe_load(Path(path).read_text())
    values = document['/detect_line']['ros__parameters']
    config = LaneConfig(
        **{key: int(values.get(key, getattr(LaneConfig(), key))) for key in (
            'roi_x_l', 'roi_x_h', 'roi_y_l', 'roi_y_h', 'reference_distance',
            'merge_boundary_margin_px',
            'zigzag_min_segment_px', 'zigzag_min_reversals', 'zigzag_lookahead_px',
            'zigzag_center_reference_x', 'zigzag_center_band_px',
            'zigzag_boundary_margin_px')},
        low=tuple(int(values.get(key, default)) for key, default in zip(
            ('lane_h_l', 'lane_l_l', 'lane_s_l'), (0, 90, 100))),
        high=tuple(int(values.get(key, default)) for key, default in zip(
            ('lane_h_h', 'lane_l_h', 'lane_s_h'), (60, 220, 255))),
        min_pixels=int(values.get('min_lane_pixels', 20)),
        merge_confirm_sec=float(values.get('merge_confirm_sec', 0.5)),
        merge_width_tolerance_px=float(values.get('merge_width_tolerance_px', 20.0)),
        zigzag_exit_delay_sec=float(values.get('zigzag_exit_delay_sec', 0.5)),
        zigzag_smoothing_sec=float(values.get('zigzag_smoothing_sec', 0.25)),
        zigzag_max_center_speed_px_sec=float(values.get('zigzag_max_center_speed_px_sec', 120.0)),
        zigzag_center_deadband_px=float(values.get('zigzag_center_deadband_px', 2.0)),
        zigzag_center_hold_sec=float(values.get('zigzag_center_hold_sec', 0.3)))
    if not (0 <= config.roi_x_l < config.roi_x_h <= 320
            and 0 <= config.roi_y_l < config.roi_y_h <= 480):
        raise ValueError('Left ROI must be within x=0..320, y=0..480 (640x480 image).')
    return config


def detect_left_lane(image, reference=None, config=None, tracker=None, side='left',
                     allow_center=True, merge_tracker=None):
    if side == 'right':
        result = detect_left_lane(cv2.flip(image, 1), reference, config, tracker,
                                  allow_center=False, merge_tracker=merge_tracker)
        width = result['frame'].shape[1]
        x0, y0, x1, y1 = result['roi']
        result['roi'] = [width - x1, y0, width - x0, y1]
        result['reference'] = width - 1 - result['reference']
        result['error'] = -result['error']
        result['frame'] = cv2.flip(result['frame'], 1)
        result['mask'] = cv2.flip(result['mask'], 1)
        if result['found']:
            result['x'] = width - 1 - result['x']
            cx, cy = result['center']
            result['center'] = (width - 1 - cx, cy)
            if result['line'] is not None:
                result['line'] = tuple((width - 1 - x, y) for x, y in result['line'])
        return result
    config = config or load_lane_config()
    reference = config.reference_distance if reference is None else reference
    # Original camera coordinates: do not squash the image to 300x300.
    frame = (image.copy() if image.shape[:2] == (480, 640)
             else cv2.resize(image, (640, 480), interpolation=cv2.INTER_LINEAR))
    x0, y0, x1, y1 = config.roi_x_l, config.roi_y_l, config.roi_x_h, config.roi_y_h
    roi = frame[y0:y1, x0:x1]
    hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
    raw_mask = cv2.inRange(hls, np.array(config.low), np.array(config.high))
    pixels = cv2.countNonZero(raw_mask)
    lookahead_y = max(0, y0 - config.zigzag_lookahead_px)
    lookahead = frame[lookahead_y:y1, x0:]
    lane_mask = cv2.inRange(cv2.cvtColor(lookahead, cv2.COLOR_BGR2HLS),
                           np.array(config.low), np.array(config.high))
    lookahead_mask = lane_mask[:, :x1 - x0]
    tracker = tracker if tracker is not None else config.make_tracker()
    if merge_tracker is not None:
        mask, center, line = merge_tracker.process(lane_mask, y0 - lookahead_y)
        tracking_center = False
    else:
        mask, center, line = tracker.process(
            raw_mask, lookahead_mask, y0 - lookahead_y,
            lane_mask=lane_mask if allow_center else None,
            reference_x=reference - x0,
            center_reference_x=config.zigzag_center_reference_x - x0)
        reference = tracker.reference_x + x0
        tracking_center = tracker.tracking_center
    if merge_tracker is not None or tracking_center:
        x1 = frame.shape[1]
        pixels = cv2.countNonZero(lane_mask)
    result = {'found': False, 'reference': reference, 'roi': [x0, y0, x1, y1],
              'mask': mask, 'mask_pixels': pixels, 'frame': frame,
              'x': None, 'error': 0, 'line': None, 'center': None,
              'center_estimated': tracker.center_estimated if merge_tracker is None else False,
              'reason': 'no_mask_pixels' if not pixels else 'too_few_pixels'}
    if pixels < config.min_pixels or center is None:
        return result
    cx, cy = center[0] + x0, center[1] + y0
    points = None if tracking_center else ((cx, y0), (cx, y1 - 1))
    result.update(found=True, x=float(cx), error=int(reference - cx),
                  line=points, center=(cx, cy), reason='detected')
    return result


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool
    from sensor_msgs.msg import CompressedImage, Image
    from cv_bridge import CvBridge
    from dl_ros2_msgs.msg import Control
    import time
    import signal

    class LeftLaneTracker(Node):
        def __init__(self):
            super().__init__('left_lane_tracker')
            self.declare_parameter('config_path', CONFIG_PATH)
            self.config = load_lane_config(self.get_parameter('config_path').value)
            self.lane_tracker = self.config.make_tracker()
            self.declare_parameter('reference_x', self.config.reference_distance)
            self.reference = int(self.get_parameter('reference_x').value)
            self.last_image = float('-inf')
            self.bridge = CvBridge()
            self.mask_pub = self.create_publisher(Image, '/lane/mask', 1)
            self.roi_pub = self.create_publisher(Image, '/lane/roi', 1)
            self.control = self.create_publisher(Control, '/control', 10)
            self.valid = self.create_publisher(Bool, '/lane_tracking_valid', 10)
            self.subscription = self.create_subscription(CompressedImage,
                '/camera/color/image_raw/compressed', self.image_callback, qos_profile_sensor_data)
            self.create_timer(.1, self.check_timeout)

        def check_timeout(self):
            if time.monotonic() - self.last_image > .8:
                self.valid.publish(Bool(data=False))

        def image_callback(self, msg):
            frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                self.valid.publish(Bool(data=False))
                return
            self.last_image = time.monotonic()
            result = detect_left_lane(frame, self.reference, self.config, self.lane_tracker)
            if self.mask_pub.get_subscription_count():
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(result['mask'], encoding='mono8'))
            if self.roi_pub.get_subscription_count():
                x0, y0, x1, y1 = result['roi']
                self.roi_pub.publish(self.bridge.cv2_to_imgmsg(result['frame'][y0:y1, x0:x1], encoding='bgr8'))
            self.valid.publish(Bool(data=result['found']))
            if result['found']:
                self.control.publish(Control(normal_mode=result['error'], turn_mode=result['error']))

    rclpy.init(args=args)
    node = LeftLaneTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
