import rclpy
import signal
from rclpy.node import Node 
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool, String, Float64MultiArray
from limo_ros2_application.lane_geometry import LaneTracker, MergeTracker, is_v_marking, is_curve_mask

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

        # Publish result (offset between reference distance and real distance)
        self.dis_publisher = self.create_publisher(Int32, 'distance_y', 10)
        self.v_publisher = self.create_publisher(Bool, 'v_marking', 10)
        self.curve_publisher = self.create_publisher(Bool, 'lane_curve', 10)
        self.right_detected_publisher = self.create_publisher(Bool, 'right_lane_detected', 10)
        self.merge_complete_publisher = self.create_publisher(Bool, 'merge_complete', 10)
        self.zigzag_publisher = self.create_publisher(Float64MultiArray, 'zigzag_lane_state', 1)

        # Publish Image for debugging
        self.debug_publisher = self.create_publisher(Image, 'debug_image', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # Parameters (For Masking Lane, For the reference distance of lane, ROI)
        self.declare_parameter('roi_x_l', 0)
        self.declare_parameter('roi_x_h', 320) 
        self.declare_parameter('roi_y_l', 400)
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

        self.declare_parameter('zigzag_min_segment_px', 12)
        self.declare_parameter('zigzag_min_reversals', 2)
        self.zigzag_min_segment_px = self.get_parameter('zigzag_min_segment_px').value
        self.zigzag_min_reversals = self.get_parameter('zigzag_min_reversals').value
        self.zigzag_lookahead_px = self.declare_parameter('zigzag_lookahead_px', 60).value
        exit_delay = self.declare_parameter('zigzag_exit_delay_sec', 0.5).value
        smoothing = self.declare_parameter('zigzag_smoothing_sec', 0.25).value
        max_center_speed = self.declare_parameter('zigzag_max_center_speed_px_sec', 120.0).value
        self.center_reference_x = self.declare_parameter('zigzag_center_reference_x', 320).value
        min_lane_pixels = self.declare_parameter('min_lane_pixels', 20).value
        self.min_lane_pixels = min_lane_pixels
        self.curve_min_deviation = self.declare_parameter('curve_min_deviation_px', 12.0).value
        center_band = self.declare_parameter('zigzag_center_band_px', 40).value
        boundary_margin = self.declare_parameter('zigzag_boundary_margin_px', 60).value
        center_deadband = self.declare_parameter('zigzag_center_deadband_px', 2.0).value
        center_hold = self.declare_parameter('zigzag_center_hold_sec', 0.3).value
        self.lane_tracker_parameters = dict(
            min_segment_px=self.zigzag_min_segment_px,
            min_reversals=self.zigzag_min_reversals, exit_delay=exit_delay,
            smoothing_sec=smoothing, max_center_speed=max_center_speed,
            min_lane_pixels=min_lane_pixels, center_band_px=center_band,
            boundary_margin_px=boundary_margin, center_deadband_px=center_deadband,
            center_hold_sec=center_hold)
        self.lane_tracker = LaneTracker(**self.lane_tracker_parameters)
        self.merge_tracker_parameters = dict(
            min_pixels=min_lane_pixels,
            margin=self.declare_parameter('merge_boundary_margin_px', 60).value,
            confirm_sec=self.declare_parameter('merge_confirm_sec', 0.5).value,
            width_tolerance=self.declare_parameter('merge_width_tolerance_px', 20.0).value)
        self.merge_tracker = MergeTracker(**self.merge_tracker_parameters)
        self.merge_tracking = False
        self.lane_side = 'left'
        self.lane_subscription = self.create_subscription(
            String, 'lane_side', self.lane_side_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.merge_subscription = self.create_subscription(
            Bool, 'merge_tracking', self.merge_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.declare_parameter('reference_distance', 170)
        self.reference_distance = self.get_parameter('reference_distance')

        # Parameter For debugging
        # 0: ROI
        # 1: Masking
        # 2: Left-lane target, or the corridor center used for zigzag alignment
        self.declare_parameter('debug_image_num', 2) #default 2
        self.debug_sequence = self.get_parameter('debug_image_num')

        # to syncronize subscriber and publisher
        self.sub_flag = False

    def lane_side_callback(self, msg):
        if msg.data != self.lane_side:
            self.lane_side = msg.data
            self.lane_tracker = LaneTracker(**self.lane_tracker_parameters)
            self.merge_tracker = MergeTracker(**self.merge_tracker_parameters)

    def merge_callback(self, msg):
        if msg.data != self.merge_tracking:
            self.merge_tracking = msg.data
            self.lane_tracker = LaneTracker(**self.lane_tracker_parameters)
            self.merge_tracker = MergeTracker(**self.merge_tracker_parameters)

    def timer_callback(self):
        if self.sub_flag:     
            if self.debug_sequence.value == 0:
                self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.roi_,'bgr8'))
            elif self.debug_sequence.value == 1:
                self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.mask_yellow,'mono8'))
            else:    
                self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.image_,'bgr8'))
          
    def image_callback(self, msg):
        # convert opencv Mat type to image msg type
        self.image_ = self.br.imgmsg_to_cv2(msg, 'bgr8')
        if self.lane_side == 'right':
            self.image_ = cv2.flip(self.image_, 1)
        
        # Make Region of Interest
        self.roi_ = self.image_[self.roi_y_l.value:self.roi_y_h.value,
                                self.roi_x_l.value:self.roi_x_h.value]

        # Masking the lane
        hls = cv2.cvtColor(self.roi_, cv2.COLOR_BGR2HLS)
        raw_mask = cv2.inRange(hls, self.yellow_lane_low, self.yellow_lane_high)
        lookahead_y = max(0, self.roi_y_l.value - self.zigzag_lookahead_px)
        lookahead = self.image_[lookahead_y:self.roi_y_h.value,
                                self.roi_x_l.value:]
        lane_mask = cv2.inRange(cv2.cvtColor(lookahead, cv2.COLOR_BGR2HLS),
                               self.yellow_lane_low, self.yellow_lane_high)
        v_marking = is_v_marking(lane_mask)
        self.v_publisher.publish(Bool(data=v_marking))
        lookahead_mask = lane_mask[:, :self.roi_x_h.value - self.roi_x_l.value]
        curve = (not v_marking or self.merge_tracking) and is_curve_mask(
            lookahead_mask, self.curve_min_deviation)
        self.curve_publisher.publish(Bool(data=curve))
        if self.merge_tracking:
            self.mask_yellow, center, line = self.merge_tracker.process(
                lane_mask, self.roi_y_l.value - lookahead_y)
            reference_x = self.reference_distance.value
        else:
            self.mask_yellow, center, line = self.lane_tracker.process(
                raw_mask, lookahead_mask, self.roi_y_l.value - lookahead_y,
                lane_mask=lane_mask if self.lane_side == 'left' else None,
                reference_x=self.reference_distance.value - self.roi_x_l.value,
                center_reference_x=self.center_reference_x - self.roi_x_l.value)
            reference_x = self.lane_tracker.reference_x + self.roi_x_l.value
        zigzag_active = (not self.merge_tracking and self.lane_side == 'left'
                         and self.lane_tracker.zigzag)
        corridor = (self.lane_tracker.corridor
                    if not self.merge_tracking and self.lane_side == 'left' else None)
        # Atomic packet: active, valid, image tilt [rad], center error [px],
        # left/right clearance [px], centerline bend [px]. Zero placeholders
        # with valid=False mean stop, never a measured straight corridor.
        geometry = ([corridor.tilt_rad, corridor.center_error_px,
                     corridor.left_clearance_px, corridor.right_clearance_px,
                     corridor.bend_px] if corridor is not None else [0.] * 5)
        self.zigzag_publisher.publish(Float64MultiArray(
            data=[float(zigzag_active), float(corridor is not None)] + geometry))
        if (self.lane_side == 'right' and not self.merge_tracking
                and cv2.countNonZero(raw_mask) < self.min_lane_pixels):
            center = None
        self.right_detected_publisher.publish(Bool(data=(
            self.lane_side == 'right' and center is not None
            and (not v_marking or self.merge_tracking))))
        if self.merge_tracking or self.lane_tracker.tracking_center:
            self.roi_ = self.image_[self.roi_y_l.value:self.roi_y_h.value,
                                    self.roi_x_l.value:]

        if center is not None:
            cx = center[0] + self.roi_x_l.value
            cy = center[1] + self.roi_y_l.value
            if self.lane_tracker.tracking_center:
                cv2.circle(self.image_, (reference_x, cy), 5, (0, 255, 0), -1)
            else:
                cv2.line(self.image_,
                        (reference_x, 0),
                        (reference_x, self.image_.shape[0]),
                        (0, 255, 0), 
                        5)
            self.image_ = cv2.circle(self.image_, (cx, cy), 10,(255, 0, 0), -1)

            distance_to_ref = reference_x - cx
        else: # When limo cannot find lane publish 0 data
            distance_to_ref = 0

        if self.lane_side == 'right':
            distance_to_ref = -distance_to_ref
            self.image_ = cv2.flip(self.image_, 1)
            self.roi_ = cv2.flip(self.roi_, 1)
            self.mask_yellow = cv2.flip(self.mask_yellow, 1)

        # Publishing the offset between current distance with lane and reference distance
        dis = Int32()
        dis.data = distance_to_ref
        self.dis_publisher.publish(dis)
        self.merge_complete_publisher.publish(Bool(data=(
            self.merge_tracking and self.merge_tracker.complete)))

        self.sub_flag = True

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)

    detect_line = DetectLine()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(detect_line, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        detect_line.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
