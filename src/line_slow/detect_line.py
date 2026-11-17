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
        self.course_status = {}
        self.course_received_at = None
        self.course_subscription = self.create_subscription(
            String, '/core/status', self.course_callback, 10)

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

    def course_callback(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict):
                self.course_status = status
                self.course_received_at = time.monotonic()
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
                     stop_distance=None, crosswalk=False)
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
            state['crosswalk'] = len(bands) >= 3
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
            area = max(0, box.xmax-box.xmin)*max(0, box.ymax-box.ymin)
            cv2.putText(overlay, '%s %.0f%% %dpx2' % (box.class_name, box.probability * 100, area),
                        (max(0, box.xmin), max(85, box.ymin - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        lines = ['YOLO: latest detections' if fresh else 'YOLO: waiting / stale']
        control_fresh = (self.controller_received_at is not None and
                         time.monotonic()-self.controller_received_at < 0.5)
        control = self.controller_status if control_fresh else {}
        course_fresh = (self.course_received_at is not None and
                        time.monotonic()-self.course_received_at < 1.0)
        course = self.course_status if course_fresh else {}
        if course:
            progress = course.get('progress', {})
            lines += ['COURSE %s/8: %s | %s | %s' % (
                course.get('stage', '?'), course.get('mission', '?'), course.get('phase', '?'),
                'FINISHED' if course.get('finished') else 'READY' if course.get('ready') else 'BLOCKED'),
                'Travel: %.2f m | elapsed: %.1f s | measured speed: %.2f m/s' % (
                    progress.get('distance_m', 0.0), progress.get('elapsed_sec', 0.0),
                    course.get('measured_speed_mps', 0.0))]
            states = {'pending': 'WAIT', 'waiting': 'WAIT', 'active': 'RUN',
                      'completed': 'DONE', 'failed': 'FAIL'}
            results = course.get('mission_results', [])
            for first in (0, 4):
                lines.append(' | '.join('%s.%s:%s' % (
                    row.get('stage'), row.get('mission'), states.get(row.get('state'), '?'))
                    for row in results[first:first+4]))
            completed = [row for row in results if row.get('state') == 'completed']
            if completed:
                last = completed[-1]
                lines.append('LAST DONE %s.%s: %s' % (
                    last['stage'], last['mission'], last['reason']))
            if course.get('fault'):
                lines.append('COURSE FAULT: '+str(course['fault']))
        else:
            lines.append('COURSE: waiting for core / stale')
        lines += ['Controller: %s / %s' % (control.get('mode', 'waiting'), control.get('phase', '-')),
                  'Control: %s' % str(control.get('reason', 'controller unavailable'))[:90],
                  'Speed: %.2f m/s | lanes: %d | metric calibration: %s' % (
                      control.get('speed', 0.0), lane_state['lane_count'], lane_state['metric_valid'])]
        lines += ['%s: %.0f%% | area %d px2' % (
                      box.class_name, box.probability * 100,
                      max(0, box.xmax-box.xmin)*max(0, box.ymax-box.ymin))
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

def main(args=None):
    rclpy.init(args=args)

    detect_line = DetectLine()
    rclpy.spin(detect_line)

    detect_line.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
