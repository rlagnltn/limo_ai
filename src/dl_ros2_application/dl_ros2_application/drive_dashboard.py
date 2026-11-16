"""A single native window for live detections, driving ROI and vehicle telemetry."""
from dataclasses import replace
import time
import signal
from pathlib import Path

import cv2
import numpy as np
from PIL import Image as PillowImage, ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, String
from limo_msgs.msg import LimoStatus
from dl_ros2_msgs.msg import BoundingBoxes
from dl_ros2_application.left_lane import detect_left_lane, load_lane_config, CONFIG_PATH


OBJECT_NAMES = {
    'red_light': '빨간 신호등', 'green_light': '초록 신호등',
    'yellow_light': '노란 신호등', 'left_sign': '좌회전 표지판',
    'right_sign': '우회전 표지판', 'pedestrian_sign': '보행자 표지판',
    'slow_sign': '서행 표지판', 'parking_sign': '주차 표지판',
    'traffic_cone': '라바콘', 'pedestrian': '보행자',
    'turn_right': '우회전', 'slow_down': '감속', 'speed_up': '가속',
}
STATE_NAMES = {
    'remote_control': ('원격 조종 모드 · 명령 모드 필요', '#F4BC65'),
    'command_only': ('주행 명령 전송 · 차량 정지', '#F4BC65'),
    'vehicle_waiting': ('차량 상태 수신 대기', '#F4BC65'),
    'motion_waiting': ('주행 명령 전송 · 속도 수신 대기', '#F4BC65'),
    'lane_missing': ('차선 미검출 · 기본 P 제어', '#F4BC65'),
    'lane_lost': ('왼쪽 차선 미검출 · 정지', '#FF777E'),
    'waiting': ('주행 추론 대기', '#F4BC65'),
    'emergency_stop': ('장애물 감지 · 비상 정지', '#FF777E'),
    'red_light_stop': ('빨간 신호등 · 정지', '#FF777E'),
    'pedestrian_stop': ('보행자 · 정지', '#FF777E'),
    'turning_right': ('우회전 주행', '#64B8FF'),
    'slowing_down': ('감속 주행', '#F4BC65'),
    'speeding_up': ('가속 주행', '#65DEC2'),
    'driving': ('차선 따라 주행 중', '#65DEC2'),
    'stopped': ('정지 중', '#F4BC65'),
}
BACKGROUND = '#0C121B'
PANEL = '#172131'
TEXT = '#ECF3FA'
MUTED = '#91A5BB'


def fit_image(draw, canvas, image, rect):
    """Letterbox an RGB image and return its displayed bounds."""
    x0, y0, x1, y1 = rect
    if image is None:
        draw.text((x0 + 18, y0 + 20), '카메라 연결을 기다리고 있습니다',
                  font=canvas.fonts[22], fill=MUTED)
        return None
    scale = min((x1 - x0) / image.width, (y1 - y0) / image.height)
    size = (max(1, int(image.width * scale)), max(1, int(image.height * scale)))
    resampling = getattr(PillowImage, 'Resampling', PillowImage)
    picture = image.resize(size, resampling.BILINEAR)
    x = x0 + (x1 - x0 - size[0]) // 2
    y = y0 + (y1 - y0 - size[1]) // 2
    canvas.paste(picture, (x, y))
    return x, y, scale


class DriveDashboard(Node):
    def __init__(self):
        super().__init__('drive_dashboard')
        self.declare_parameter('display', True)
        self.display = self.get_parameter('display').value
        self.bridge = CvBridge()
        self.declare_parameter('config_path', CONFIG_PATH)
        # The original detect_line accepts any nonzero mask moment.
        self.lane_config = replace(load_lane_config(self.get_parameter('config_path').value), min_pixels=1)
        self.right_lane_config = load_lane_config(self.get_parameter('config_path').value)
        self.lane_tracker = self.lane_config.make_tracker()
        self.lane_side = 'left'
        self.merge_tracking = False
        self.merge_tracker = self.right_lane_config.make_merge_tracker()
        self.camera = None
        self.boxes = []
        self.control = {}
        self.e_stop = None
        self.distance = None
        self.vehicle = None
        self.measured = None
        self.received = {}
        self.camera_fps = 0.0
        self.window_open = False
        font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
        if not font.is_file():
            font = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
        self.fonts = {size: ImageFont.truetype(str(font), size)
                      for size in (16, 18, 20, 22, 24, 28, 32)}
        self.subscriptions_ = [
            self.create_subscription(Image, '/camera/color/image_raw',
                                     self.camera_callback, qos_profile_sensor_data),
            self.create_subscription(BoundingBoxes, '/yolo_bounding_boxes',
                                     self.boxes_callback, 10),
            self.create_subscription(Twist, '/cmd_vel', self.control_callback, 10),
            self.create_subscription(Bool, '/e_stop', self.e_stop_callback, 10),
            self.create_subscription(Int32, '/distance_y', self.distance_callback, 10),
            self.create_subscription(String, '/lane_side', self.lane_side_callback,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(Bool, '/merge_tracking', self.merge_callback,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(Odometry, '/odom', self.odom_callback,
                                     qos_profile_sensor_data),
            self.create_subscription(LimoStatus, '/limo_status', self.vehicle_callback,
                                     qos_profile_sensor_data),
        ]
        self.publisher = self.create_publisher(Image, '/autonomous/dashboard', 1)
        self.create_timer(0.1, self.timer_callback)

    def fresh(self, name, age=1.5):
        return time.monotonic() - self.received.get(name, float('-inf')) < age

    def camera_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if image is None:
                return
            now = time.monotonic()
            previous = self.received.get('camera')
            if previous is not None and now > previous:
                fps = 1.0 / (now - previous)
                self.camera_fps = fps if not self.camera_fps else .8 * self.camera_fps + .2 * fps
            self.camera = image
            self.received['camera'] = now
        except (CvBridgeError, ValueError, cv2.error) as error:
            self.get_logger().warning(f'Camera decode: {error}')

    def boxes_callback(self, msg):
        self.boxes = list(msg.bounding_boxes)
        self.received['yolo'] = time.monotonic()

    def control_callback(self, msg):
        self.control = {'command': {'linear_x': msg.linear.x, 'angular_z': msg.angular.z}}
        self.received['control'] = time.monotonic()

    def e_stop_callback(self, msg):
        self.e_stop = msg.data
        self.received['e_stop'] = time.monotonic()

    def distance_callback(self, msg):
        self.distance = msg.data
        self.received['distance'] = time.monotonic()

    def lane_side_callback(self, msg):
        if msg.data != self.lane_side:
            self.lane_side = msg.data
            config = self.right_lane_config if msg.data == 'right' else self.lane_config
            self.lane_tracker = config.make_tracker()
            self.merge_tracker = self.right_lane_config.make_merge_tracker()

    def merge_callback(self, msg):
        if msg.data != self.merge_tracking:
            self.merge_tracking = msg.data
            config = self.right_lane_config if self.lane_side == 'right' else self.lane_config
            self.lane_tracker = config.make_tracker()
            self.merge_tracker = self.right_lane_config.make_merge_tracker()

    def odom_callback(self, msg):
        self.measured = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.received['odom'] = time.monotonic()

    def vehicle_callback(self, msg):
        self.vehicle = msg
        self.received['vehicle'] = time.monotonic()

    def render_frame(self):
        canvas = PillowImage.new('RGB', (1280, 800), BACKGROUND)
        canvas.fonts = self.fonts
        draw = ImageDraw.Draw(canvas)

        def label(x, y, value, size=20, color=TEXT):
            draw.text((x, y), str(value), font=self.fonts[size], fill=color)

        label(24, 12, 'LIMO  자율주행 모니터', 28)
        label(1012, 23, time.strftime('%H:%M:%S'), 20, MUTED)
        for rect in ((16, 72, 816, 576), (16, 592, 816, 784), (832, 72, 1264, 784)):
            draw.rounded_rectangle(rect, radius=16, fill=PANEL)
        label(32, 80, '실시간 객체 탐지', 22)
        label(609, 85, f'{self.camera_fps:.0f} FPS  ·  TESTV2', 16, MUTED)
        image = None
        lane = None
        if self.camera is not None:
            config = self.right_lane_config if self.merge_tracking or self.lane_side == 'right' else self.lane_config
            lane = detect_left_lane(self.camera, config=config,
                                    tracker=self.lane_tracker, side=self.lane_side,
                                    merge_tracker=self.merge_tracker if self.merge_tracking else None)
            annotated = self.camera.copy()
            h, w = annotated.shape[:2]
            x0, y0, x1, y1 = lane['roi']
            cv2.rectangle(annotated, (int(x0 * w / 640), int(y0 * h / 480)),
                          (int(x1 * w / 640), int(y1 * h / 480)), (220, 160, 40), 2)
            if lane['found']:
                if lane['line'] is not None:
                    points = tuple((int(x * w / 640), int(y * h / 480)) for x, y in lane['line'])
                    cv2.line(annotated, *points, (80, 240, 100), 4)
                else:
                    cx, cy = lane['center']
                    cv2.circle(annotated, (int(cx * w / 640), int(cy * h / 480)),
                               7, (80, 240, 100), -1)
            image = PillowImage.fromarray(cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB))
        bounds = fit_image(draw, canvas, image, (28, 120, 804, 564))
        live_boxes = (sorted(self.boxes, key=lambda b: b.probability, reverse=True)
                      if self.fresh('yolo', 3.0) and self.fresh('camera') else [])
        draw = ImageDraw.Draw(canvas)
        if bounds is not None:
            x, y, scale = bounds
            for box in live_boxes:
                coords = (x + max(0, box.xmin) * scale, y + max(0, box.ymin) * scale,
                          x + max(0, box.xmax) * scale, y + max(0, box.ymax) * scale)
                if coords[2] <= coords[0] or coords[3] <= coords[1]:
                    continue
                draw.rectangle(coords, outline='#65DEC2', width=2)
                name = OBJECT_NAMES.get(box.class_name, box.class_name)
                text = f'{box.class_name} {box.probability:.0%}'
                tx, ty = coords[0], max(y, coords[1] - 27)
                box_text = draw.textbbox((tx, ty), text, font=self.fonts[18])
                draw.rectangle((box_text[0] - 3, ty, box_text[2] + 5, ty + 27), fill='#123B38')
                label(tx, ty, text, 18)
        if not self.fresh('camera'):
            label(44, 526, '카메라 영상 대기 / 연결 끊김', 20, '#F4BC65')

        label(32, 600, '차선 ROI 원본', 20)
        label(424, 600, 'HLS 마스크 · 지그재그 중앙점 추종', 18)
        roi = None
        mask_image = None
        if lane is not None:
            x0, y0, x1, y1 = lane['roi']
            crop = lane['frame'][y0:y1, x0:x1].copy()
            reference = int(lane['reference'] - x0)
            if lane['line'] is not None:
                cv2.line(crop, (reference, 0), (reference, y1 - y0 - 1), (255, 160, 40), 2)
            if lane['found']:
                cx, cy = lane['center']
                if lane['line'] is None:
                    cv2.circle(crop, (reference, cy - y0), 3, (255, 160, 40), -1)
                cv2.circle(crop, (cx - x0, cy - y0), 5, (80, 240, 100), -1)
            roi = PillowImage.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
            mask_image = PillowImage.fromarray(lane['mask']).convert('RGB')
        fit_image(draw, canvas, roi, (28, 635, 404, 742))
        fit_image(draw, canvas, mask_image, (420, 635, 804, 742))
        draw = ImageDraw.Draw(canvas)
        if lane is not None:
            count = lane['mask_pixels']
            status = f'검출 · 오차 {lane["error"]:+d}px' if lane['found'] else '미검출 · ROI/색상 확인'
            label(32, 750, f'원본 검출 {count}px · {status}', 16,
                  '#65DEC2' if lane['found'] else '#F4BC65')

        label(852, 86, '현재 리모 상태', 24)
        state = None
        if self.fresh('control'):
            command = self.control['command']
            if self.fresh('e_stop') and self.e_stop:
                state = 'emergency_stop'
            elif not self.fresh('vehicle', 3) or self.vehicle is None:
                state = 'vehicle_waiting'
            elif self.vehicle.control_mode == 3:
                state = 'remote_control'
            elif lane is not None and self.fresh('camera') and not lane['found']:
                state = 'lane_missing'
            elif abs(command['linear_x']) + abs(command['angular_z']) < 1e-6:
                state = 'stopped'
            elif not self.fresh('odom') or self.measured is None:
                state = 'motion_waiting'
            elif abs(self.measured[0]) + abs(self.measured[1]) < 1e-6:
                state = 'command_only'
            else:
                state = 'driving'
        state_text, state_color = STATE_NAMES.get(state, ('제어 상태 수신 대기', '#F4BC65'))
        draw.rounded_rectangle((850, 132, 1246, 190), radius=12, fill='#233247')
        label(866, 145, state_text, 22, state_color)
        offset = f'{self.distance:+d}px' if self.fresh('distance') else '수신 대기'
        side = '오른쪽' if self.lane_side == 'right' else '왼쪽'
        if self.merge_tracking:
            phase = ('합류 확인중' if self.merge_tracker.stable_since is not None
                     else '분리선 추적' if self.merge_tracker.split_seen else '분리선 확인전')
            mode = f'{phase} · {side}'
        else:
            mode = f'{side} 차선 P 제어'
        label(852, 200, f'{mode} · 오차 {offset}', 16, MUTED)

        label(852, 238, '실제 속도 / 회전', 18, MUTED)
        if self.fresh('odom') and self.measured is not None:
            label(852, 266, f'{self.measured[0]:+.2f} m/s   {self.measured[1]:+.2f} rad/s', 24)
        else:
            label(852, 266, '차량 속도 수신 대기', 22, MUTED)
        command = self.control.get('command', {}) if self.fresh('control') else {}
        if command:
            label(852, 309, f'명령  {command["linear_x"]:+.2f} m/s · {command["angular_z"]:+.2f} rad/s', 16, MUTED)
        else:
            label(852, 309, '명령 속도 수신 대기', 16, MUTED)
        if self.fresh('vehicle', 3) and self.vehicle is not None:
            label(852, 347, f'배터리 {self.vehicle.battery_voltage:.1f} V', 20)
            error = self.vehicle.error_code
            label(852, 380, f'제어 {self.vehicle.control_mode} · 구동 {self.vehicle.motion_mode} · 오류 0x{error:04X}',
                  16, '#FF777E' if error else MUTED)
        else:
            label(852, 347, '차량 연결 대기', 20, MUTED)

        draw.line((852, 419, 1244, 419), fill='#304054', width=1)
        label(852, 432, f'현재 탐지 객체  {len(live_boxes)}개', 22)
        if not self.fresh('yolo', 3.0):
            label(852, 478, '객체인식 신호 대기', 20, '#F4BC65')
        elif not live_boxes:
            label(852, 478, '현재 탐지된 객체 없음', 20, MUTED)
        for i, box in enumerate(live_boxes[:6]):
            name = OBJECT_NAMES.get(box.class_name, box.class_name)
            label(852, 474 + i * 36, f'{box.class_name}', 18)
            label(1180, 475 + i * 36, f'{box.probability:.0%}', 18, '#65DEC2')
        if len(live_boxes) > 6:
            label(852, 684, f'외 {len(live_boxes) - 6}개', 16, MUTED)
        for i, (key, name) in enumerate((('camera', '카메라'), ('yolo', '탐지'), ('control', '제어'))):
            x = 852 + i * 126
            color = '#65DEC2' if self.fresh(key, 3.0 if key == 'yolo' else 1.5) else '#F4BC65'
            draw.ellipse((x, 731, x + 8, 739), fill=color)
            label(x + 15, 720, name, 18, color)
        label(852, 753, '종료: Q / Esc / 창 닫기', 16, MUTED)
        return cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)

    def timer_callback(self):
        frame = self.render_frame()
        if self.publisher.get_subscription_count():
            message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            message.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(message)
        if self.display:
            title = 'LIMO Autonomous Dashboard'
            if not self.window_open:
                cv2.namedWindow(title, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(title, 1280, 800)
                self.window_open = True
            cv2.imshow(title, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    node = DriveDashboard()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node.window_open:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
