#!/usr/bin/env python3
"""YOLO11 testv2 클래스 0/1의 박스 면적으로 동작하는 ROS 2 제어 노드.

먼저 yolo_object_detect를 testv2.engine으로 실행해야 한다.
크기는 원본 영상 좌표 기준 (xmax-xmin)*(ymax-ymin), 단위 pixel^2.
파라미터는 실행 시 --ros-args -p red_min_area:=3000.0 등으로 지정한다.
이 노드가 제어하는 동안 다른 /cmd_vel 발행 노드는 중지해야 한다.
기본적으로 시작 시 직진하며 빨간불 감지 후 지연 시간 동안에도 직진한다.
정지 이후에는 0을 계속 발행한다. 검출 스트림이 없거나 끊기면 정지한다.
"""

import math
import os
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from dl_ros2_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage


class TrafficLightTrigger(Node):
    def __init__(self):
        super().__init__('traffic_light_trigger')
        defaults = {
            'boxes_topic': '/yolo_bounding_boxes',
            'cmd_vel_topic': '/cmd_vel',
            'emergency_stop_topic': '/emergency_stop',
            'red_min_area': 3000.0,
            'green_min_area': 2700.0,
            'min_confidence': 0.7,
            'red_stop_delay_sec': 0.0,
            'green_drive_sec': 3.0,
            'forward_speed': 0.2,
            'start_forward': True,
            'rearm_absence_sec': 1.0,
            'detection_timeout_sec': 1.0,
            'show_window': True,
            'image_topic': '/camera/color/image_raw/compressed',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        # 시작 시 파라미터를 읽는다. 변경하려면 새 파라미터로 재실행한다.
        self.config = {name: self.get_parameter(name).value for name in defaults}
        for name, value in self.config.items():
            if isinstance(value, float) and (not math.isfinite(value) or value < 0):
                raise ValueError(f'{name} must be finite and nonnegative')
        if not 0 <= self.config['min_confidence'] <= 1:
            raise ValueError('min_confidence must be between 0 and 1')
        for name in ('rearm_absence_sec', 'detection_timeout_sec'):
            if self.config[name] <= 0:
                raise ValueError(f'{name} must be positive')

        self.state = 'startup' if self.config['start_forward'] else 'idle'
        self.deadline = 0.0
        self.last_message = None
        self.emergency = False
        self.armed = {'red_light': True, 'green_light': True}
        self.absent_since = {'red_light': None, 'green_light': None}
        self.latest_boxes = []
        self.image = None
        self.image_received = None
        self.bridge = CvBridge()
        self.window_name = 'Traffic Light Trigger - Q / ESC: stop and exit'
        self.show_window = self.config['show_window']
        if self.show_window and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            self.get_logger().warning('No desktop display; preview disabled')
            self.show_window = False
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            self.create_subscription(
                CompressedImage, self.config['image_topic'], self.on_image,
                qos_profile_sensor_data)
            self.create_timer(0.1, self.draw_preview)
        self.publisher = self.create_publisher(Twist, self.config['cmd_vel_topic'], 10)
        self.create_subscription(
            BoundingBoxes, self.config['boxes_topic'], self.on_boxes,
            qos_profile_sensor_data)
        self.create_subscription(
            Bool, self.config['emergency_stop_topic'], self.on_emergency, 10)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            'Waiting for boxes: red_light(0), green_light(1); '
            f"area thresholds={self.config['red_min_area']}/"
            f"{self.config['green_min_area']} pixel^2")

    def publish_speed(self, speed=0.0):
        command = Twist()
        command.linear.x = speed
        self.publisher.publish(command)

    def stop(self, reason):
        self.state = 'stopped'
        self.publish_speed()
        self.get_logger().info(reason)

    def on_emergency(self, msg):
        self.emergency = msg.data
        if self.emergency:
            self.stop('Emergency stop; current action cancelled')

    def on_boxes(self, msg):
        now = time.monotonic()
        self.latest_boxes = list(msg.bounding_boxes)
        # 메시지 단절 시간은 신호가 사라진 시간으로 취급하지 않는다.
        if (self.last_message is not None
                and now - self.last_message > self.config['detection_timeout_sec']):
            self.absent_since = dict.fromkeys(self.armed, None)
        self.last_message = now
        present = set()
        qualified = set()
        for box in msg.bounding_boxes:
            name = box.class_name
            if name not in self.armed:
                continue
            if not math.isfinite(box.probability):
                continue
            if box.probability < self.config['min_confidence']:
                continue
            present.add(name)
            width = max(0, box.xmax - box.xmin)
            height = max(0, box.ymax - box.ymin)
            key = 'red_min_area' if name == 'red_light' else 'green_min_area'
            if width > 0 and height > 0 and width * height >= self.config[key]:
                qualified.add(name)

        for name in self.armed:
            if name in present:
                self.absent_since[name] = None
            elif self.absent_since[name] is None:
                self.absent_since[name] = now
            elif now - self.absent_since[name] >= self.config['rearm_absence_sec']:
                self.armed[name] = True

        if self.emergency:
            return
        # 동시에 보이면 빨간불 우선. 빨간불 대기 타이머는 매 프레임 초기화하지 않는다.
        if 'red_light' in qualified:
            if self.armed['red_light'] and self.state != 'red_wait':
                self.armed['red_light'] = False
                self.state = 'red_wait'
                self.deadline = now + self.config['red_stop_delay_sec']
                self.get_logger().info('Red trigger: delayed stop scheduled')
                self.on_timer()
            return
        if (self.state in ('idle', 'stopped')
                and 'green_light' in qualified and self.armed['green_light']):
            self.armed['green_light'] = False
            self.state = 'green_drive'
            self.deadline = now + self.config['green_drive_sec']
            self.get_logger().info('Green trigger: drive straight, then stop')
            self.on_timer()

    def on_timer(self):
        now = time.monotonic()
        if self.emergency:
            self.publish_speed()
        elif self.state == 'startup':
            # 첫 검출 메시지를 받기 전에는 카메라/YOLO 준비를 기다린다.
            if self.last_message is None:
                self.publish_speed()
            elif now - self.last_message > self.config['detection_timeout_sec']:
                self.stop('Detection stream timeout: stop')
            else:
                self.state = 'forward'
                self.publish_speed(self.config['forward_speed'])
        elif self.state in ('red_wait', 'green_drive') and now >= self.deadline:
            self.stop('Action completed: stop')
        elif self.state in ('forward', 'red_wait', 'green_drive'):
            if (self.last_message is None
                    or now - self.last_message > self.config['detection_timeout_sec']):
                self.stop('Detection stream timeout: stop')
            else:
                self.publish_speed(self.config['forward_speed'])
        elif self.state == 'stopped':
            self.publish_speed()

    def on_image(self, msg):
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            self.image_received = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f'Image decode failed: {exc}')

    def draw_preview(self):
        # 제어와 별도 타이머: 화면을 위해 제어 콜백에서 기다리지 않는다.
        import numpy as np

        now = time.monotonic()
        frame = (self.image.copy() if self.image is not None
                 else np.zeros((480, 800, 3), dtype=np.uint8))
        boxes_fresh = (self.last_message is not None
                       and now - self.last_message <= self.config['detection_timeout_sec'])
        if boxes_fresh:
            for box in self.latest_boxes:
                width = max(0, box.xmax - box.xmin)
                height = max(0, box.ymax - box.ymin)
                area = width * height
                color = (150, 150, 150)
                label = f'{box.class_name} conf={box.probability:.2f}'
                if box.class_name in self.armed:
                    key = ('red_min_area' if box.class_name == 'red_light'
                           else 'green_min_area')
                    threshold = self.config[key]
                    passed = (width > 0 and height > 0 and area >= threshold
                              and math.isfinite(box.probability)
                              and box.probability >= self.config['min_confidence'])
                    color = (0, 255, 0) if passed else (0, 165, 255)
                    label += f' area={area}/{threshold:.0f} {"PASS" if passed else "BELOW"}'
                cv2.rectangle(frame, (box.xmin, box.ymin),
                              (box.xmax, box.ymax), color, 2)
                cv2.putText(frame, label, (max(0, box.xmin), max(85, box.ymin - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        state = 'EMERGENCY STOP' if self.emergency else self.state.upper()
        if self.state in ('red_wait', 'green_drive') and not self.emergency:
            state += f' {max(0.0, self.deadline - now):.1f}s'
        lines = [
            f'State: {state} | boxes: {"OK" if boxes_fresh else "WAIT / STALE"}',
            ' | '.join(f'{name}: {"ARMED" if armed else "LATCHED"}'
                       for name, armed in self.armed.items()),
            'Green box: PASS | Orange: below area/conf | Q/ESC: stop + exit',
        ]
        if self.image_received is None or now - self.image_received > 1.0:
            lines.append('CAMERA: WAIT / STALE')
        for index, line in enumerate(lines):
            position = (10, 20 + index * 20)
            cv2.putText(frame, line, position, cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 0, 0), 3)
            cv2.putText(frame, line, position, cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1)
        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27) or cv2.getWindowProperty(
                self.window_name, cv2.WND_PROP_VISIBLE) < 1:
            self.stop('Preview closed: stop')
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrafficLightTrigger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_speed()
            if node.show_window:
                cv2.destroyAllWindows()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
