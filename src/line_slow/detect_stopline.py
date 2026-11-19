#!/usr/bin/env python3
"""Detect white paint in the stop-line ROI, with an optional tuning window."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DetectStopline(Node):
    WINDOW = 'Stop line ROI (x 100-439, y 400-479)'
    HSV_CONTROLS = (
        ('H min', 0, 179), ('H max', 179, 179),
        ('S min', 0, 255), ('S max', 60, 255),
        ('V min', 200, 255), ('V max', 255, 255),
    )

    WHITE_THRESHOLD_PERCENT = 20

    def __init__(self, on_detection=None, enabled=None):
        super().__init__('detect_stopline')
        self.bridge = CvBridge()
        self.white_alert = False
        self.on_detection = on_detection
        self.enabled = enabled
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback,
            qos_profile_sensor_data,
        )

    def image_callback(self, msg):
        if self.enabled is not None and not self.enabled():
            return
        if msg.width < 440 or msg.height < 480:
            self.get_logger().warning(
                f'Camera image {msg.width}x{msg.height} is too small for the stop-line ROI.',
                throttle_duration_sec=5.0,
            )
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return

        roi = frame[400:480, 100:440]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        bounds = [initial if self.on_detection is not None else
                  cv2.getTrackbarPos(label, self.WINDOW)
                  for label, initial, _ in self.HSV_CONTROLS]
        mask = cv2.inRange(hsv, tuple(bounds[::2]), tuple(bounds[1::2]))
        white_pixels = cv2.countNonZero(mask)
        alert = white_pixels * 100 >= mask.size * self.WHITE_THRESHOLD_PERCENT
        if self.on_detection is not None:
            self.on_detection(alert)
            return
        percentage = 100 * white_pixels / mask.size
        if alert and not self.white_alert:
            self.get_logger().info(f'White area reached {self.WHITE_THRESHOLD_PERCENT}% ({percentage:.1f}%).')
        self.white_alert = alert

        status = np.zeros((28, roi.shape[1], 3), dtype=np.uint8)
        label = f'White: {percentage:.1f}%'
        if alert:
            label += f'  |  {self.WHITE_THRESHOLD_PERCENT}% ALERT'
        color = (0, 255, 255) if alert else (255, 255, 255)
        cv2.putText(status, label, (6, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.53, color, 1, cv2.LINE_AA)
        cv2.imshow(self.WINDOW, np.vstack((roi, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), status)))


def main(args=None):
    rclpy.init(args=args)
    node = DetectStopline()
    try:
        cv2.namedWindow(node.WINDOW, cv2.WINDOW_AUTOSIZE)
        for label, initial, maximum in node.HSV_CONTROLS:
            cv2.createTrackbar(label, node.WINDOW, initial, maximum, lambda _: None)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
            if cv2.getWindowProperty(node.WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
