import os
import sys
import time
import numpy as np
np.bool_ = bool
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
from ultralytics import YOLO
from dl_ros2_msgs.msg import BoundingBox, BoundingBoxes

os.environ['LD_PRELOAD'] = '/usr/lib/aarch64-linux-gnu/libgomp.so.1'

class YoloObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detection_node')
        
        self.declare_parameter('model_weight_path',
                               '/home/wego/wego_ws/src/dl_ros2_application/weight/testv5.engine')
        self.declare_parameter('canonical_class_names', [''])
        self.canonical_names = self.get_parameter('canonical_class_names').value
        if self.canonical_names != [''] and len(self.canonical_names) != 8:
            raise ValueError('canonical_class_names must contain training IDs 0..7')
        model_path = self.get_parameter('model_weight_path').get_parameter_value().string_value

        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f"Model path not provided or file not found: {model_path}")
            sys.exit(1)

        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"YOLO model loaded from {model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {str(e)}")
            sys.exit(1)

        self.tmp_image = None
        self.image_received_at = None
        self.box_publisher = self.create_publisher(BoundingBoxes, 'yolo_bounding_boxes', 10)

        self.yolo_publisher = self.create_publisher(Image, 'yolo', 10)
        self.yolo_compressed_publisher = self.create_publisher(CompressedImage, 'yolo/compressed', 10)

        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def image_callback(self, msg):
        try:
            # JPEG/PNG decoding does not need cv_bridge's native GDAL/TIFF chain.
            image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Invalid compressed camera image')
            self.tmp_image = image
            self.image_received_at = time.monotonic()
        except Exception as e:
            self.get_logger().error(f"Failed to decode compressed image: {str(e)}")

    def timer_callback(self):
        if (self.tmp_image is None or self.image_received_at is None
                or time.monotonic()-self.image_received_at > 0.5):
            return
        
        image = self.tmp_image.copy()
        b_boxes = BoundingBoxes()
        
        try:
            results = self.model(image, verbose=False)
            result = results[0]

            image_annotated = image.copy()
            
            if result.boxes is not None and result.boxes.xywh is not None:
                # self.get_logger().info(f'Detected {len(result.boxes.xywh)} objects')
                
                xywh = result.boxes.xywh.cpu().numpy()
                confs = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()

                for i in range(len(xywh)):
                    if confs[i] < 0.5:
                        continue
                    
                    x, y, w, h = xywh[i]
                    identifier = int(classes[i])
                    class_name = (self.canonical_names[identifier]
                                  if len(self.canonical_names) == 8 and 0 <= identifier < 8
                                  else self.model.names[identifier])
                    
                    bbox = BoundingBox()
                    bbox.class_name = class_name
                    bbox.probability = float(confs[i])
                    bbox.xmin = int(x - w / 2)
                    bbox.ymin = int(y - h / 2)
                    bbox.xmax = int(x + w / 2)
                    bbox.ymax = int(y + h / 2)
                    b_boxes.bounding_boxes.append(bbox)
                    
                    width_px = max(0, bbox.xmax - bbox.xmin)
                    height_px = max(0, bbox.ymax - bbox.ymin)
                    area_px = width_px * height_px
                    label = (f"{class_name} {confs[i]:.0%} | {area_px} px2 "
                             f"({width_px}x{height_px})")
                    self.plot_one_box([bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax], image_annotated, label=label)
            
            else:
                self.get_logger().info('No objects detected')
            
            b_boxes.header.stamp = self.get_clock().now().to_msg()
            self.box_publisher.publish(b_boxes)
            
            out_msg = Image()
            out_msg.header = b_boxes.header
            out_msg.height, out_msg.width = image_annotated.shape[:2]
            out_msg.encoding = 'bgr8'
            out_msg.is_bigendian = False
            out_msg.step = out_msg.width * 3
            out_msg.data = image_annotated.tobytes()
            self.yolo_publisher.publish(out_msg)
            
            encoded_ok, encoded = cv2.imencode('.jpg', image_annotated)
            if encoded_ok:
                yolo_compressed_msg = CompressedImage()
                yolo_compressed_msg.header = b_boxes.header
                yolo_compressed_msg.format = 'jpeg'
                yolo_compressed_msg.data = encoded.tobytes()
                self.yolo_compressed_publisher.publish(yolo_compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in processing: {str(e)}')
            
        finally:
            self.tmp_image = None

    def plot_one_box(self, x, img, color=(255, 0, 0), label=None, line_thickness=None):
        tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            label_x = max(0, min(c1[0], img.shape[1] - t_size[0] - 1))
            label_y = max(t_size[1] + 4, min(c1[1], img.shape[0] - 1))
            cv2.rectangle(img, (label_x, label_y - t_size[1] - 4),
                          (label_x + t_size[0], label_y), color, -1, cv2.LINE_AA)
            cv2.putText(
                img, label, (label_x, label_y - 2), 0, tl / 3, [225, 255, 255],
                thickness=tf, lineType=cv2.LINE_AA
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running node: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
