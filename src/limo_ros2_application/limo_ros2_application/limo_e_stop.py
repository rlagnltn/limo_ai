import rclpy
import signal
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from std_msgs.msg import Bool 
from sensor_msgs.msg import LaserScan
from math import pi, cos, sin

class LimoEStop(Node):
    def __init__(self):
        super().__init__('limo_e_stop')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Bool, 'e_stop', 10)
        
    def laser_callback(self, msg):
        estop = Bool()
        estop.data = False

        for i, data in enumerate(msg.ranges):
            current_angle = msg.angle_min + msg.angle_increment*i
            cx = data * cos(current_angle)
            cy = data * sin(current_angle)
            if 0.01 < cx <0.2 and -0.1 < cy < 0.1:
                estop.data = True
                break

        self.publisher_.publish(estop)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)

    limo_e_stop = LimoEStop()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(limo_e_stop, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        limo_e_stop.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
