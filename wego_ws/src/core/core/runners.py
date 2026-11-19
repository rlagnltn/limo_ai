import rclpy
import json
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.signals import SignalHandlerOptions
from .configuration import ros_args


def run(cls, args=None):
    rclpy.init(args=ros_args(args), signal_handler_options=SignalHandlerOptions.NO)
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def lane(args=None):
    from .behaviors.detect_line import DetectLine
    run(DetectLine, args)


def team_lane():
    from jetracer_ros2 import simple_lane

    config = Path(get_package_share_directory('core')) / 'config'
    simple_lane.TUNE_FILE = str(config / 'simple_lane_tune_v9.json')
    simple_lane.run_ros('drive', '/core/lane_cmd', observe_m8=False, show_tuning=False)


def perception(args=None):
    from .behaviors.limo_e_stop import LimoEStop
    from std_srvs.srv import SetBool
    from std_msgs.msg import Bool, String
    from .lidar_lifecycle import ScanLease

    class CoursePerception(LimoEStop):
        def __init__(self):
            super().__init__()
            client = self.create_client(SetBool, '/set_scan_enabled')
            self.scan_clients = {'start': client, 'stop': client}
            self.scan_lease = ScanLease()
            self.scan_future = None
            self.scan_started = 0.0
            self.scan_reason = 'waiting_for_stop_service'
            self.scan_state = self.create_publisher(String, '/core/lidar_state', 10)
            self.create_subscription(Bool, '/core/lidar_enabled', self.on_lidar_request, 10)
            self.create_timer(0.1, self.manage_scan)

        def on_lidar_request(self, msg):
            self.scan_lease.receive(msg.data, time.monotonic())

        def manage_scan(self):
            now = time.monotonic()
            lease = self.scan_lease
            if self.scan_future is not None:
                if self.scan_future.done():
                    try:
                        response = self.scan_future.result()
                        if not response.success:
                            raise RuntimeError(response.message)
                        lease.complete()
                        self.scan_reason = 'scanning' if lease.actual else 'motor_stopped'
                        self.get_logger().info('LiDAR: '+self.scan_reason)
                    except Exception as error:
                        lease.fail(now)
                        self.scan_reason = str(error)
                    self.scan_future = None
                elif now-self.scan_started > 5.0:
                    self.scan_future.cancel()
                    self.scan_future = None
                    lease.fail(now)
                    self.scan_reason = 'scan_service_timeout'
            command = lease.next_command(now)
            if command:
                client = self.scan_clients[command]
                if client.service_is_ready():
                    if command == 'start':
                        # Require new observations after motor restart.
                        self.last_scan = None
                        self.stop = True
                        self.clear_count = 0
                    lease.begin(command)
                    self.scan_started = now
                    self.scan_future = client.call_async(SetBool.Request(data=command == 'start'))
                    self.scan_reason = command+'_requested'
                else:
                    self.scan_reason = 'waiting_for_'+command+'_service'
            self.scan_state.publish(String(data=json.dumps(dict(
                enabled=lease.actual, requested=lease.desired(now),
                pending=lease.pending is not None, reason=self.scan_reason))))

        def destroy_node(self):
            # Best effort immediate stop; lease expiration also stops scanning when core exits.
            client = self.scan_clients['stop']
            if rclpy.ok() and client.service_is_ready():
                future = client.call_async(SetBool.Request(data=False))
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            return super().destroy_node()

    run(CoursePerception, args)
