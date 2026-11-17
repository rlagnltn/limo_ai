"""Non-DL controller; the safety node may additionally publish emergency zeros.

mission_command (std_msgs/String, JSON) accepts mode=follow/slow/guide/
obstacles/branch_a/branch_b/cones/pedestrian/traffic/parking/hold.
YOLO slow_sign (class 5) starts the timed speed-bump mission.
Other signs and signals use mission_command. {"signal":"green"} releases a traffic stop.
Parking requires a measured path and parking_box in the entry robot frame.
Geometry/calibration gates must be verified on the real robot before enabling
metric manoeuvres. Never use elapsed time alone as a parking/stop distance.
"""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from dl_ros2_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int32, String
from limo_msgs.msg import LimoStatus
if __package__:
    from .limo_e_stop import clamp, wrap_angle, yaw_of
    from .lane_change import LaneChangePlanner
else:
    from limo_e_stop import clamp, wrap_angle, yaw_of
    from lane_change import LaneChangePlanner


def finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Expected a finite number')
    return float(value)


def local_point(pose, x, y):
    dx, dy = x-pose[0], y-pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c*dx+s*dy, -s*dx+c*dy


def world_point(pose, x, y):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0]+c*x-s*y, pose[1]+s*x+c*y


class LimoControl(Node):
    def __init__(self, **kwargs):
        super().__init__('limo_control', **kwargs)
        defaults = dict(enabled=True, default_speed=0.3, max_speed=0.35,
                        slow_speed=0.2, uncertain_speed=0.12, parking_speed=0.08,
                        p_gain=0.003, d_gain=0.0002, heading_gain=0.25,
                        wheelbase=0.2, max_steering_angle=0.35,
                        robot_front=0.18, robot_rear=0.18, robot_width=0.22,
                        rear_axle_offset=0.1, safety_margin=0.04,
                        acceleration=0.25, deceleration=0.4, reaction_time=0.25,
                        curvature_rate=2.0, sensor_timeout=0.5, vehicle_status_timeout=2.5,
                        lane_hold_time=0.8,
                        lane_hold_distance=0.1, lane_hold_speed=0.08,
                        lookahead=0.8, avoidance_horizon=1.2, lane_change_distance=1.2,
                        guide_distance=1.677, stop_tolerance=0.02,
                        stop_search_distance=1.5, stopped_speed=0.005,
                        stop_confirm_time=0.2,
                        pedestrian_wait=3.2, mission_timeout=15.0, lap_timeout=600.0,
                        geometry_verified=False, auto_obstacle_mode=True,
                        require_path_coverage_for_lane_follow=True,
                        allow_sparse_scan_crawl=False, sparse_scan_speed=0.05,
                        command_control_mode=1, command_mode_settle=0.2,
                        require_ackermann=False, base_frame='base_link',
                        odom_frame='odom', initial_mode='follow', auto_slow_sign=True,
                        branch_approach=0.2, branch_angle=0.7, branch_curvature=1.5,
                        branch_speed=0.08, branch_exit_distance=0.3,
                        branch_yaw_tolerance=0.08, parking_lookahead=0.2,
                        stop_line_margin=0.1)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for k in ('default_speed', 'max_speed', 'slow_speed', 'uncertain_speed',
                  'parking_speed', 'wheelbase', 'max_steering_angle', 'robot_front',
                  'robot_rear', 'robot_width', 'acceleration', 'deceleration',
                  'sensor_timeout', 'vehicle_status_timeout', 'lookahead', 'avoidance_horizon', 'lap_timeout',
                  'curvature_rate', 'lane_change_distance', 'guide_distance',
                  'stop_tolerance', 'stop_search_distance', 'stopped_speed',
                  'stop_confirm_time', 'reaction_time'):
            if not math.isfinite(self.cfg[k]) or self.cfg[k] <= 0:
                raise ValueError('Invalid control parameter: '+k)
        if not 0 < self.cfg['max_steering_angle'] < 0.5:
            raise ValueError('Steering limit must respect the Ackermann mechanism')
        if (not 0 < self.cfg['p_gain'] <= 0.02 or self.cfg['d_gain'] < 0 or
                self.cfg['heading_gain'] < 0):
            raise ValueError('Invalid steering gains')
        if self.cfg['initial_mode'] not in ('follow', 'hold'):
            raise ValueError('Start in follow/hold; arm other missions after sensors are ready')
        if not 3 <= self.cfg['pedestrian_wait'] < self.cfg['mission_timeout'] <= 15:
            raise ValueError('Invalid mission waiting times')
        if (not 0 <= self.cfg['rear_axle_offset'] <= self.cfg['robot_rear'] or
                not 0 <= self.cfg['safety_margin'] <= 0.2 or
                not 0 <= self.cfg['lane_hold_time'] <= 1.0 or
                not 0 < self.cfg['lane_hold_distance'] <= 0.15 or
                not 0 < self.cfg['lane_hold_speed'] <= 0.1):
            raise ValueError('Invalid geometry or lane hold interval')
        if not 0 < self.cfg['sparse_scan_speed'] <= 0.05:
            raise ValueError('Sparse-return test crawl must not exceed 0.05 m/s')
        if (not 0 < self.cfg['parking_lookahead'] <= 1.0 or
                not 0 <= self.cfg['stop_line_margin'] <= 1.0):
            raise ValueError('Invalid parking lookahead/stop margin')
        if (not 0 < self.cfg['branch_angle'] < math.pi or
                not 0 < self.cfg['branch_curvature'] <= math.tan(self.cfg['max_steering_angle'])/self.cfg['wheelbase'] or
                not 0 < self.cfg['branch_speed'] <= self.cfg['max_speed'] or
                not 0 <= self.cfg['branch_approach'] <= 3 or
                not 0 < self.cfg['branch_exit_distance'] <= 3 or
                not 0 < self.cfg['branch_yaw_tolerance'] < self.cfg['branch_angle']):
            raise ValueError('Invalid intersection geometry')
        if (type(self.cfg['command_control_mode']) is not int or
                not 0 <= self.cfg['command_control_mode'] <= 255 or
                not 0 <= self.cfg['command_mode_settle'] <= 1.0):
            raise ValueError('Invalid command-control mode configuration')
        self.max_curvature = math.tan(self.cfg['max_steering_angle'])/self.cfg['wheelbase']
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'mission_status', 10)
        self.subscriptions_ = [
            self.create_subscription(Bool, 'e_stop', self.e_stop_callback, 10),
            self.create_subscription(String, 'lane_state', self.lane_callback, 10),
            self.create_subscription(String, 'obstacle_state', self.obstacle_callback, 10),
            self.create_subscription(String, 'mission_command', self.command_callback, 10),
            self.create_subscription(BoundingBoxes, '/yolo_bounding_boxes', self.yolo_callback, 10),
            self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile_sensor_data),
            self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile_sensor_data),
            self.create_subscription(LimoStatus, 'limo_status', self.vehicle_callback, 10),
            # Compatibility only: distance_y alone never authorizes motion.
            self.create_subscription(Int32, 'distance_y', self.distance_callback, 10),
        ]
        self.lane = {}
        self.obstacles = {}
        self.received = {}
        self.e_stop_flag = True
        self.pose = None
        self.odom_speed = 0.0
        self.travel = 0.0
        self.vehicle_ok = False
        self.vehicle_control_mode = None
        self.command_ready_since = None
        self.odom_fault = False
        self.last_valid_lane = None
        self.last_valid_lane_travel = 0.0
        self.last_tracking_curvature = 0.0
        self.filtered_error = 0.0
        self.previous_error = 0.0
        self.last_speed = 0.0
        self.last_curvature = 0.0
        self.last_tick = time.monotonic()
        self.started_at = None
        self.mode = 'hold'
        self.phase = 'idle'
        self.reason = 'initializing'
        self.hold_reason = 'initial_hold' if self.cfg['initial_mode'] == 'hold' else ''
        self.entered_at = self.last_tick
        self.phase_travel = 0.0
        self.phase_heading = 0.0
        self.blocked_since = None
        self.stop_target = None
        self.stopped_at = None
        self.mission_distance = None
        self.stationary_since = None
        self.signal = ('red', 0.0)
        self.slow_until = 0.0
        self.slow_sign_used = False
        self.slow_sign_until = None
        self.target_lane = None
        self.lane_planner = LaneChangePlanner()
        self.multi_frames = 0
        self.command_ids = deque(maxlen=100)
        self.parking_path = []
        self.parking_box = None
        self.parking_origin = None
        self.parking_index = 0
        self.progress_at, self.progress_travel = self.last_tick, self.travel
        self._enter(self.cfg['initial_mode'], self.last_tick)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def _enter(self, mode, now):
        if hasattr(self, 'lane_planner'):
            self.lane_planner.reset()
        self.slow_sign_until = None
        if mode != 'hold':
            self.hold_reason = ''
        self.mode, self.phase = mode, 'running'
        self.entered_at, self.phase_travel = now, self.travel
        self.phase_heading = self.pose[2] if self.pose else 0.0
        self.blocked_since = None
        self.target_lane = None
        self.stop_target = None
        self.stopped_at = None
        self.mission_distance = None
        self.stationary_since = None
        self.last_tracking_curvature = self.last_curvature
        self.progress_at, self.progress_travel = now, self.travel
        if mode in ('pedestrian', 'traffic'):
            self.phase = 'approach_stop'
            self.signal = ('red', now)
        elif mode in ('branch_a', 'branch_b'):
            self.phase = 'approach'
        elif mode == 'hold':
            self.phase = 'stopped'

    def enter_hold(self, reason, now):
        self._enter('hold', now)
        self.hold_reason = reason
        self.get_logger().warning('Automatic hold: '+reason)

    def distance_callback(self, msg):
        pass  # Rich lane_state validity and freshness are mandatory.

    def e_stop_callback(self, msg):
        self.e_stop_flag = bool(msg.data)
        self.received['e_stop'] = time.monotonic()
        if self.e_stop_flag and getattr(self, 'lidar_enabled', True):
            self.publish_motion(0.0, 0.0)

    def lane_callback(self, msg):
        now = time.monotonic()
        try:
            lane = json.loads(msg.data)
            if not isinstance(lane, dict) or lane.get('version') != 1:
                raise ValueError('Invalid lane schema')
            width = finite_number(lane['width'])
            error = finite_number(lane['offset'])
            if width < 16 or abs(error) > width:
                raise ValueError('Invalid lane geometry')
            if type(lane.get('valid')) is not bool or lane.get('lane_count', 0) not in (0, 1, 3):
                raise ValueError('Invalid lane status')
            if (not 0 <= finite_number(lane.get('confidence', 0.0)) <= 1 or
                    abs(finite_number(lane.get('heading_error', 0.0))) > 1):
                raise ValueError('Invalid lane confidence/heading')
            if 'ground_lines' in lane:
                lines = lane['ground_lines']
                if (not isinstance(lines, list) or len(lines) != 2
                        or any(not isinstance(line, list) or len(line) != 2 for line in lines)):
                    raise ValueError('Invalid ground boundary lines')
                for slope, intercept in lines:
                    if abs(finite_number(slope)) > 2 or abs(finite_number(intercept)) > 5:
                        raise ValueError('Ground boundary outside planning range')
                if abs(finite_number(lane['ground_heading'])) > math.pi/3:
                    raise ValueError('Invalid ground heading')
            for key in ('merge_valid', 'merge_complete'):
                if key in lane and type(lane[key]) is not bool:
                    raise ValueError('Invalid merge status')
            if lane.get('merge_mode') not in (None, 'branch_a', 'branch_b'):
                raise ValueError('Invalid merge direction')
            if lane.get('merge_valid') is True:
                if abs(finite_number(lane['merge_offset'])) > width:
                    raise ValueError('Invalid merge offset')
            for key in ('centers', 'ground_centers'):
                values = lane.get(key, [])
                if not isinstance(values, list) or len(values) > 3:
                    raise ValueError('Invalid lane candidates')
                if key == 'centers':
                    for value in values:
                        if not 0 <= finite_number(value) < width:
                            raise ValueError('Lane centre outside image')
                else:
                    for point in values:
                        if len(point) != 2 or not 0 < finite_number(point[0]) < 5 or abs(finite_number(point[1])) > 3:
                            raise ValueError('Invalid metric lane centre')
            self.lane = lane
            self.received['lane'] = now
            if lane.get('valid') is True:
                self.last_valid_lane = now
                self.last_valid_lane_travel = self.travel
            if lane.get('lane_count') == 3 and lane.get('valid') is True:
                self.multi_frames = min(10, self.multi_frames+1)
            else:
                self.multi_frames = 0
        except (ValueError, KeyError, TypeError, OverflowError):
            self.lane = {}
            self.received.pop('lane', None)

    def obstacle_callback(self, msg):
        try:
            state = json.loads(msg.data)
            if state.get('valid') is not True or state.get('frame') != self.cfg['base_frame']:
                raise ValueError('Invalid obstacle frame/data')
            points, coverage = state['points'], state['coverage']
            if (not 0 < finite_number(state['range_limit']) <= 10 or
                    len(state['sensor_origin']) != 2 or
                    any(abs(finite_number(v)) > 3 for v in state['sensor_origin'])):
                raise ValueError('Invalid scan limits/origin')
            if len(points) > 10000 or len(coverage) != 36:
                raise ValueError('Invalid obstacle packet')
            for point in points:
                if len(point) != 2 or any(abs(finite_number(v)) > 10 for v in point):
                    raise ValueError('Invalid obstacle point')
            for value in coverage:
                if not 0 <= finite_number(value) <= 1:
                    raise ValueError('Invalid coverage')
            self.obstacles = state
            self.obstacle_error = ''
            self.received['obstacles'] = time.monotonic()
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as error:
            self.obstacles = {}
            self.received.pop('obstacles', None)
            try:
                self.obstacle_error = str(state.get('reason', str(error)))[:180]
            except (AttributeError, UnboundLocalError):
                self.obstacle_error = str(error)

    def odom_callback(self, msg):
        now = time.monotonic()
        try:
            if msg.header.frame_id != self.cfg['odom_frame'] or msg.child_frame_id != self.cfg['base_frame']:
                raise ValueError('Odometry frames do not match configuration')
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if stamp and (age > self.cfg['sensor_timeout'] or age < -0.1):
                raise ValueError('Stale odometry')
            pose = (finite_number(msg.pose.pose.position.x),
                    finite_number(msg.pose.pose.position.y), yaw_of(msg.pose.pose.orientation))
            velocity = finite_number(msg.twist.twist.linear.x)
            if self.pose is not None:
                dt = now-self.received.get('odom', now)
                distance = math.hypot(pose[0]-self.pose[0], pose[1]-self.pose[1])
                if distance > self.cfg['max_speed']*max(dt, 0.02)*4+0.1:
                    self.odom_fault = True
                else:
                    self.travel += distance
            self.pose, self.odom_speed = pose, velocity
            self.received['odom'] = now
        except (ValueError, TypeError):
            self.received.pop('odom', None)

    def vehicle_callback(self, msg):
        now = time.monotonic()
        self.vehicle_motion_mode = int(msg.motion_mode)
        self.vehicle_error_code = int(msg.error_code)
        self.vehicle_ok = (msg.error_code == 0 and
                           (not self.cfg['require_ackermann'] or msg.motion_mode == 1))
        self.vehicle_control_mode = int(msg.control_mode)
        if self.vehicle_control_mode == self.cfg['command_control_mode']:
            if self.command_ready_since is None:
                self.command_ready_since = now
        else:
            self.command_ready_since = None
        self.received['vehicle'] = now
        if not self.vehicle_ok or self.command_ready_since is None:
            self.publish_motion(0.0, 0.0)
            self.last_tracking_curvature = 0.0

    def imu_callback(self, msg):
        pitch_rate = msg.angular_velocity.y
        if math.isfinite(pitch_rate) and abs(pitch_rate) > 0.25:
            # This is a bump fallback, not a replacement for slowing BEFORE a sign.
            self.slow_until = time.monotonic()+2.0

    def yolo_callback(self, msg):
        if not self.cfg['auto_slow_sign'] or self.mode != 'follow' or self.slow_sign_used:
            return
        if any(
            box.class_name == 'slow_sign'
            and box.xmax > box.xmin
            and box.ymax > box.ymin
            and (box.xmax - box.xmin) * (box.ymax - box.ymin) >= 3000
            for box in msg.bounding_boxes
        ):
            now = time.monotonic()
            self._enter('slow', now)
            self.slow_sign_used = True
            self.slow_sign_until = now + 10.0
            self.get_logger().info('slow_sign: starting 10-second speed-bump mission')

    def command_callback(self, msg):
        now = time.monotonic()
        self.last_command_error = ''
        try:
            command = json.loads(msg.data)
            if not isinstance(command, dict):
                raise ValueError('Expected command object')
            if 'signal' in command:
                if command['signal'] not in ('red', 'yellow', 'green'):
                    raise ValueError('Invalid traffic signal')
                self.signal = (command['signal'], now)
                return
            mode = command['mode']
            if mode not in ('follow', 'slow', 'guide', 'obstacles', 'branch_a',
                            'branch_b', 'cones', 'pedestrian', 'traffic', 'parking', 'hold'):
                raise ValueError('Unknown mission')
            identifier = command.get('id')
            if identifier is not None:
                if not isinstance(identifier, str) or len(identifier) > 100:
                    raise ValueError('Invalid command id')
                if identifier in self.command_ids:
                    return
            elif mode == self.mode and not any(command.get(k) is True for k in
                                               ('reset_odom_fault', 'reset_lap')):
                return  # Repeated detections must not restart waits or manoeuvres.
            parking = None
            if mode == 'parking':
                if not self.cfg['geometry_verified'] or not self.fresh('odom', now):
                    raise ValueError('Parking requires verified geometry and odometry')
                points, box = command['path'], command['parking_box']
                if not isinstance(points, list) or not 2 <= len(points) <= 100 or len(box) != 4:
                    raise ValueError('Provide measured parking path and rectangle')
                box = [finite_number(v) for v in box]
                if box[1]-box[0] < self.cfg['robot_front']+self.cfg['robot_rear'] or box[3]-box[2] < self.cfg['robot_width']:
                    raise ValueError('Parking box too small')
                path = []
                for point in points:
                    if len(point) != 4:
                        raise ValueError('Path points are [x,y,yaw,signed_speed]')
                    x, y, yaw, speed = [finite_number(v) for v in point]
                    if math.hypot(x, y) > 5 or abs(speed) > self.cfg['parking_speed'] or abs(speed) < 0.02:
                        raise ValueError('Parking waypoint/speed outside limits')
                    wx, wy = world_point(self.pose, x, y)
                    path.append((wx, wy, wrap_angle(yaw+self.pose[2]), speed))
                parking = (path, box, self.pose)
            if mode == 'guide' and not self.fresh('odom', now):
                raise ValueError('Heading hold requires odometry')
            distance = command.get('distance')
            if distance is not None:
                distance = finite_number(distance)
                if not 0 < distance <= 5:
                    raise ValueError('Distance must be in metres, 0 < d <= 5')
                if mode in ('pedestrian', 'traffic') and not self.fresh('odom', now):
                    raise ValueError('Distance-based stopping requires fresh odometry')
            self._enter(mode, now)
            if identifier is not None:
                self.command_ids.append(identifier)
            self.mission_distance = distance
            if mode == 'hold':
                self.hold_reason = 'operator_hold'
                self.publish_motion(0.0, 0.0)
            if parking:
                self.parking_path, self.parking_box, self.parking_origin = parking
                self.parking_index = 0
                self.parking_direction = 1
                self.parking_shift_since = None
            if mode in ('pedestrian', 'traffic') and distance is not None:
                self.stop_target = self.travel+distance
            if command.get('reset_odom_fault') is True and mode == 'hold':
                self.odom_fault = False
            if command.get('reset_lap') is True and mode == 'hold':
                self.started_at = None
                self.slow_sign_used = False
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            self.last_command_error = str(error)
            self.get_logger().warning('Mission command rejected: '+str(error))

    def fresh(self, name, now):
        timeout = self.cfg['vehicle_status_timeout'] if name == 'vehicle' else self.cfg['sensor_timeout']
        return name in self.received and 0 <= now-self.received[name] <= timeout

    def steering_curvature(self, now, dt):
        width = float(self.lane['width'])
        error = float(self.lane['offset'])*640.0/width
        self.filtered_error = 0.65*self.filtered_error+0.35*error
        derivative = clamp((self.filtered_error-self.previous_error)/dt, -500.0, 500.0)
        self.previous_error = self.filtered_error
        heading = clamp(float(self.lane.get('heading_error', 0.0)), -1.0, 1.0)
        angle = clamp(self.cfg['p_gain']*self.filtered_error+self.cfg['d_gain']*derivative+
                      self.cfg['heading_gain']*heading,
                      -self.cfg['max_steering_angle'], self.cfg['max_steering_angle'])
        return math.tan(angle)/self.cfg['wheelbase']

    def curve_clear(self, curvature, horizon, direction=1, road_bounds=None, check_coverage=True):
        """Check the swept rectangle, including overhangs; unknown space is blocked."""
        if not getattr(self, 'lidar_enabled', True):
            return True
        for index in range(21):
            distance = direction*horizon*index/20
            angle = curvature*distance
            if abs(curvature) < 1e-6:
                x, y = distance, 0.0
            else:
                offset = self.cfg['rear_axle_offset']
                x = math.sin(angle)/curvature+offset*(math.cos(angle)-1)
                y = (1-math.cos(angle))/curvature+offset*math.sin(angle)
            if not self.footprint_clear(x, y, angle, direction, road_bounds, check_coverage and index > 2):
                return False
        return True

    def footprint_clear(self, x, y, angle, direction=1, road_bounds=None, check_coverage=True):
        half = self.cfg['robot_width']/2+self.cfg['safety_margin']
        front = self.cfg['robot_front']+self.cfg['safety_margin']
        rear = self.cfg['robot_rear']+self.cfg['safety_margin']
        c, s = math.cos(angle), math.sin(angle)
        if road_bounds is not None:
            for dx, dy in ((front, half), (front, -half), (-rear, half), (-rear, -half)):
                px = x+c*dx-s*dy
                py = y+s*dx+c*dy
                if isinstance(road_bounds[0], (list, tuple)):
                    left = road_bounds[0][0]*px+road_bounds[0][1]
                    right = road_bounds[1][0]*px+road_bounds[1][1]
                else:
                    left, right = road_bounds
                if py > left or py < right:
                    return False
        if check_coverage:
            origin = self.obstacles.get('sensor_origin', [0.0, 0.0])
            coverage = self.obstacles.get('coverage', [0.0]*36)
            edge = front if direction > 0 else -rear
            for lateral in (-half, 0.0, half):
                px, py = x+c*edge-s*lateral, y+s*edge+c*lateral
                dx, dy = px-origin[0], py-origin[1]
                ray = min(35, int((math.atan2(dy, dx)+math.pi)*36/(2*math.pi)))
                if math.hypot(dx, dy) > self.obstacles.get('range_limit', 0.0) or coverage[ray] < 0.6:
                    return False
        for ox, oy in self.obstacles.get('points', []):
            dx, dy = ox-x, oy-y
            if -rear <= c*dx+s*dy <= front and abs(-s*dx+c*dy) <= half:
                return False
        return True

    def lane_change_curvature(self, baseline):
        if not self.cfg['geometry_verified'] or self.lane.get('metric_valid') is not True:
            return None, 'lane_change_needs_calibration'
        return self.lane_planner.command(self)

    def cone_curvature(self):
        # This mode is explicitly armed: objects bounding the gap are assumed cones.
        points = self.obstacles.get('points', [])
        near = [(x, y) for x, y in points if 0.3 < x < 1.5 and abs(y) < 1.5]
        left = [p for p in near if p[1] > 0.08]
        right = [p for p in near if p[1] < -0.08]
        if not left or not right:
            return None, 'cone_corridor_not_observed'
        lp = min(left, key=lambda p: p[1])
        rp = max(right, key=lambda p: p[1])
        bounds = (lp[1], rp[1])
        if bounds[0]-bounds[1] < self.cfg['robot_width']+2*self.cfg['safety_margin']:
            return None, 'cone_gap_too_narrow'
        x, y = max(0.5, (lp[0]+rp[0])/2), (lp[1]+rp[1])/2
        desired = 2*y/((x+self.cfg['rear_axle_offset'])**2+y*y)
        candidates = [desired]+[self.max_curvature*i/8 for i in range(-8, 9)]
        clear = [k for k in candidates if abs(k) <= self.max_curvature and
                 self.curve_clear(k, self.cfg['avoidance_horizon'], road_bounds=bounds)]
        if not clear:
            return None, 'cone_path_blocked'
        return min(clear, key=lambda k: abs(k-desired)+0.2*abs(k-self.last_curvature)), 'cone_gap'

    def parking_control(self):
        if self.parking_origin is None or not self.parking_path:
            return 0.0, 0.0, 'parking_path_missing'
        goal = self.parking_path[self.parking_index]
        direction = 1 if goal[3] > 0 else -1
        if direction != self.parking_direction:
            if abs(self.odom_speed) > self.cfg['stopped_speed']:
                self.parking_shift_since = None
            elif self.parking_shift_since is None:
                self.parking_shift_since = time.monotonic()
            elif time.monotonic()-self.parking_shift_since >= self.cfg['stop_confirm_time']:
                self.parking_direction = direction
                self.parking_shift_since = None
            return 0.0, 0.0, 'parking_direction_change_stop'
        x, y = local_point(self.pose, goal[0], goal[1])
        distance = math.hypot(x, y)
        cusp = (self.parking_index < len(self.parking_path)-1 and
                goal[3]*self.parking_path[self.parking_index+1][3] < 0)
        if distance <= (0.005 if cusp else 0.025):
            if self.parking_index < len(self.parking_path)-1:
                self.parking_index += 1
                return self.parking_control()
            yaw_error = abs(wrap_angle(goal[2]-self.pose[2]))
            inside = True
            for dx in (-self.cfg['robot_rear'], self.cfg['robot_front']):
                for dy in (-self.cfg['robot_width']/2, self.cfg['robot_width']/2):
                    wx, wy = world_point(self.pose, dx, dy)
                    bx, by = local_point(self.parking_origin, wx, wy)
                    box = self.parking_box
                    inside = inside and box[0]+0.005 < bx < box[1]-0.005 and box[2]+0.005 < by < box[3]-0.005
            if inside and yaw_error < 0.12 and abs(self.odom_speed) <= self.cfg['stopped_speed']:
                self.phase = 'complete'
                return 0.0, 0.0, 'parked_four_corners_inside'
            return 0.0, 0.0, 'parking_alignment_or_boundary_error'
        direction = 1 if goal[3] > 0 else -1
        # Track a lookahead point within the current forward/reverse segment.
        # A short first reverse waypoint otherwise asks for an impossible curvature
        # when the forward alignment ends a few millimetres before its target.
        target = goal
        for candidate in self.parking_path[self.parking_index+1:]:
            if candidate[3]*goal[3] < 0 or math.hypot(x, y) >= self.cfg['parking_lookahead']:
                break
            target = candidate
            x, y = local_point(self.pose, target[0], target[1])
        if direction*x < -0.03:
            return 0.0, 0.0, 'parking_waypoint_behind_travel_direction'
        x += self.cfg['rear_axle_offset']
        k = clamp(2*y/max(0.01, x*x+y*y), -self.max_curvature, self.max_curvature)
        speed = direction*min(abs(goal[3]), math.sqrt(2*self.cfg['deceleration']*distance))
        if not self.curve_clear(k, min(0.6, distance+0.1), direction):
            return 0.0, 0.0, 'parking_path_blocked_or_unobserved'
        return speed, k, 'parking_waypoint_%d' % self.parking_index

    def compute_control(self, now, dt):
        if not self.get_parameter('enabled').value:
            return 0.0, 0.0, 'disabled_by_parameter'
        if self.mode == 'hold':
            return 0.0, 0.0, self.hold_reason or 'held'
        if not self.fresh('vehicle', now):
            return 0.0, 0.0, 'vehicle_stale_or_missing'
        if self.vehicle_control_mode != self.cfg['command_control_mode']:
            # RC/manual ownership is an intentional pause, not a failed mission.
            self.blocked_since = None
            self.progress_at, self.progress_travel = now, self.travel
            return 0.0, 0.0, 'waiting_for_command_mode(%s)' % self.vehicle_control_mode
        if (self.command_ready_since is None or
                now-self.command_ready_since < self.cfg['command_mode_settle']):
            return 0.0, 0.0, 'command_mode_settling'
        lidar_enabled = getattr(self, 'lidar_enabled', True)
        sensors = ('odom', 'vehicle', 'lane') + (('e_stop', 'obstacles') if lidar_enabled else ())
        for sensor in sensors:
            if not self.fresh(sensor, now):
                if sensor == 'obstacles' and getattr(self, 'obstacle_error', ''):
                    return 0.0, 0.0, 'obstacles_invalid: '+self.obstacle_error
                return 0.0, 0.0, sensor+'_stale_or_missing'
        if (lidar_enabled and self.e_stop_flag) or not self.vehicle_ok or self.odom_fault:
            return 0.0, 0.0, 'emergency_stop_or_vehicle_fault'
        if self.started_at is not None and now-self.started_at >= self.cfg['lap_timeout']:
            self.enter_hold('lap_time_limit', now)
            return 0.0, 0.0, 'lap_time_limit'
        travelled = self.travel-self.phase_travel
        distance = getattr(self, 'mission_distance', None)
        if (distance is not None and travelled >= distance and
                self.mode in ('slow', 'branch_a', 'branch_b', 'cones', 'obstacles')):
            self._enter('follow', now)
        if self.phase == 'complete':
            return 0.0, 0.0, 'mission_complete'
        if self.mode == 'parking':
            return self.parking_control()
        if self.mode in ('pedestrian', 'traffic'):
            if self.phase == 'waiting':
                if abs(self.odom_speed) > self.cfg['stopped_speed']:
                    self.phase = 'approach_stop'
                    self.stopped_at = self.stationary_since = None
                    return 0.0, 0.0, 'motion_during_required_stop'
                if self.stopped_at is None:
                    return 0.0, 0.0, 'stop_not_confirmed'
                waiting = now-self.stopped_at
                if waiting >= self.cfg['mission_timeout']:
                    self.phase = 'failed'
                    return 0.0, 0.0, 'mission_wait_timeout'
                release = (waiting >= self.cfg['pedestrian_wait'] if self.mode == 'pedestrian'
                           else self.signal[0] == 'green' and now-self.signal[1] < 1.0)
                if not release:
                    return 0.0, 0.0, 'waiting_'+self.mode
                self._enter('follow', now)
                return self.compute_control(now, dt)
            elif self.phase == 'failed':
                return 0.0, 0.0, 'mission_failed'
            if self.stop_target is None:
                distance = self.lane.get('stop_distance')
                if not self.cfg['geometry_verified'] or self.lane.get('metric_valid') is not True:
                    return 0.0, 0.0, 'stop_line_metric_calibration_or_detection_required'
                if distance is not None:
                    self.stop_target = self.travel+max(
                        0.0, finite_number(distance)-self.cfg['robot_front']-self.cfg['stop_line_margin'])
                elif travelled >= self.cfg['stop_search_distance']:
                    self.phase = 'failed'
                    return 0.0, 0.0, 'stop_line_not_found_within_search_distance'
            if self.stop_target is not None:
                remaining = self.stop_target-self.travel
                if remaining <= self.cfg['stop_tolerance']:
                    if remaining < -0.05:
                        self.phase = 'failed'
                        return 0.0, 0.0, 'rear_axle_stop_overshoot'
                    if abs(self.odom_speed) <= self.cfg['stopped_speed']:
                        if self.stationary_since is None:
                            self.stationary_since = now
                        if now-self.stationary_since >= self.cfg['stop_confirm_time']:
                            self.phase, self.stopped_at = 'waiting', now
                    else:
                        self.stationary_since = None
                    return 0.0, 0.0, 'stopping_at_rear_axle_target'
        if self.mode == 'guide':
            length = getattr(self, 'mission_distance', None) or self.cfg['guide_distance']
            if travelled >= length:
                self._enter('follow', now)
            else:
                error = wrap_angle(self.phase_heading-self.pose[2])
                return min(self.cfg['slow_speed'], self.cfg['default_speed']), clamp(
                    2.0*error, -self.max_curvature, self.max_curvature), 'holding_entry_heading'
        if self.mode in ('branch_a', 'branch_b'):
            if not self.cfg['geometry_verified']:
                return 0.0, 0.0, 'intersection_geometry_not_verified'
            sign = 1 if self.mode == 'branch_a' else -1
            if self.phase == 'approach' and travelled >= self.cfg['branch_approach']:
                self.phase = 'turning'
                self.phase_heading = self.pose[2]
            if self.phase == 'turning':
                progress = sign*wrap_angle(self.pose[2]-self.phase_heading)
                if progress > self.cfg['branch_angle']+0.25:
                    self.enter_hold('intersection_yaw_overshoot', now)
                    return 0.0, 0.0, 'intersection_yaw_overshoot'
                if progress >= self.cfg['branch_angle']-self.cfg['branch_yaw_tolerance']:
                    self.phase, self.phase_travel = 'merging', self.travel
                else:
                    return self.cfg['branch_speed'], sign*self.cfg['branch_curvature'], 'intersection_odometry_turn'
            if self.phase == 'merging':
                if self.lane.get('merge_mode') != self.mode or self.lane.get('merge_valid') is not True:
                    return 0.0, 0.0, 'waiting_for_merge_boundary'
                if (self.lane.get('merge_complete') is True and self.lane.get('valid') is True
                        and self.travel-self.phase_travel >= self.cfg['branch_exit_distance']):
                    self._enter('follow', now)
                else:
                    # Keep jihyuk's steering gains/limits, using v1's selected boundary.
                    width = float(self.lane['width'])
                    angle = clamp(self.cfg['p_gain']*self.lane['merge_offset']*640.0/width,
                                  -self.cfg['max_steering_angle'], self.cfg['max_steering_angle'])
                    return self.cfg['branch_speed'], math.tan(angle)/self.cfg['wheelbase'], 'intersection_merge_tracking'
        if self.mode == 'cones':
            curvature, reason = self.cone_curvature()
            return (0.0, 0.0, reason) if curvature is None else (
                min(self.cfg['slow_speed'], self.cfg['default_speed']), curvature, reason)
        if self.lane.get('valid') is not True:
            if self.lane.get('reason') == 'invalid_camera_frame':
                return 0.0, 0.0, 'camera_frame_invalid'
            if (self.last_valid_lane is None or
                    (self.command_ready_since is not None and
                     self.last_valid_lane < self.command_ready_since) or
                    now-self.last_valid_lane > self.cfg['lane_hold_time'] or
                    self.travel-self.last_valid_lane_travel >= self.cfg['lane_hold_distance']):
                return 0.0, 0.0, 'lane_lost'
            return (min(self.cfg['lane_hold_speed'], self.cfg['default_speed']),
                    self.last_tracking_curvature, 'brief_lane_loss_curve_hold')
        baseline = self.steering_curvature(now, dt)
        speed = min(self.cfg['default_speed'], self.cfg['max_speed'])
        speed /= 1+0.9*abs(baseline)
        if float(self.lane.get('confidence', 0.0)) < 0.8:
            speed = min(speed, self.cfg['uncertain_speed'])
        if self.slow_sign_until is not None:
            speed = min(speed, 0.2)
        elif self.mode in ('slow', 'branch_a', 'branch_b', 'pedestrian', 'traffic') or now < self.slow_until:
            speed = min(speed, self.cfg['slow_speed'], self.cfg['default_speed'])
        if self.mode in ('pedestrian', 'traffic') and self.stop_target is None:
            speed = min(speed, 0.08)
        if self.stop_target is not None:
            remaining = max(0.0, self.stop_target-self.travel-self.cfg['stop_tolerance'])
            speed = min(speed, math.sqrt(2*self.cfg['deceleration']*remaining))
        if self.mode == 'follow' and self.cfg['auto_obstacle_mode'] and self.multi_frames >= 4:
            self._enter('obstacles', now)
        if self.mode == 'obstacles':
            curvature, reason = self.lane_change_curvature(baseline)
            if curvature is None:
                return 0.0, 0.0, reason
            else:
                return min(speed, self.cfg['slow_speed'], 0.15), curvature, reason
        return speed, baseline, 'lane_follow'

    def publish_motion(self, speed, curvature):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(speed*curvature) if abs(speed) > 1e-6 else 0.0
        self.publisher_.publish(msg)
        self.last_speed, self.last_curvature = speed, curvature

    def timer_callback(self):
        now = time.monotonic()
        dt = clamp(now-self.last_tick, 0.01, 0.2)
        self.last_tick = now
        try:
            if self.slow_sign_until is not None and now >= self.slow_sign_until:
                self._enter('follow', now)
                self.get_logger().info('Speed-bump mission complete: resuming lane following')
            speed, curvature, reason = self.compute_control(now, dt)
            if not all(math.isfinite(v) for v in (speed, curvature)):
                raise ValueError('Nonfinite control output')
            speed = clamp(speed, -self.cfg['parking_speed'], self.cfg['max_speed'])
            curvature = clamp(curvature, -self.max_curvature, self.max_curvature)
            # Optional single-lane policy: retain measured-hit/braking checks,
            # but do not mistake missing returns for a measured obstacle.
            # Three-lane detections require coverage immediately, even before
            # auto_obstacle_mode has accumulated enough frames to switch modes.
            partial_lane_scan = (
                not self.cfg['require_path_coverage_for_lane_follow'] and speed > 0 and
                self.mode in ('follow', 'slow') and
                self.lane.get('valid') is True and self.lane.get('lane_count') == 1 and
                self.multi_frames == 0)
            sparse = (self.cfg['allow_sparse_scan_crawl'] and speed > 0 and
                      self.mode in ('follow', 'slow') and
                      self.lane.get('lane_count') == 1 and self.multi_frames == 0 and
                      (self.lane.get('valid') is True or reason == 'brief_lane_loss_curve_hold') and
                      self.obstacles.get('valid') is True and
                      self.obstacles.get('scan_health', 0.0) >= 0.3 and
                      self.obstacles.get('front_coverage', 0.0) >= 0.05)
            if speed != 0:
                if self.last_speed*speed < 0 or self.odom_speed*speed < -0.0004:
                    speed, curvature, reason = 0.0, 0.0, 'direction_change_wait_for_stop'
            if speed != 0:
                curvature = clamp(curvature, self.last_curvature-self.cfg['curvature_rate']*dt,
                                  self.last_curvature+self.cfg['curvature_rate']*dt)
                # Slowing down is immediate; acceleration is limited.
                if abs(speed) > abs(self.last_speed):
                    speed = math.copysign(min(abs(speed), abs(self.last_speed)+self.cfg['acceleration']*dt), speed)
                braking_speed = max(abs(speed), abs(self.odom_speed))
                horizon = (braking_speed*self.cfg['reaction_time'] +
                           braking_speed*braking_speed/(2*self.cfg['deceleration'])+0.12)
                direction = 1 if speed > 0 else -1
                if not self.curve_clear(curvature, horizon, direction):
                    # Test the actual swept footprint, not just four central bins.
                    # A measured collision must never be treated as missing coverage.
                    if not self.curve_clear(curvature, horizon, direction, check_coverage=False):
                        speed, curvature, reason = 0.0, 0.0, 'swept_path_obstacle'
                    elif partial_lane_scan:
                        # compute_control has already checked sensor health,
                        # freshness, vehicle ownership, lane validity and e-stop.
                        # Unknown space remains unknown; only measured hits are
                        # checked here. No speed, turn or acceleration limits change.
                        reason = 'partial_scan: '+reason
                    elif sparse:
                        speed = min(speed, self.cfg['sparse_scan_speed'])
                        reason = 'sparse_scan_crawl: '+reason
                    else:
                        speed, curvature, reason = 0.0, 0.0, 'swept_path_unobserved'
            if speed == 0:
                curvature = 0.0
                if (self.mode != 'hold' and self.phase not in ('complete', 'waiting') and
                        (self.started_at is not None or self.mode != 'follow') and
                        not reason.startswith('waiting_for_command_mode')):
                    if self.blocked_since is None:
                        self.blocked_since = now
                    elif now-self.blocked_since > self.cfg['mission_timeout']:
                        self.enter_hold('no_progress_timeout: '+reason, now)
                        reason = self.hold_reason
            else:
                self.blocked_since = None
                if self.started_at is None:
                    self.started_at = now
                    self.progress_at = now
                if self.travel-self.progress_travel >= 0.01:
                    self.progress_at, self.progress_travel = now, self.travel
                elif now-self.progress_at > self.cfg['mission_timeout']:
                    self.enter_hold('odometry_no_progress', now)
                    speed, curvature, reason = 0.0, 0.0, 'odometry_no_progress'
            self.publish_motion(speed, curvature)
            if (speed > 0 and self.lane.get('valid') is True and
                    self.mode in ('follow', 'slow', 'branch_a', 'branch_b')):
                self.last_tracking_curvature = curvature
            self.reason = reason
        except (ValueError, KeyError, TypeError, OverflowError, ZeroDivisionError) as error:
            self.publish_motion(0.0, 0.0)
            self.reason = 'invalid_state: '+str(error)
        status = String()
        status.data = json.dumps(dict(version=1, mode=self.mode, phase=self.phase, reason=self.reason,
                                     speed=self.last_speed, curvature=self.last_curvature,
                                     target_lane=self.target_lane,
                                     hold_reason=self.hold_reason,
                                     control_mode=self.vehicle_control_mode,
                                     odometry_speed=self.odom_speed,
                                     travel=self.travel,
                                     geometry_verified=self.cfg['geometry_verified']),
                                 allow_nan=False, separators=(',', ':'))
        self.status_publisher.publish(status)

    def destroy_node(self):
        self.publish_motion(0.0, 0.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LimoControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
