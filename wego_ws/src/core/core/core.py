"""Active mission lifecycle and the sole final motor-command publisher."""
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from dl_ros2_msgs.msg import BoundingBoxes

from .behaviors.detect_stopline import DetectStopline
from .behaviors.limo_control import LimoControl
from .configuration import ros_args
from .course import Course, DEFAULTS, STAGES
from .detections import SignTracker
from .missions.base import Observation
from .lidar_lifecycle import requires_lidar


class Core(Node):
    def __init__(self, controller, parking_node):
        super().__init__('core')
        self.controller = controller
        self.parking_node = parking_node
        self.parking_command = Twist()
        self.last_parking_command = -math.inf
        defaults = dict(DEFAULTS, enabled=True, start_stage=1, stop_after_stage=False, input_timeout=0.5,
                        detection_timeout=1.0, sign_timeout=0.5, min_confidence=0.7,
                        class_min_areas=[2900.0, 2700.0, 4400.0, 4400.0,
                                         4100.0, 1800.0, 1800.0, 1800.0],
                        confirm_frames=3)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for key in ('input_timeout', 'detection_timeout', 'sign_timeout'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if (not 0 <= self.cfg['min_confidence'] <= 1 or self.cfg['confirm_frames'] < 1
                or len(self.cfg['class_min_areas']) != 8
                or any(not math.isfinite(v) or v < 0 for v in self.cfg['class_min_areas'])
                or not 1 <= self.cfg['start_stage'] <= 8):
            raise ValueError('Invalid class thresholds/start_stage')
        self.cfg['end_stage'] = self.cfg['start_stage'] if self.cfg['stop_after_stage'] else 8
        self.course = Course(self.cfg)
        self.course.index = STAGES.index(self.cfg['start_stage'])
        self.cone_sign_areas = self.cfg['class_min_areas'].copy()
        self.cone_sign_areas[5] = self.cfg['cone_slow_sign_min_area']
        self.obstacle_sign_areas = self.cfg['class_min_areas'].copy()
        self.obstacle_sign_areas[2:4] = [self.cfg['obstacle_direction_min_area']]*2
        self.tracker = SignTracker(self.cfg['min_confidence'], self.cfg['class_min_areas'],
                                   self.cfg['confirm_frames'], self.cfg['sign_timeout'])
        self.last_boxes = self.last_command = self.last_estop = -math.inf
        self.command = Twist()
        self.stop_line_seen_at = None
        self.estop = True
        self.fault = ''
        self.reason = 'waiting_for_inputs'
        self.last_blockers = None
        self.last_blocker_log = -math.inf
        self.lidar_actual = None
        self.lidar_state_received = -math.inf
        self.lidar_pending = True
        self.lidar_reason = 'waiting_for_scanner_controller'
        controller.lidar_enabled = requires_lidar(self.course.active)
        self.lidar_request = self.create_publisher(Bool, '/core/lidar_enabled', 10)
        self.create_subscription(String, '/core/lidar_state', self.on_lidar_state, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/core/status', 10)
        self.mission_events = self.create_publisher(
            String, '/core/events', QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.event_sequence = 0
        self.event_log_path = str(Path.home() / '.ros/log' /
                                  ('mission_events_'+time.strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}.jsonl'))
        self.get_logger().info('Mission event log: '+self.event_log_path)
        self.create_subscription(Twist, '/core/control_cmd', self.on_command, 10)
        self.create_subscription(Twist, 'mission/parking/cmd_vel', self.on_parking_command, 10)
        self.create_subscription(Bool, '/e_stop', self.on_estop, 10)
        self.create_subscription(BoundingBoxes, '/yolo_bounding_boxes', self.on_boxes,
                                 qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

    def on_stop_line(self, detected):
        self.stop_line_seen_at = time.monotonic() if detected else None

    def on_command(self, msg):
        self.command, self.last_command = msg, time.monotonic()

    def on_parking_command(self, msg):
        self.parking_command, self.last_parking_command = msg, time.monotonic()

    def on_estop(self, msg):
        self.estop, self.last_estop = msg.data, time.monotonic()

    def on_boxes(self, msg):
        now = time.monotonic()
        areas_by_mission = {'obstacle': self.obstacle_sign_areas,
                            'cone': self.cone_sign_areas}
        areas = areas_by_mission.get(self.course.active.name)
        self.tracker.update(msg.bounding_boxes, now, areas)
        self.last_boxes = now

    def on_lidar_state(self, msg):
        try:
            state = json.loads(msg.data)
            if state.get('enabled') is not None and type(state['enabled']) is not bool:
                raise ValueError('Invalid scanner state')
            self.lidar_actual = state.get('enabled')
            self.lidar_pending = state.get('pending') is not False
            self.lidar_reason = str(state.get('reason', 'unknown'))
            self.lidar_state_received = time.monotonic()
        except (ValueError, TypeError, KeyError, AttributeError):
            self.lidar_actual = None

    def update_lidar_request(self):
        wanted = (requires_lidar(self.course.active) and not self.course.finished
                  and not self.fault and self.get_parameter('enabled').value)
        if wanted != self.controller.lidar_enabled:
            self.controller.lidar_enabled = wanted
            self.last_command = -math.inf
            if wanted:
                for key in ('obstacles', 'e_stop'):
                    self.controller.received.pop(key, None)
                self.last_estop = -math.inf
        self.lidar_request.publish(Bool(data=wanted))
        return wanted

    def dispatch(self, command):
        if command is None:
            return
        try:
            mode = command.get('mode')
            if mode:
                self.controller.cfg['auto_obstacle_mode'] = (
                    self.course.active.name == 'obstacle' and mode != 'hold'
                    and self.course.active.phase == 'avoiding')
                if mode == self.controller.mode:
                    return
            self.controller.command_callback(String(data=json.dumps(command)))
            if mode:
                self.last_command = -math.inf
                if self.controller.mode != mode:
                    detail = getattr(self.controller, 'last_command_error', '') or 'see /mission_status'
                    raise ValueError(f'Controller rejected {mode}: {detail}')
        except (ValueError, TypeError, KeyError) as error:
            self.fail(str(error))

    def fail(self, reason):
        if not self.fault:
            self.get_logger().error(reason)
        self.fault = reason
        if not self.course.fault and not self.course.finished:
            self.course.fault = reason
            self.course.record('failed', reason, Observation(time.monotonic(), self.controller.travel))
        self.controller.command_callback(String(data='{"mode":"hold"}'))
        self.last_command = -math.inf

    def report_mission_events(self):
        labels = {'waiting': '대기', 'active': '진행', 'completed': '완료 판정', 'failed': '실패/정지'}
        reasons = {
            'slow_duration_completed': '방지턱 서행 설정 시간 완료',
            'guide_distance_completed': '서행 유도선 설정 거리 통과',
            'slow_stop_line_detected': '방지턱 서행 시작 정지선 확인',
            'guide_stop_line_detected': '서행 구간 시작 정지선 확인',
            'pedestrian_stop_and_wait_completed': '보행자 정지 및 대기 완료',
            'waiting_for_stop_line': '차선 추종 중 정지선 대기',
            'obstacle_stage_entered': '3차선 장애물 회피 시작',
            'direction_sign_obstacle_exit': '갈림길 표지판 확인·장애물 미션 종료·방향 전달',
            'branch_cones_confirmed': '갈림길 조향 완료 후 같은 영상에서 대형 라바콘 확인',
            'cone_exit_stop_completed': '라바콘 통로 종료 후 정지 대기 완료',
            'cone_slow_sign_confirmed': '서행 표지판 확인·라바콘 미션 종료',
            'cone_slow_started': '라바콘 통과 후 서행 시작',
            'cone_slow_traffic_light_confirmed': '신호등 확인·서행 종료·신호 전달',
            'green_pixel_threshold_departure': '초록불 면적 기준 충족/차선 추종 복귀',
            't_parking_completed': '전면 T주차 노드 완료·정지 명령',
        }
        for event in getattr(self.course, 'events', []):
            if event['sequence'] <= self.event_sequence:
                continue
            payload = dict(event, timestamp=time.strftime('%Y-%m-%d %H:%M:%S'))
            text = json.dumps(payload, ensure_ascii=False)
            self.mission_events.publish(String(data=text))
            message = '[미션 %s/8 %s] %s | %s | %.2fm / %.1fs' % (
                event['stage'], event['title'], labels[event['state']],
                reasons.get(event['reason'], event['reason']), event['distance_m'], event['elapsed_sec'])
            if event['state'] == 'failed':
                self.get_logger().error(message)
            else:
                self.get_logger().info(message)
            try:
                path = Path(self.event_log_path)
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open('a', encoding='utf-8') as stream:
                    stream.write(text+'\n')
            except OSError as error:
                self.get_logger().warning('Mission event file unavailable: '+str(error))
            self.event_sequence = event['sequence']

    def tick(self):
        now = time.monotonic()
        control = self.controller
        lidar_enabled = self.update_lidar_request()
        active = self.course.active
        parking_active = active.name == 'parking' and active.phase == 'parking'
        parking = self.parking_node.logic if parking_active else None
        blockers = []
        if not self.get_parameter('enabled').value:
            blockers.append('core_disabled')
        if self.fault:
            blockers.append(self.fault)
        if self.lidar_actual is not lidar_enabled or self.lidar_pending:
            blockers.append('lidar_transition: '+self.lidar_reason)
        if now-self.lidar_state_received > self.cfg['input_timeout']:
            blockers.append('no_lidar_motor_state')
        if lidar_enabled and self.estop:
            blockers.append('lidar_emergency_stop')
        if lidar_enabled and now-self.last_estop > self.cfg['input_timeout']:
            blockers.append('no_estop_heartbeat')
        if now-self.last_boxes > self.cfg['detection_timeout']:
            blockers.append('no_yolo_detections_heartbeat')
        sensors = ('odom', 'vehicle')
        if not parking_active:
            sensors += ('lane',)
        if lidar_enabled:
            sensors += ('obstacles', 'e_stop')
        for key in sensors:
            if not control.fresh(key, now):
                blockers.append(key+'_stale_or_missing')
        if not control.vehicle_ok:
            if getattr(control, 'vehicle_error_code', 0):
                blockers.append('vehicle_error_code(%s)' % control.vehicle_error_code)
            elif control.cfg['require_ackermann']:
                blockers.append('ackermann_mode_required(%s)' %
                                getattr(control, 'vehicle_motion_mode', 'unknown'))
            else:
                blockers.append('vehicle_status_not_ready')
        if control.odom_fault:
            blockers.append('odometry_fault')
        if control.vehicle_control_mode != control.cfg['command_control_mode']:
            blockers.append('waiting_for_command_mode(%s)' % control.vehicle_control_mode)
        if parking_active and control.vehicle_motion_mode != 1:
            blockers.append('parking_ackermann_mode_required')
        healthy = not blockers
        if blockers != self.last_blockers and now-self.last_blocker_log >= 2:
            if blockers:
                self.get_logger().warning('Drive blocked: '+', '.join(blockers))
            else:
                self.get_logger().info('All driving inputs ready')
            self.last_blockers, self.last_blocker_log = blockers, now
        if control.mode == 'hold' and control.hold_reason not in ('', 'initial_hold', 'operator_hold'):
            self.fail(control.hold_reason)
            healthy = False
        if not healthy and self.course.active.name == 'pedestrian' and self.course.active.phase == 'restarting':
            self.course.active.restart_at = None
        if healthy and not self.course.finished:
            if parking is not None and parking.state == 'WAIT':
                parking_now = self.parking_node._now()
                if (parking.pose_t is not None and parking.scan_t is not None
                        and parking_now-parking.pose_t <= parking.cfg.odom_timeout_sec
                        and parking_now-parking.scan_t <= parking.cfg.scan_timeout_sec):
                    self.last_parking_command = -math.inf
                    self.parking_node._start_cb(Bool(data=True))
            stop_line_seen_at = self.stop_line_seen_at
            if (stop_line_seen_at is not None and
                    now-stop_line_seen_at > self.cfg['input_timeout']):
                stop_line_seen_at = None
            obs = Observation(now, control.travel, self.tracker.signs(now), control.lane,
                              tuple(control.obstacles.get('points', [])) if lidar_enabled else (), control.mode,
                              control.phase, control.odom_speed, control.reason,
                              large_cone_count=(self.tracker.large_cone_count
                                                if now-self.last_boxes <= self.cfg['sign_timeout']
                                                else 0),
                              parking_state=parking.state if parking else 'WAIT',
                              parking_reason=parking.reason if parking else '',
                              stop_line_seen_at=stop_line_seen_at)
            epoch = self.course.epoch
            result = self.course.tick(obs)
            self.reason = result.reason
            if result.fault:
                self.fail(result.fault)
            else:
                self.dispatch(result.command)
            if self.course.epoch != epoch:
                self.tracker.reset()
                self.get_logger().info(f'Mission {STAGES[self.course.index]}: {self.course.active.name}')
            # Switch motor demand before any command can escape into the next mission.
            after = self.update_lidar_request()
            if after != lidar_enabled or self.course.finished:
                healthy = False
        Core.report_mission_events(self)
        output = Twist()
        cone_ready = (control.mode != 'cones' or
                      (control.phase == 'cone_tracking' and
                       now-control.cone_scan_time <= 0.3))
        if healthy and not self.fault and not self.course.finished:
            if parking is not None:
                candidate = self.parking_command
                if (parking.state in ('APPROACH', 'SEARCH', 'TURN', 'ENTER')
                        and now-self.last_parking_command <= self.cfg['input_timeout']
                        and all(math.isfinite(v) for v in (candidate.linear.x, candidate.angular.z))):
                    output = candidate
            elif (cone_ready and control.mode != 'hold' and now-self.last_command <= self.cfg['input_timeout']
                  and all(math.isfinite(v) for v in (self.command.linear.x, self.command.angular.z))):
                output = self.command
        self.publisher.publish(output)
        if control.phase == 'branch_fixed_steering':
            control.record_fixed_branch_output(now, output)
        progress = self.course.progress(Observation(now, control.travel)) if hasattr(self.course, 'progress') else {}
        self.status.publish(String(data=json.dumps(dict(
            stage=STAGES[self.course.index], mission=self.course.active.name,
            phase=self.course.active.phase, finished=self.course.finished,
            ready=healthy, fault=self.fault, reason=self.reason,
            controller_mode='parking' if parking else control.mode,
            controller_phase=parking.state.lower() if parking else control.phase,
            controller_reason=parking.reason if parking else control.reason, blockers=blockers,
            vehicle_motion_mode=getattr(control, 'vehicle_motion_mode', None),
            vehicle_control_mode=control.vehicle_control_mode,
            lidar_required=control.lidar_enabled, lidar_scanning=self.lidar_actual,
            lidar_reason=self.lidar_reason, progress=progress,
            mission_results=self.course.summary() if hasattr(self.course, 'summary') else [],
            last_event=self.course.events[-1] if getattr(self.course, 'events', []) else None,
            event_log_path=getattr(self, 'event_log_path', ''),
            commanded_speed_mps=output.linear.x, measured_speed_mps=control.odom_speed))))

    def destroy_node(self):
        self.publisher.publish(Twist())
        self.lidar_request.publish(Bool(data=False))
        return super().destroy_node()


def main(args=None):
    from jetracer_ros2.mission_parking_node import MissionParkingNode

    rclpy.init(args=ros_args(args))
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        controller = LimoControl(
            cli_args=['--ros-args', '-r', 'cmd_vel:=/core/control_cmd'],
            parameter_overrides=[Parameter('auto_slow_sign', value=False),
                                 Parameter('auto_obstacle_mode', value=False)])
        nodes.append(controller)
        parking = MissionParkingNode()
        nodes.append(parking)
        core = Core(controller, parking)
        nodes.append(core)
        nodes.append(DetectStopline(
            on_detection=core.on_stop_line,
            enabled=lambda: (core.course.waiting_for_stop_line
                             and not core.course.fault and not core.fault),
        ))
        for node in nodes:
            executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in reversed(nodes):
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
