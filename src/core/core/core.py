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

from .behaviors.limo_control import LimoControl
from .configuration import ros_args
from .course import Course, DEFAULTS
from .detections import SignTracker
from .missions.base import Observation
from .lidar_lifecycle import LIDAR_MISSIONS


class Core(Node):
    def __init__(self, controller):
        super().__init__('core')
        self.controller = controller
        defaults = dict(DEFAULTS, enabled=True, start_stage=1, stop_after_stage=False, input_timeout=0.5,
                        detection_timeout=1.0, sign_timeout=0.5, min_confidence=0.7,
                        class_min_areas=[2900.0, 2700.0, 1800.0, 1800.0,
                                         1800.0, 1800.0, 1800.0, 1800.0],
                        confirm_frames=3, active_timeout=120.0)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for key in ('input_timeout', 'detection_timeout', 'sign_timeout', 'active_timeout'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if (not 0 <= self.cfg['min_confidence'] <= 1 or self.cfg['confirm_frames'] < 1
                or len(self.cfg['class_min_areas']) != 8
                or any(not math.isfinite(v) or v < 0 for v in self.cfg['class_min_areas'])
                or not 1 <= self.cfg['start_stage'] <= 8):
            raise ValueError('Invalid class thresholds/start_stage')
        for key in ('parking_speed', 'rear_axle_offset', 'wheelbase', 'max_steering_angle'):
            self.cfg[key] = controller.cfg[key]
        self.cfg['end_stage'] = self.cfg['start_stage'] if self.cfg['stop_after_stage'] else 8
        self.course = Course(self.cfg)
        self.course.index = self.cfg['start_stage']-1
        self.tracker = SignTracker(self.cfg['min_confidence'], self.cfg['class_min_areas'],
                                   self.cfg['confirm_frames'], self.cfg['sign_timeout'])
        self.last_boxes = self.last_command = self.last_estop = -math.inf
        self.command = Twist()
        self.estop = True
        self.fault = ''
        self.reason = 'waiting_for_inputs'
        self.mission_elapsed = 0.0
        self.previous_tick = time.monotonic()
        self.last_blockers = None
        self.last_blocker_log = -math.inf
        self.lidar_actual = None
        self.lidar_state_received = -math.inf
        self.lidar_pending = True
        self.lidar_reason = 'waiting_for_scanner_controller'
        controller.lidar_enabled = self.course.active.name in LIDAR_MISSIONS
        self.lidar_latched = self.course.active.name == 'obstacle'
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
        self.create_subscription(Bool, '/e_stop', self.on_estop, 10)
        self.create_subscription(BoundingBoxes, '/yolo_bounding_boxes', self.on_boxes,
                                 qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

    def on_command(self, msg):
        self.command, self.last_command = msg, time.monotonic()

    def on_estop(self, msg):
        self.estop, self.last_estop = msg.data, time.monotonic()

    def on_boxes(self, msg):
        now = time.monotonic()
        self.tracker.update(msg.bounding_boxes, now)
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
        active = self.course.active.name in LIDAR_MISSIONS
        if self.course.active.name == 'obstacle':
            self.lidar_latched = True
        elif self.course.active.name == 'cone':
            self.lidar_latched = False
        signs = self.tracker.signs(time.monotonic())
        if (getattr(self, 'lidar_latched', False) and not active and (2 in signs) != (3 in signs)
                and Core.lidar_front_clear(self, time.monotonic())):
            self.lidar_latched = False
        wanted = ((active or getattr(self, 'lidar_latched', False)) and not self.course.finished
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

    def lidar_front_clear(self, now):
        control = self.controller
        return (self.lidar_actual is True and not self.lidar_pending
                and now-self.lidar_state_received <= self.cfg['input_timeout']
                and not self.estop and now-self.last_estop <= self.cfg['input_timeout']
                and control.fresh('obstacles', now) and control.fresh('e_stop', now)
                and control.obstacles.get('valid') is True
                and not any(0 < x < 0.8 and abs(y) < 0.3
                            for x, y in control.obstacles.get('points', []))
                and control.curve_clear(0.0, 0.8))

    def dispatch(self, command):
        if command is None:
            return
        try:
            mode = command.get('mode')
            if mode:
                # Course activation owns the mode; divider detection never arms it.
                self.controller.cfg['auto_obstacle_mode'] = False
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
            'obstacle_stage_activated': '4번 활성화 / 장애물 회피 시작',
            'bump_passed_by_odometry': '서행 표지판 사라짐 + 설정 거리/시간 충족',
            'guide_distance_completed': '서행 유도선 설정 거리 통과',
            'pedestrian_stop_and_wait_completed': '보행자 정지 후 대기 완료/발진',
            'direction_sign_and_clear_center_exit': '방향 표지판 + 중앙 정렬/전방 LiDAR 비움 확인',
            'branch_turn_and_lane_reacquired': '갈림길 회전 후 차선 재검출',
            'cone_exit_and_lane_confirmed': '최소 거리 + 라바콘 구간 이탈/차선 확인',
            'green_pixel_threshold_departure': '초록불 면적 기준 충족/차선 추종 복귀',
            't_parking_completed': '주차 경로/영역/자세/정지 조건 충족',
        }
        for event in getattr(self.course, 'events', []):
            if event['sequence'] <= self.event_sequence:
                continue
            payload = dict(event, timestamp=time.strftime('%Y-%m-%d %H:%M:%S'))
            text = json.dumps(payload, ensure_ascii=False)
            self.mission_events.publish(String(data=text))
            message = '[미션 %d/8 %s] %s | %s | %.2fm / %.1fs' % (
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
        dt = min(0.2, max(0.0, now-self.previous_tick))
        self.previous_tick = now
        control = self.controller
        lidar_enabled = self.update_lidar_request()
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
        sensors = ('lane', 'odom', 'vehicle') + (('obstacles', 'e_stop') if lidar_enabled else ())
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
        if healthy and not self.course.finished:
            obs = Observation(now, control.travel, self.tracker.signs(now), control.lane,
                              tuple(control.obstacles.get('points', [])) if lidar_enabled else (), control.mode,
                              control.phase, control.odom_speed, control.reason,
                              Core.lidar_front_clear(self, now)
                              if lidar_enabled and self.course.active.name == 'obstacle' else False)
            epoch = self.course.epoch
            result = self.course.tick(obs)
            self.reason = result.reason
            self.mission_elapsed += dt
            if result.fault:
                self.fail(result.fault)
            elif self.mission_elapsed > self.cfg['active_timeout']:
                self.fail('mission_timeout: '+self.course.active.name)
            else:
                self.dispatch(result.command)
            if self.course.epoch != epoch:
                self.mission_elapsed = 0.0
                self.tracker.reset()
                self.get_logger().info(f'Mission {self.course.index+1}: {self.course.active.name}')
            # Switch motor demand before any command can escape into the next mission.
            after = self.update_lidar_request()
            if after != lidar_enabled or self.course.finished:
                healthy = False
        Core.report_mission_events(self)
        output = Twist()
        if (healthy and not self.fault and not self.course.finished and control.mode != 'hold'
                and now-self.last_command <= self.cfg['input_timeout']
                and all(math.isfinite(v) for v in (self.command.linear.x, self.command.angular.z))):
            output = self.command
        self.publisher.publish(output)
        progress = self.course.progress(Observation(now, control.travel)) if hasattr(self.course, 'progress') else {}
        self.status.publish(String(data=json.dumps(dict(
            stage=self.course.index+1, mission=self.course.active.name,
            phase=self.course.active.phase, finished=self.course.finished,
            ready=healthy, fault=self.fault, reason=self.reason,
            controller_mode=control.mode, controller_phase=control.phase,
            controller_reason=control.reason, blockers=blockers,
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
    rclpy.init(args=ros_args(args))
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        controller = LimoControl(
            cli_args=['--ros-args', '-r', 'cmd_vel:=/core/control_cmd'],
            parameter_overrides=[Parameter('auto_slow_sign', value=False),
                                 Parameter('auto_obstacle_mode', value=False)])
        nodes.append(controller)
        nodes.append(Core(controller))
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
