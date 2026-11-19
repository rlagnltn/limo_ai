"""Run the actual core tick without ROS nodes or a connected robot."""
import importlib.util
from pathlib import Path
import sys
import time
import tempfile
import json
from types import SimpleNamespace
import unittest

root = Path(__file__).parents[1]
sys.path.insert(0, str(root))
import core
if 'core.behaviors' not in sys.modules:
    spec = importlib.util.spec_from_file_location(
        'core.behaviors', root.parent / 'line_slow/__init__.py',
        submodule_search_locations=[str(root.parent / 'line_slow')])
    module = importlib.util.module_from_spec(spec)
    sys.modules['core.behaviors'] = module
    spec.loader.exec_module(module)
from core.core import Core
from core.missions.base import Result
from core.missions.base import Observation
from core.course import Course, DEFAULTS
from geometry_msgs.msg import Twist


class CoreLidarGateTests(unittest.TestCase):
    def test_mission_events_are_published_and_saved_exactly_once(self):
        cfg = dict(DEFAULTS, parking_speed=.05, rear_axle_offset=.1,
                   wheelbase=.2, max_steering_angle=.35)
        course = Course(cfg)
        course.index = 1
        course.tick(Observation(0))
        course.tick(Observation(1, stop_line_seen_at=.99))
        course.tick(Observation(2, travel=1.81))
        messages, logs = [], []
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/'events.jsonl')
            h = SimpleNamespace(course=course, event_sequence=0, event_log_path=path,
                                mission_events=SimpleNamespace(publish=messages.append),
                                get_logger=lambda: SimpleNamespace(info=logs.append,
                                                                  error=logs.append,
                                                                  warning=logs.append))
            Core.report_mission_events(h)
            Core.report_mission_events(h)
            saved = [json.loads(line) for line in Path(path).read_text().splitlines()]
        self.assertEqual(len(saved), 3)
        self.assertEqual(len(messages), 3)
        self.assertEqual(saved[-1]['state'], 'completed')
        self.assertEqual(saved[-1]['reason'], 'guide_distance_completed')
        self.assertIn('완료 판정', logs[-1])

    def harness(self, mission, motor_state):
        now = time.monotonic()
        output, requests, status = [], [], []
        control = SimpleNamespace(
            cfg=dict(command_control_mode=1, require_ackermann=False),
            lidar_enabled=mission in ('obstacle', 'cone'), received={},
            fresh=lambda key, now: key not in ('obstacles','e_stop'),
            vehicle_ok=True, odom_fault=False, vehicle_control_mode=1,
            mode='follow', phase='running', reason='lane_follow',
            hold_reason='', travel=0., odom_speed=0., lane=dict(valid=True),
            obstacles={})
        course = SimpleNamespace(active=SimpleNamespace(name=mission, phase='waiting'),
                                 finished=False, epoch=0, index=0,
                                 tick=lambda obs: Result(reason='waiting'))
        h = SimpleNamespace(controller=control, course=course, cfg=dict(
            input_timeout=.5, detection_timeout=1., sign_timeout=.5),
            get_parameter=lambda name: SimpleNamespace(value=True),
            fault='', estop=True, stop_line_seen_at=None,
            last_estop=-float('inf'), last_boxes=now,
            lidar_actual=motor_state, lidar_pending=False,
            lidar_reason='motor_stopped' if motor_state is False else 'scanning',
            lidar_state_received=now, last_blockers=None, last_blocker_log=now,
            last_command=now, command=Twist(),
            tracker=SimpleNamespace(signs=lambda now: frozenset(), large_cone_count=0), reason='',
            dispatch=lambda command: None,
            publisher=SimpleNamespace(publish=output.append),
            status=SimpleNamespace(publish=status.append),
            lidar_request=SimpleNamespace(publish=requests.append))
        h.command.linear.x = .1
        h.update_lidar_request = lambda: Core.update_lidar_request(h)
        return h, output, requests

    def test_non_lidar_missions_drive_without_scan_or_estop_heartbeat(self):
        for mission in ('slow','guide','pedestrian','intersection','traffic','parking'):
            h, output, requests = self.harness(mission, False)
            Core.tick(h)
            self.assertGreater(output[-1].linear.x, 0, mission)
            self.assertFalse(requests[-1].data, mission)

    def test_lidar_missions_wait_for_motor_and_new_scan(self):
        for mission in ('obstacle', 'cone'):
            h, output, requests = self.harness(mission, False)
            Core.tick(h)
            self.assertEqual(output[-1].linear.x, 0)
            self.assertTrue(requests[-1].data)
            # Motor response alone is insufficient; scan/stop data must also be fresh.
            h.lidar_actual = True
            Core.tick(h)
            self.assertEqual(output[-1].linear.x, 0)
            h.estop = False
            h.last_estop = time.monotonic()
            h.controller.fresh = lambda key, now: True
            Core.tick(h)
            self.assertGreater(output[-1].linear.x, 0)

    def test_obstacle_waits_for_stop_line_with_lidar_off_and_lane_following(self):
        h, output, requests = self.harness('obstacle', False)
        h.course.active.phase = 'waiting_for_stop_line'
        h.controller.lidar_enabled = False
        Core.tick(h)
        self.assertGreater(output[-1].linear.x, 0)
        self.assertFalse(requests[-1].data)
        self.assertFalse(h.fault)

        def enter_obstacle(obs):
            h.course.active.phase = 'avoiding'
            return Result({'mode': 'obstacles'})

        h.course.tick = enter_obstacle
        Core.tick(h)
        self.assertEqual(output[-1].linear.x, 0)
        self.assertTrue(requests[-1].data)

    def test_obstacle_avoiding_continues_while_waiting_for_direction_sign(self):
        h, output, _ = self.harness('obstacle', True)
        h.course.active.phase = 'avoiding'
        h.course.tick = lambda obs: Result(reason='avoiding')
        h.controller.mode = 'obstacles'
        h.controller.fresh = lambda key, now: True
        h.estop = False
        h.last_estop = time.monotonic()
        Core.tick(h)
        self.assertGreater(output[-1].linear.x, 0)
        self.assertFalse(h.fault)

    def test_non_lidar_mission_waits_for_motor_to_finish_stopping(self):
        h, output, _ = self.harness('slow', True)
        Core.tick(h)
        self.assertEqual(output[-1].linear.x, 0)
        h.lidar_actual = False
        Core.tick(h)
        self.assertGreater(output[-1].linear.x, 0)

    def test_cone_does_not_publish_previous_command_at_exit_or_after_scan_loss(self):
        h, output, _ = self.harness('cone', True)
        h.estop = False
        h.last_estop = time.monotonic()
        h.controller.fresh = lambda key, now: True
        h.controller.mode = 'cones'
        h.controller.phase = 'cone_tracking'
        h.controller.cone_scan_time = time.monotonic()
        Core.tick(h)
        self.assertGreater(output[-1].linear.x, 0.)
        h.controller.cone_scan_time -= .31
        Core.tick(h)
        self.assertEqual(output[-1].linear.x, 0.)
        h.controller.cone_scan_time = time.monotonic()
        h.controller.phase = 'cone_acquire'
        Core.tick(h)
        self.assertEqual(output[-1].linear.x, 0.)

    def test_forward_parking_uses_vendor_command_and_stops_on_failure(self):
        h, output, requests = self.harness('parking', True)
        h.course.active.phase = 'parking'
        h.controller.mode = 'hold'
        h.controller.vehicle_motion_mode = 1
        h.controller.lidar_enabled = True
        h.controller.fresh = lambda key, now: key != 'lane'
        h.estop = False
        h.last_estop = time.monotonic()
        h.last_parking_command = time.monotonic()
        h.parking_command = Twist()
        h.parking_command.linear.x = .12
        h.parking_command.angular.z = -.264
        logic = SimpleNamespace(state='TURN', reason='')
        h.parking_node = SimpleNamespace(logic=logic)
        Core.tick(h)
        self.assertTrue(requests[-1].data)
        self.assertAlmostEqual(output[-1].linear.x, .12)
        self.assertAlmostEqual(output[-1].angular.z, -.264)
        h.estop = True
        Core.tick(h)
        self.assertEqual(output[-1].linear.x, 0.)
        h.estop = False
        logic.state = 'FAILED'
        Core.tick(h)
        self.assertEqual(output[-1].linear.x, 0.)


if __name__ == '__main__':
    unittest.main()
