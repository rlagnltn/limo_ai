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
        course.tick(Observation(1, signs=frozenset({5})))
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
            lidar_enabled=mission == 'obstacle', received={},
            fresh=lambda key, now: key not in ('obstacles','e_stop'),
            vehicle_ok=True, odom_fault=False, vehicle_control_mode=1,
            mode='follow', phase='running', reason='lane_follow',
            hold_reason='', travel=0., odom_speed=0., lane=dict(valid=True),
            obstacles={})
        course = SimpleNamespace(active=SimpleNamespace(name=mission, phase='waiting'),
                                 finished=False, epoch=0, index=0,
                                 tick=lambda obs: Result(reason='waiting'))
        h = SimpleNamespace(controller=control, course=course, cfg=dict(
            input_timeout=.5, detection_timeout=1., active_timeout=120.),
            get_parameter=lambda name: SimpleNamespace(value=True),
            fault='', previous_tick=now, estop=True,
            last_estop=-float('inf'), last_boxes=now,
            lidar_actual=motor_state, lidar_pending=False,
            lidar_reason='motor_stopped' if motor_state is False else 'scanning',
            lidar_state_received=now, last_blockers=None, last_blocker_log=now,
            last_command=now, command=Twist(), mission_elapsed=0.,
            tracker=SimpleNamespace(signs=lambda now: frozenset()), reason='',
            dispatch=lambda command: None,
            publisher=SimpleNamespace(publish=output.append),
            status=SimpleNamespace(publish=status.append),
            lidar_request=SimpleNamespace(publish=requests.append))
        h.command.linear.x = .1
        h.update_lidar_request = lambda: Core.update_lidar_request(h)
        return h, output, requests

    def test_non_lidar_missions_drive_without_scan_or_estop_heartbeat(self):
        for mission in ('slow','guide','pedestrian','intersection','cone','traffic','parking'):
            h, output, requests = self.harness(mission, False)
            Core.tick(h)
            self.assertGreater(output[-1].linear.x, 0, mission)
            self.assertFalse(requests[-1].data, mission)

    def test_lidar_missions_wait_for_motor_and_new_scan(self):
        for mission in ('obstacle',):
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

    def test_non_lidar_mission_waits_for_motor_to_finish_stopping(self):
        h, output, _ = self.harness('slow', True)
        Core.tick(h)
        self.assertEqual(output[-1].linear.x, 0)
        h.lidar_actual = False
        Core.tick(h)
        self.assertGreater(output[-1].linear.x, 0)

    def test_cone_clock_receives_final_core_output_including_blocked_stop(self):
        h, output, _ = self.harness('cone', False)
        recorded = []
        h.controller.record_cone_output = lambda now, command: recorded.append(command.linear.x)
        Core.tick(h)
        self.assertGreater(recorded[-1], 0.)
        h.last_boxes = -float('inf')
        Core.tick(h)
        self.assertEqual(recorded[-1], 0.)
        self.assertEqual(recorded[-1], output[-1].linear.x)


if __name__ == '__main__':
    unittest.main()
