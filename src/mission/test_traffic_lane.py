"""ROS 소켓 없이 신호등 상태와 실제 Twist 출력, 차선 검출을 검사한다."""
import importlib.util
import json
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from std_msgs.msg import Bool, String

import traffic_light_trigger as traffic


class TrafficLaneTest(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(traffic.time, 'monotonic', return_value=100.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = traffic.TrafficLightTrigger.__new__(traffic.TrafficLightTrigger)
        n = self.node
        n.config = dict(min_confidence=.7, red_min_area=3000., green_min_area=2700.,
                        red_stop_delay_sec=0., green_drive_sec=3., forward_speed=.2,
                        rearm_absence_sec=1., detection_timeout_sec=1.,
                        lane_timeout_sec=.5, lane_p_gain=.01)
        n.state = 'startup'
        n.last_message = None
        n.emergency = False
        n.armed = dict(red_light=True, green_light=True)
        n.absent_since = dict(red_light=None, green_light=None)
        n.lane_received = n.lane_status_received = None
        n.lane_detected = False
        n.lane_gap = 0
        self.commands = []
        n.publisher = SimpleNamespace(publish=self.commands.append)
        n.get_logger = lambda: SimpleNamespace(info=lambda _: None)

    def boxes(self, *names):
        self.node.on_boxes(SimpleNamespace(bounding_boxes=[
            SimpleNamespace(class_name=name, probability=.9,
                            xmin=0, ymin=0, xmax=60, ymax=60) for name in names]))

    def lane(self, gap=20, detected=True):
        self.node.on_lane_state(String(data=json.dumps(dict(offset=gap, valid=detected))))

    def assert_velocity(self, linear, angular):
        self.assertAlmostEqual(self.commands[-1].linear.x, linear)
        self.assertAlmostEqual(self.commands[-1].angular.z, angular)

    def test_red_green_and_three_second_stop(self):
        n = self.node
        self.lane()
        n.on_timer()
        self.assert_velocity(0., 0.)
        self.boxes()
        n.on_timer()
        self.assert_velocity(.2, .2)
        self.boxes('red_light', 'green_light')
        self.assertEqual(n.state, 'stopped')
        self.assert_velocity(0., 0.)
        self.boxes('green_light')
        self.assertEqual(n.state, 'green_drive')
        self.assert_velocity(.2, .2)
        self.now.return_value = 103.1
        self.lane(-10)
        self.boxes()
        n.on_timer()
        self.assertEqual(n.state, 'stopped')
        self.assert_velocity(0., 0.)

    def test_missing_lost_stale_and_recovered_lane(self):
        self.boxes()
        self.node.on_timer()
        self.assert_velocity(0., 0.)
        self.lane(-10)
        self.node.on_timer()
        self.assert_velocity(.2, -.1)
        self.lane(detected=False)
        self.assert_velocity(0., 0.)
        self.lane()
        self.now.return_value = 100.6
        self.node.on_timer()
        self.assert_velocity(0., 0.)
        self.lane(0)
        self.node.on_timer()
        self.assert_velocity(.2, 0.)

    def test_red_delay_keeps_steering_and_emergency_stops(self):
        self.node.config['red_stop_delay_sec'] = 2.
        self.lane()
        self.boxes('red_light')
        self.assertEqual(self.node.state, 'red_wait')
        self.assert_velocity(.2, .2)
        self.node.on_emergency(Bool(data=True))
        self.assert_velocity(0., 0.)
        self.boxes('green_light')
        self.node.on_timer()
        self.assert_velocity(0., 0.)

    def test_detection_timeout_stops_both_velocities(self):
        self.lane()
        self.boxes()
        self.node.on_timer()
        self.now.return_value = 101.1
        self.lane()
        self.node.on_timer()
        self.assertEqual(self.node.state, 'stopped')
        self.assert_velocity(0., 0.)

    def test_malformed_lane_state_stops(self):
        self.lane()
        self.boxes()
        self.node.on_timer()
        for data in ('{}', 'bad json', '[]', '{"valid":true,"offset":NaN}'):
            self.node.on_lane_state(String(data=data))
            self.assert_velocity(0., 0.)

    def test_base_detector_center_and_signal_integration(self):
        path = Path(__file__).resolve().parents[1] / 'line_slow/detect_line.py'
        spec = importlib.util.spec_from_file_location('lane_source', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        parameters, published = {}, {}
        def publisher(_node, _type, topic, _depth):
            published[topic] = []
            return SimpleNamespace(publish=published[topic].append)
        def declare(_node, name, value):
            parameters[name] = value
        with ExitStack() as stack:
            for name, replacement in {
                '__init__': lambda *_a, **_k: None,
                'create_publisher': publisher,
                'create_subscription': lambda *_a, **_k: None,
                'create_timer': lambda *_a, **_k: None,
                'declare_parameter': declare,
                'get_parameter': lambda _n, key: SimpleNamespace(value=parameters[key]),
                'get_logger': lambda _n: SimpleNamespace(warning=lambda _: None),
            }.items():
                stack.enter_context(patch.object(module.Node, name, replacement))
            n = module.DetectLine()
            self.assertEqual(parameters['reference_distance'], 0)
            self.assertEqual(parameters['roi_x_h'], 0)
            self.assertTrue(parameters['show_visualization'])
            self.boxes()
            for shift in (0, -40, 40):
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                for x in (80+shift, 560+shift):
                    frame[280:480, x-5:x+6] = (0, 255, 255)
                n.image_callback(n.br.cv2_to_imgmsg(frame, 'bgr8'))
                msg = published['lane_state'][-1]
                state = json.loads(msg.data)
                self.assertTrue(state['valid'])
                self.assertAlmostEqual(state['offset'], -shift, delta=2)
                self.node.on_lane_state(msg)
                self.node.on_timer()
                self.assert_velocity(.2, state['offset']*.01)
            frame[:] = 0
            n.image_callback(n.br.cv2_to_imgmsg(frame, 'bgr8'))
            msg = published['lane_state'][-1]
            self.assertFalse(json.loads(msg.data)['valid'])
            self.node.on_lane_state(msg)
            self.assert_velocity(0., 0.)


if __name__ == '__main__':
    unittest.main()
