"""Offline tests: python3 -m unittest discover -s . -p 'test_lane_limo_adapter.py'."""
import math
import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch

import numpy as np

from lane_obstacle_avoidance import LaneObstacleAvoidance
from lane_obstacle_avoidance_ros import scan_bins, sector_clear, twist_values, HSVTuner, save_hsv


class AdapterTests(unittest.TestCase):
    def test_scan_angle_layout_and_invalid_values(self):
        scan = S(angle_min=-math.pi, angle_increment=math.pi / 180,
                 ranges=[2.0] * 360, range_min=.01, range_max=10.)
        scan.ranges[180] = .4  # forward is sample 180, not sample zero
        bins = scan_bins(scan)
        self.assertAlmostEqual(bins[0], .4)
        self.assertAlmostEqual(bins[180], 2.)
        scan.ranges[180] = float('inf')
        self.assertTrue(math.isnan(scan_bins(scan)[0]))

    def test_reversed_scan_and_laser_rotation(self):
        scan = S(angle_min=math.pi, angle_increment=-math.pi / 180,
                 ranges=[float('nan')] * 360, range_min=.01, range_max=10.)
        scan.ranges[90] = 1.0  # laser-left rotates into base-front
        self.assertAlmostEqual(scan_bins(scan, -math.pi / 2)[0], 1.0)
        self.assertAlmostEqual(scan_bins(scan, -math.pi / 2, (.1, 0.))[0], 1.1)

    def test_partial_scan_is_not_fabricated_as_clear(self):
        scan = S(angle_min=0., angle_increment=.01, ranges=[1.], range_min=.01, range_max=10.)
        bins = scan_bins(scan)
        self.assertFalse(sector_clear(bins, range(-7, 8), .2))
        self.assertTrue(math.isnan(bins[180]))

    def test_steering_sign_and_limit(self):
        speed, yaw = twist_values(40, .05, .2, .35, .007)
        self.assertEqual(speed, .05)
        self.assertLess(yaw, 0)  # original right -> negative ROS yaw
        self.assertGreater(twist_values(-40, .05, .2, .35, .007)[1], 0)
        self.assertAlmostEqual(twist_values(10000, .05, .2, .35, .007)[1],
                               -.05 * math.tan(.35) / .2)

    def test_original_lane_and_change_state_machine(self):
        core = LaneObstacleAvoidance(obstacle_distance=.5)
        image = np.zeros((480, 640, 3), np.uint8)
        scan = [2.] * 360
        lanes = {'left': [160] * 8, 'right': [480] * 8}
        with patch.object(core, 'detect_lane_indices', return_value=lanes):
            self.assertEqual(core.update(image, scan).steering, 0.)
            scan[0] = .4
            result = core.update(image, scan)
            self.assertEqual((result.target_lane, result.steering), ('right', 40.))
            scan[0] = 2.
            result = core.update(image, scan)
            self.assertEqual((result.lane, result.target_lane), ('right', None))

    def test_lane_detector_runs(self):
        core = LaneObstacleAvoidance()
        self.assertEqual(core.detect_lane_indices(np.zeros((480, 640, 3), np.uint8)),
                         {'left': [], 'right': []})

    def test_yellow_selected_white_rejected(self):
        core = LaneObstacleAvoidance()
        yellow = np.full((480, 640, 3), (0, 255, 255), dtype=np.uint8)
        white = np.full((480, 640, 3), 255, dtype=np.uint8)
        self.assertTrue(core.detect_lane_indices(yellow)['left'])
        self.assertTrue(core.detect_lane_indices(yellow)['right'])
        self.assertEqual(core.detect_lane_indices(white), {'left': [], 'right': []})
        white_core = LaneObstacleAvoidance(lane_hsv_low=(0, 0, 220),
                                          lane_hsv_high=(179, 25, 255))
        self.assertTrue(white_core.detect_lane_indices(white)['left'])

    def test_invalid_hsv_rejected(self):
        with self.assertRaises(ValueError):
            LaneObstacleAvoidance(lane_hsv_low=(50, 70, 70))

    def test_slider_values_apply_to_actual_detector(self):
        core = LaneObstacleAvoidance()
        tuner = HSVTuner.__new__(HSVTuner)
        tuner.algorithm, tuner.closed = core, False
        # Crossed bounds are corrected; actual algorithm receives new values.
        positions = {'H low': 20, 'H high': 10, 'S low': 100,
                     'S high': 255, 'V low': 80, 'V high': 250}
        with patch('cv2.getWindowProperty', return_value=1), \
             patch('cv2.getTrackbarPos', side_effect=lambda name, _: positions[name]), \
             patch('cv2.setTrackbarPos') as set_pos, \
             patch('cv2.imshow'), patch('cv2.waitKey', return_value=ord('s')):
            key = tuner.update(np.zeros((480, 640, 3), np.uint8))
        self.assertEqual(key, ord('s'))
        self.assertEqual(core.lane_hsv_low, (20, 100, 80))
        self.assertEqual(core.lane_hsv_high, (20, 255, 250))
        set_pos.assert_called_once_with('H high', tuner.WINDOW, 20)

    def test_save_ros_parameter_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = save_hsv(Path(directory) / 'params.yaml', (15, 70, 70), (40, 255, 255))
            content = path.read_text()
            self.assertIn('lane_obstacle_avoidance:\n  ros__parameters:', content)
            self.assertIn('lane_hsv_low: [15, 70, 70]', content)
            self.assertIn('lane_hsv_high: [40, 255, 255]', content)


if __name__ == '__main__':
    unittest.main()
