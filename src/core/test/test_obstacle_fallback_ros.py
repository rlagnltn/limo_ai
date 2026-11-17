"""Short-path recovery uses the real swept-body checks, without ROS nodes."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[2] / 'line_slow'))
from lane_change import LaneChangePlanner
from limo_control import LimoControl


class ObstacleFallbackTests(unittest.TestCase):
    def control(self):
        h = NS(pose=(0., 0., 0.), last_curvature=0., lidar_enabled=True,
               max_curvature=math.tan(.4)/.2,
               cfg=dict(robot_width=.22, robot_front=.18, robot_rear=.18,
                        rear_axle_offset=.1, safety_margin=0., curvature_rate=2.,
                        lane_change_distance=1.2),
               lane=dict(ground_lines=[[0., .825], [0., -.825]]),
               obstacles=dict(points=[], coverage=[1.]*36, range_limit=3.,
                              sensor_origin=[0., 0.]))
        h.footprint_clear = lambda *a, **kw: LimoControl.footprint_clear(h, *a, **kw)
        h.curve_clear = lambda *a, **kw: LimoControl.curve_clear(h, *a, **kw)
        return h

    def test_no_full_path_uses_observed_short_opening(self):
        h = self.control()
        # A distant row prevents every full 2.6m plan; there is room to advance.
        h.obstacles['points'] = [(1.4, i*.04) for i in range(-21, 22)]
        planner = LaneChangePlanner()
        self.assertFalse(planner.plan(h, h.lane['ground_lines']))
        curvature, reason = planner.command(h)
        self.assertIsNotNone(curvature)
        self.assertEqual(reason, 'cautious_open_direction')

    def test_choose_right_when_left_and_center_are_obstructed(self):
        h = self.control()
        h.obstacles['points'] = [(.7+i*.04, j*.04)
                                  for i in range(5) for j in range(-1, 16)]
        planner = LaneChangePlanner()
        with patch.object(planner, 'plan', return_value=[]):
            curvature, reason = planner.command(h)
        self.assertEqual(reason, 'cautious_open_direction')
        self.assertLess(curvature, 0.)
        applied = max(-.1, min(.1, curvature))
        self.assertTrue(h.curve_clear(applied, .24, road_bounds=h.lane['ground_lines']))

    def test_blocked_or_unobserved_short_path_stops(self):
        for problem in ('wall', 'unknown', 'road_too_narrow'):
            h = self.control()
            if problem == 'wall':
                h.obstacles['points'] = [(.3, i*.02) for i in range(-50, 51)]
            elif problem == 'unknown':
                h.obstacles['coverage'] = [0.]*36
            else:
                h.lane['ground_lines'] = [[0., .1], [0., -.1]]
            planner = LaneChangePlanner()
            with patch.object(planner, 'plan', return_value=[]):
                curvature, reason = planner.command(h)
            self.assertIsNone(curvature, problem)
            self.assertEqual(reason, 'no_clear_short_forward_path')

    def test_full_plan_resumes_when_open(self):
        h = self.control()
        _, reason = LaneChangePlanner().command(h)
        self.assertEqual(reason, 'tracking_odometry_lane_path')


if __name__ == '__main__':
    unittest.main()
