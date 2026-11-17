import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
from core.course import Course, DEFAULTS
from core.detections import SignTracker, class_id
from core.missions.base import Observation
from core.parking_path import t_parking_path


def config(**overrides):
    cfg = dict(DEFAULTS, parking_speed=0.05, rear_axle_offset=0.1,
               wheelbase=0.2, max_steering_angle=0.35)
    cfg.update(overrides)
    return cfg


class CourseTests(unittest.TestCase):
    def test_early_parking_light_and_direction_do_not_skip_slow(self):
        course = Course(config())
        course.tick(Observation(0))
        for identifier in (0, 1, 2, 3, 4, 6, 7):
            result = course.tick(Observation(1, signs=frozenset({identifier})))
            self.assertIsNone(result.command)
            self.assertEqual(course.index, 0)
        result = course.tick(Observation(2, signs=frozenset({5})))
        self.assertEqual(result.command['mode'], 'slow')

    def test_full_course_replay(self):
        c = Course(config())
        valid = dict(valid=True, lane_count=1)
        def tick(t, distance=0, signs=(), mode='follow', phase='running', lane=None):
            return c.tick(Observation(t, distance, frozenset(signs), valid if lane is None else lane,
                                      (), mode, phase))
        tick(0)
        self.assertEqual(tick(1, signs=[5]).command['mode'], 'slow')
        tick(3, .7, mode='slow')
        self.assertTrue(tick(3.6, .7, mode='slow').done)
        self.assertEqual(c.active.name, 'guide')
        self.assertEqual(tick(4, .7).command['mode'], 'slow')
        self.assertTrue(tick(5, 1.6, mode='slow').done)
        tick(6, 1.6)
        self.assertEqual(tick(7, 1.6, signs=[4]).command['mode'], 'pedestrian')
        tick(8, 2, mode='pedestrian', phase='waiting')
        self.assertTrue(tick(11, 2).done)
        tick(12, 2)
        tick(13, 2, lane=dict(valid=True, lane_count=3))
        self.assertEqual(tick(13.5, 2, lane=dict(valid=True, lane_count=3)).command['mode'], 'obstacles')
        tick(15, 3.1, mode='obstacles')
        self.assertTrue(tick(16.1, 3.1, mode='obstacles').done)
        tick(17, 3.1)
        self.assertEqual(tick(18, 3.1, signs=[2]).command['mode'], 'branch_a')
        tick(19, 4)
        self.assertTrue(tick(19.6, 4).done)
        tick(20, 4)
        self.assertEqual(tick(21, 4, signs=[7]).command['mode'], 'cones')
        self.assertEqual(tick(23, 5.3, mode='cones').command['mode'], 'hold')
        self.assertTrue(tick(24.1, 5.3, mode='hold').done)
        tick(25, 5.3)
        self.assertEqual(tick(26, 5.3, signs=[0]).command['mode'], 'hold')
        self.assertEqual(tick(27, 5.8, signs=[0, 1], mode='hold').command['mode'], 'hold')
        self.assertTrue(tick(28, 5.8, signs=[1], mode='hold').done)
        tick(30, 5.8)
        parking = tick(31, 5.8, signs=[6]).command
        self.assertEqual(parking['mode'], 'parking')
        self.assertGreater(len(parking['path']), 2)
        self.assertTrue(tick(35, 7, mode='parking', phase='complete').done)
        self.assertTrue(c.finished)
        self.assertEqual(tick(36, 7).command['mode'], 'hold')
        self.assertEqual([row['state'] for row in c.summary()], ['completed']*8)
        self.assertEqual(len([e for e in c.events if e['state']=='completed']), 8)
        self.assertEqual(c.results[0]['reason'], 'bump_passed_by_odometry')
        self.assertEqual(c.results[7]['reason'], 't_parking_completed')

    def test_traffic_green_cannot_bypass_physical_stop(self):
        c = Course(config())
        c.index = 6
        c.tick(Observation(0))
        r = c.tick(Observation(1, signs=frozenset({1})))
        self.assertEqual(r.command['mode'], 'hold')
        r = c.tick(Observation(2, signs=frozenset({1}), mode='hold', speed=.1))
        self.assertEqual(r.command, {'mode': 'hold'})
        self.assertFalse(r.done)

    def test_direction_conflict_waits_then_chooses_right(self):
        c = Course(config())
        c.index = 4
        c.tick(Observation(0))
        self.assertEqual(c.tick(Observation(1, signs=frozenset({2, 3}))).command['mode'], 'hold')
        self.assertEqual(c.tick(Observation(2, signs=frozenset({3}), mode='hold')).command['mode'], 'branch_b')

    def test_isolated_mission_stops_instead_of_advancing(self):
        c = Course(config(end_stage=2))
        c.index = 1
        c.tick(Observation(0))
        c.tick(Observation(1, travel=1))
        self.assertTrue(c.finished)
        self.assertEqual(c.index, 1)

    def test_exit_timeout_latches_hold(self):
        c = Course(config(obstacle_max_distance=2.0))
        c.index = 3
        c.tick(Observation(0))
        c.tick(Observation(1, lane={'lane_count': 3}))
        c.tick(Observation(2, lane={'lane_count': 3}))
        self.assertTrue(c.tick(Observation(3, travel=3)).fault)
        self.assertEqual(c.tick(Observation(4)).command['mode'], 'hold')
        self.assertEqual(c.results[3]['state'], 'failed')
        self.assertEqual(len([e for e in c.events if e['state']=='failed']), 1)

    def test_pending_missions_are_not_reported_as_successful(self):
        c = Course(config())
        c.tick(Observation(0))
        self.assertEqual(c.results[0]['state'], 'waiting')
        self.assertEqual(c.results[1]['state'], 'pending')
        c.tick(Observation(1, signs=frozenset({5})))
        self.assertEqual(c.results[0]['state'], 'active')
        self.assertFalse(any(row['state']=='completed' for row in c.summary()))
        summary = c.summary()
        summary[0]['state'] = 'completed'
        self.assertEqual(c.results[0]['state'], 'active')


class DetectionTests(unittest.TestCase):
    def test_exact_pixel_area_thresholds_for_all_classes(self):
        areas = [2900., 2700., 1800., 1800., 1800., 1800., 1800., 1800.]
        for identifier, area in enumerate(areas):
            for delta in (-1, 0, 1):
                tracker = SignTracker(.7, areas, 3)
                box = SimpleNamespace(class_name=str(identifier), probability=.9,
                                      xmin=0, ymin=0, xmax=1, ymax=int(area)+delta)
                for frame in range(3):
                    tracker.update([box], frame*.1)
                self.assertEqual(identifier in tracker.signs(.2), delta >= 0)

    def test_training_ids_and_aliases(self):
        for expected, name in enumerate(('red light', 'green light', 'left_sign', 'right_sign',
                                         'pedestrian_sign', 'slow_sign', 'parking_sign', 'traffic_cone')):
            self.assertEqual(class_id(name), expected)
        self.assertIsNone(class_id('unknown'))

    def test_confirmation_reset_and_stale_messages(self):
        tracker = SignTracker(.7, [100.0]*8, 3, .5)
        box = SimpleNamespace(class_name='parking_sign', probability=.9, xmin=0, ymin=0, xmax=20, ymax=20)
        tracker.update([box], 0)
        tracker.update([box], .1)
        self.assertFalse(tracker.signs(.1))
        tracker.update([box], .2)
        self.assertEqual(tracker.signs(.2), frozenset({6}))
        self.assertFalse(tracker.signs(1))
        tracker.reset()
        tracker.update([box], 1.1)
        self.assertFalse(tracker.signs(1.1))

    def test_invalid_boxes_do_not_trigger(self):
        tracker = SignTracker(.7, [100.0]*8, 1)
        bad = SimpleNamespace(class_name='5', probability=float('nan'), xmin=0, ymin=0, xmax=20, ymax=20)
        tracker.update([bad], 0)
        self.assertFalse(tracker.signs(0))

    def test_red_inhibits_departure_without_waiting_for_three_frames(self):
        tracker = SignTracker(.7, [100.0]*8, 3)
        red = SimpleNamespace(class_name='red_light', probability=.9, xmin=0, ymin=0, xmax=20, ymax=20)
        tracker.update([red], 0)
        self.assertEqual(tracker.signs(0), frozenset({0}))


class ParkingTests(unittest.TestCase):
    def test_reversible_path_and_mirrored_sides(self):
        right = t_parking_path('right', .6, .6, .3, .08, .05, .1, .2, .35)
        left = t_parking_path('left', .6, .6, .3, .08, .05, .1, .2, .35)
        self.assertLessEqual(len(right), 100)
        self.assertGreater(right[0][3], 0)
        self.assertLess(right[-1][3], 0)
        self.assertAlmostEqual(right[-1][0], -.1)
        self.assertAlmostEqual(right[-1][1], -.8)
        self.assertAlmostEqual(right[-1][2], math.pi/2)
        for r, l in zip(right, left):
            self.assertAlmostEqual(r[0], l[0])
            self.assertAlmostEqual(r[1], -l[1])
            self.assertAlmostEqual(r[2], -l[2])

    def test_reject_impossible_geometry(self):
        with self.assertRaises(ValueError):
            t_parking_path('right', .6, .1, .3, .08, .05, .1, .2, .35)
        with self.assertRaises(ValueError):
            t_parking_path('right', .6, .6, .3, .001, .05, .1, .2, .35)


if __name__ == '__main__':
    unittest.main()
