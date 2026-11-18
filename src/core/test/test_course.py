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
        def tick(t, distance=0, signs=(), mode='follow', phase='running', count=0):
            return c.tick(Observation(t, distance, frozenset(signs),
                                      dict(valid=True, lane_count=1), (), mode, phase,
                                      speed=.2 if mode == 'follow' else 0., large_cone_count=count))
        tick(0)
        self.assertEqual(tick(1, signs=[5]).command['mode'], 'slow')
        tick(3, .7, mode='slow')
        self.assertFalse(tick(5.99, .7, mode='slow').done)
        self.assertTrue(tick(6, .7, mode='slow').done)
        self.assertEqual(tick(7, .7).command['mode'], 'follow')
        self.assertEqual(tick(8, 1., signs=[5]).command['mode'], 'slow')
        self.assertFalse(tick(9, 2.79, mode='slow').done)
        self.assertTrue(tick(10, 2.81, mode='slow').done)
        tick(11, 2.81)
        self.assertEqual(tick(12, 2.81, signs=[4]).command,
                         {'mode': 'pedestrian', 'immediate_stop': True})
        tick(13, 2.81, mode='pedestrian', phase='waiting')
        self.assertFalse(tick(16, 2.81).done)
        self.assertFalse(tick(35.99, 4).done)
        self.assertTrue(tick(36, 4).done)
        self.assertEqual(tick(37, 4).command['mode'], 'obstacles')
        self.assertFalse(tick(38, 50, mode='obstacles').done)
        self.assertTrue(tick(39, 50, signs=[2], mode='obstacles').done)
        self.assertEqual(tick(40, 50, mode='hold').command['mode'], 'branch_a')
        self.assertFalse(tick(41, 51, count=5).done)
        self.assertFalse(tick(60.99, 55, count=5).done)
        self.assertFalse(tick(61, 55, count=4).done)
        self.assertTrue(tick(62, 55, count=5).done)
        self.assertEqual(tick(63, 55).command['mode'], 'cones')
        self.assertFalse(tick(64, 55, mode='cones', phase='cone_left').done)
        for step in range(21):
            result = tick(65+step*.05, 55, mode='cones', phase='cone_acquire')
        self.assertTrue(result.done)
        tick(66, 56.3)
        self.assertEqual(tick(67, 56.3, signs=[0]).command['mode'], 'hold')
        self.assertFalse(tick(68, 56.3, signs=[0, 1], mode='hold').done)
        self.assertTrue(tick(69, 56.3, signs=[1], mode='hold').done)
        tick(70, 56.3)
        self.assertEqual(tick(71, 56.3, signs=[6]).command['mode'], 'parking')
        self.assertTrue(tick(72, 57, mode='parking', phase='complete').done)
        self.assertTrue(c.finished)
        self.assertEqual([row['state'] for row in c.summary()], ['completed']*8)
        self.assertEqual(len([e for e in c.events if e['state']=='completed']), 8)

    def test_cone_requires_completed_manoeuvre_stop_and_continuous_lane_acquisition(self):
        c = Course(config())
        c.index = 5
        self.assertEqual(c.tick(Observation(0)).command, {'mode': 'cones'})
        def observe(now, phase='cone_acquire', valid=True, speed=0.):
            return c.tick(Observation(now, signs=frozenset({7}), lane={'valid': valid},
                                      mode='cones', phase=phase, speed=speed))
        self.assertFalse(observe(.1, phase='cone_tracking').done)
        self.assertFalse(observe(.2, phase='cone_left').done)
        self.assertFalse(observe(.3, speed=.1).done)
        for step in range(10):
            self.assertFalse(observe(.4+step*.05).done)
        self.assertFalse(observe(.9, valid=False).done)
        for step in range(21):
            result = observe(1.+step*.05)
        self.assertTrue(result.done)
        self.assertEqual(result.command, {'mode': 'follow'})
        self.assertEqual(c.active.name, 'traffic')

    def test_traffic_green_cannot_bypass_physical_stop(self):
        c = Course(config())
        c.index = 6
        c.tick(Observation(0))
        r = c.tick(Observation(1, signs=frozenset({1})))
        self.assertEqual(r.command['mode'], 'hold')
        r = c.tick(Observation(2, signs=frozenset({1}), mode='hold', speed=.1))
        self.assertEqual(r.command, {'mode': 'hold'})
        self.assertFalse(r.done)

    def test_branch_requires_twenty_seconds_moving_and_five_large_cones(self):
        c = Course(config())
        c.index = 4
        c.tick(Observation(0))
        c.tick(Observation(1, signs=frozenset({2})))
        def observe(now, speed=.2, count=5):
            return c.tick(Observation(now, lane={'valid': True}, speed=speed, large_cone_count=count))
        self.assertFalse(observe(2).done)
        self.assertFalse(observe(21.99).done)
        self.assertFalse(observe(22, count=4).done)
        self.assertFalse(observe(23, speed=0).done)
        self.assertFalse(observe(24).done)
        self.assertFalse(observe(43.99).done)
        self.assertTrue(observe(44).done)

    def test_pedestrian_follow_delay_then_obstacle_entry_without_three_lanes(self):
        c = Course(config())
        c.index = 2
        c.tick(Observation(0))
        c.tick(Observation(1, signs=frozenset({4})))
        c.tick(Observation(2, mode='pedestrian', phase='waiting'))
        self.assertFalse(c.tick(Observation(5, mode='follow')).done)
        self.assertFalse(c.tick(Observation(24.99, mode='follow')).done)
        self.assertEqual(c.active.name, 'pedestrian')
        self.assertTrue(c.tick(Observation(25, mode='follow')).done)
        self.assertEqual(c.active.name, 'obstacle')
        result = c.tick(Observation(25.05, lane={'valid': True, 'lane_count': 1}))
        self.assertEqual(result.command, {'mode': 'obstacles'})
        self.assertEqual(c.active.phase, 'avoiding')

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
        c.tick(Observation(.5, signs=frozenset({5})))
        c.tick(Observation(1, travel=1.81))
        self.assertTrue(c.finished)
        self.assertEqual(c.index, 1)

    def test_guide_waits_for_own_sign_and_counts_distance_from_trigger(self):
        c = Course(config())
        c.index = 1
        self.assertEqual(c.tick(Observation(0)).command, {'mode': 'follow'})
        self.assertFalse(c.tick(Observation(1, travel=2)).done)
        self.assertIsNone(c.tick(Observation(2, travel=3, signs=frozenset({4}))).command)
        result = c.tick(Observation(3, travel=4, signs=frozenset({5})))
        self.assertEqual(result.command, {'mode': 'slow'})
        self.assertFalse(c.tick(Observation(4, travel=4.7)).done)
        self.assertTrue(c.tick(Observation(5, travel=5.81)).done)

    def test_obstacle_exits_only_on_direction_and_hands_it_over(self):
        c = Course(config())
        c.index = 3
        c.tick(Observation(0))
        self.assertFalse(c.tick(Observation(3, travel=30, lane={'valid': True, 'lane_count': 1})).done)
        self.assertEqual(c.tick(Observation(4, signs=frozenset({2, 3}))).command, {'mode': 'hold'})
        self.assertTrue(c.tick(Observation(5, signs=frozenset({3}))).done)
        self.assertEqual(c.tick(Observation(6)).command, {'mode': 'branch_b'})

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
    def test_count_three_simultaneous_large_cones(self):
        tracker = SignTracker(.7, [1800.]*8, 3)
        def box(area, confidence=.9, name='traffic_cone'):
            return SimpleNamespace(class_name=name, probability=confidence,
                                   xmin=0, ymin=0, xmax=1, ymax=area)
        tracker.update([box(8000), box(9000), box(8000), box(7999),
                        box(9000, .6), box(9000, name='parking_sign')], 0)
        self.assertEqual(tracker.large_cone_count, 3)
        tracker.update([], .1)
        self.assertEqual(tracker.large_cone_count, 0)

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
