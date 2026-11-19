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
        def tick(t, distance=0, signs=(), mode='follow', phase='running', count=0, stop_line=None):
            return c.tick(Observation(t, distance, frozenset(signs),
                                      dict(valid=True, lane_count=1), (), mode, phase,
                                      speed=.2 if mode == 'follow' else 0., large_cone_count=count,
                                      stop_line_seen_at=stop_line))
        tick(0)
        self.assertEqual(tick(1, signs=[5]).command['mode'], 'slow')
        self.assertFalse(tick(7.99, signs=[5], mode='slow').done)
        self.assertTrue(tick(8, signs=[5], mode='slow').done)
        self.assertEqual(tick(11.1).command['mode'], 'follow')
        self.assertEqual(tick(11.2, 1., stop_line=11.15).command['mode'], 'slow')
        self.assertFalse(tick(11.3, 2.59, mode='slow').done)
        self.assertTrue(tick(11.4, 2.61, mode='slow').done)
        tick(11.5, 2.61)
        self.assertEqual(tick(12, 2.61, signs=[4]).command,
                         {'mode': 'pedestrian', 'immediate_stop': True})
        tick(13, 2.61, mode='pedestrian', phase='waiting')
        self.assertFalse(tick(17, 2.61).done)
        self.assertTrue(tick(20, 2.61).done)
        self.assertEqual(tick(36, 4).reason, 'waiting_for_stop_line')
        self.assertEqual(tick(37, 4, stop_line=36.99).command['mode'], 'obstacles')
        self.assertFalse(tick(38, 50, mode='obstacles').done)
        self.assertTrue(tick(39, 50, signs=[2], mode='obstacles').done)
        self.assertEqual(tick(40, 50, mode='hold').command['mode'], 'branch_a')
        self.assertFalse(tick(41, 51, mode="branch_a", count=1).done)
        self.assertFalse(tick(61, 55, count=0).done)
        self.assertTrue(tick(62, 55, count=1).done)
        self.assertEqual(tick(63, 55).command['mode'], 'cones')
        self.assertFalse(tick(64, 55, mode='cones', phase='cone_left').done)
        for step in range(21):
            result = tick(65+step*.05, 55, mode='cones', phase='cone_acquire')
        self.assertFalse(result.done)
        self.assertEqual(result.command['mode'], 'follow')
        self.assertFalse(tick(66.5, 56.3).done)
        self.assertFalse(tick(66.7, 56.3, signs=[0]).done)
        result = tick(67, 56.3, signs=[5])
        self.assertTrue(result.done)
        self.assertEqual(result.command, {'mode': 'slow'})
        self.assertEqual(tick(67.1, 56.3, mode='slow').command, {'mode': 'slow'})
        result = tick(67.2, 56.3, signs=[0, 6], mode='slow')
        self.assertTrue(result.done)
        self.assertIsNone(result.command)
        self.assertEqual(result.handoff_signs, frozenset({0}))
        self.assertEqual(tick(67.3, 56.3, mode='slow').command['mode'], 'hold')
        self.assertFalse(tick(68, 56.3, signs=[0, 1, 6], mode='hold').done)
        result = tick(69, 56.3, signs=[1, 6], mode='hold')
        self.assertTrue(result.done)
        self.assertFalse(result.handoff_signs)
        self.assertEqual(tick(70, 56.3).command, {'mode': 'follow'})
        parking = tick(71, 56.5, signs=[6])
        self.assertEqual(parking.command, {'mode': 'hold'})
        self.assertEqual(parking.reason, 'parking_sign_6')
        self.assertTrue(c.tick(Observation(72, 57, parking_state='DONE')).done)
        self.assertTrue(c.finished)
        self.assertEqual([row['state'] for row in c.summary()], ['completed']*9)
        self.assertEqual([e['stage'] for e in c.events if e['state']=='completed'],
                         [1, 2, 3, 4, 5, 6, 6.5, 7, 8])

    def test_cone_resumes_after_stopping_one_second_without_a_valid_lane(self):
        c = Course(config())
        c.index = 5
        self.assertEqual(c.tick(Observation(0)).command, {'mode': 'cones'})
        def observe(now, phase='cone_acquire', speed=0.):
            return c.tick(Observation(now, signs=frozenset({7}), lane={'valid': False},
                                      mode='cones', phase=phase, speed=speed))
        self.assertFalse(observe(.1, phase='cone_tracking').done)
        self.assertFalse(observe(.2, phase='cone_left').done)
        self.assertFalse(observe(.3, speed=.1).done)
        for step in range(20):
            self.assertIsNone(observe(.4+step*.05).command)
        result = observe(1.41)
        self.assertFalse(result.done)
        self.assertEqual(result.command, {'mode': 'follow'})
        self.assertEqual(c.active.name, 'cone')
        self.assertFalse(c.tick(Observation(2.1, signs=frozenset({1}))).done)
        result = c.tick(Observation(2.2, signs=frozenset({5})))
        self.assertTrue(result.done)
        self.assertEqual(result.command, {'mode': 'slow'})
        self.assertEqual(c.active.name, 'cone_slow')
        self.assertEqual(c.tick(Observation(2.3, mode='slow')).command, {'mode': 'slow'})
        self.assertFalse(c.tick(Observation(2.4, mode='slow')).done)
        result = c.tick(Observation(2.5, signs=frozenset({1}), mode='slow'))
        self.assertTrue(result.done)
        self.assertIsNone(result.command)
        self.assertEqual(result.handoff_signs, frozenset({1}))
        self.assertEqual(c.active.name, 'traffic')
        self.assertEqual(c.tick(Observation(2.6)).command, {'mode': 'hold'})

    def test_traffic_green_cannot_bypass_physical_stop(self):
        c = Course(config())
        c.index = 7
        c.tick(Observation(0))
        crosswalk = c.tick(Observation(0.5, lane={'crosswalk': True}))
        self.assertIsNone(crosswalk.command)
        self.assertFalse(crosswalk.done)
        r = c.tick(Observation(1, signs=frozenset({1})))
        self.assertEqual(r.command['mode'], 'hold')
        r = c.tick(Observation(2, signs=frozenset({1}), mode='hold', speed=.1))
        self.assertEqual(r.command, {'mode': 'hold'})
        self.assertFalse(r.done)

    def test_branch_ends_on_one_large_cone_as_soon_as_fixed_turn_ends(self):
        c = Course(config())
        c.index = 4
        c.tick(Observation(0))
        c.tick(Observation(1, signs=frozenset({2})))
        self.assertFalse(c.tick(Observation(2, mode='branch_a', large_cone_count=1)).done)
        self.assertFalse(c.tick(Observation(4, mode='follow', large_cone_count=0)).done)
        result = c.tick(Observation(4.05, mode='follow', lane={'valid': False},
                                    speed=0, large_cone_count=1))
        self.assertTrue(result.done)
        self.assertEqual(c.active.name, 'cone')
        self.assertEqual(c.tick(Observation(4.1)).command, {'mode': 'cones'})

    def test_pedestrian_completes_before_obstacle_waits_for_a_new_stop_line(self):
        c = Course(config())
        c.index = 2
        c.tick(Observation(0))
        c.tick(Observation(1, signs=frozenset({4})))
        self.assertFalse(c.tick(Observation(2, mode='pedestrian', phase='waiting')).done)
        self.assertFalse(c.tick(Observation(5, mode='follow')).done)
        self.assertTrue(c.tick(Observation(8, mode='follow')).done)
        self.assertEqual(c.results[2]['state'], 'completed')
        self.assertEqual(c.results[3]['state'], 'waiting')
        self.assertEqual(c.active.phase, 'waiting_for_stop_line')

        for now, seen_at in ((8.05, 7.9), (10, None), (25, 8)):
            result = c.tick(Observation(now, stop_line_seen_at=seen_at))
            self.assertIsNone(result.command)
            self.assertEqual(result.reason, 'waiting_for_stop_line')

        result = c.tick(Observation(25.05, lane={'valid': True, 'lane_count': 1},
                                    stop_line_seen_at=25.01))
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
        c.tick(Observation(.5, stop_line_seen_at=.49))
        c.tick(Observation(1, travel=1.61))
        self.assertTrue(c.finished)
        self.assertEqual(c.index, 1)

    def test_guide_ignores_signs_and_stale_lines_then_counts_distance_from_new_line(self):
        c = Course(config())
        c.index = 1
        self.assertEqual(c.tick(Observation(1)).command, {'mode': 'follow'})
        self.assertTrue(c.waiting_for_stop_line)
        self.assertIsNone(c.tick(Observation(2, travel=3, signs=frozenset({5}),
                                             stop_line_seen_at=.9)).command)
        result = c.tick(Observation(3, travel=4, stop_line_seen_at=2.99))
        self.assertEqual(result.command, {'mode': 'slow'})
        self.assertFalse(c.waiting_for_stop_line)
        self.assertFalse(c.tick(Observation(4, travel=5.59)).done)
        self.assertTrue(c.tick(Observation(5, travel=5.61)).done)

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
        tracker = SignTracker(.7, [1800.]*7 + [26000.], 3)
        def box(area, confidence=.9, name='traffic_cone'):
            return SimpleNamespace(class_name=name, probability=confidence,
                                   xmin=0, ymin=0, xmax=1, ymax=area)
        tracker.update([box(26000), box(27000), box(26000), box(25999),
                        box(27000, .6), box(27000, name='parking_sign')], 0)
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
