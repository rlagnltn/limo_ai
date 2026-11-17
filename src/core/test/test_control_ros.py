"""Exercise actual controller calculations without creating ROS nodes/sockets."""
from pathlib import Path
import math
import sys
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[2] / 'line_slow'))
sys.path.insert(0, str(Path(__file__).parents[1]))
from limo_control import LimoControl
from core.parking_path import t_parking_path
from std_msgs.msg import String


class ControllerTests(unittest.TestCase):
    def cone_harness(self):
        h = SimpleNamespace(mode='cones', phase='cone_right', cone_segment_elapsed=0.,
                            cone_output_time=None, cone_output_phase=None, cone_output_active=False,
                            vehicle_motion_mode=1, cfg=dict(cone_right_degrees=10., cone_left_degrees=15.,
                            cone_right_seconds=2., cone_left_seconds=2., cone_speed=.3, wheelbase=.2))
        return h

    def test_forced_cone_angles_and_two_timed_segments_then_stop(self):
        from geometry_msgs.msg import Twist
        h = self.cone_harness()
        now = 0.
        for phase, expected_degrees in (('cone_right', -10.), ('cone_left', 15.)):
            self.assertEqual(h.phase, phase)
            for step in range(42):
                speed, curvature, reason = LimoControl.forced_cone_control(h)
                if reason != phase:
                    break
                self.assertAlmostEqual(speed, .3)
                self.assertAlmostEqual(math.degrees(math.atan(curvature*.2)), expected_degrees)
                command = Twist()
                command.linear.x, command.angular.z = speed, speed*curvature
                LimoControl.record_cone_output(h, now, command)
                now += .05
            else:
                self.fail('Cone segment did not finish after two seconds of issued output')
            if phase == 'cone_right':
                self.assertEqual(h.phase, 'cone_left')
        self.assertEqual(h.phase, 'cone_acquire')
        self.assertEqual(LimoControl.forced_cone_control(h)[:2], (0., 0.))

    def test_forced_cone_does_not_count_core_blocks_or_wrong_steering(self):
        from geometry_msgs.msg import Twist
        h = self.cone_harness()
        zero = Twist()
        for step in range(100):
            LimoControl.record_cone_output(h, step*.05, zero)
        self.assertEqual(h.cone_segment_elapsed, 0.)
        wrong = Twist()
        wrong.linear.x = .3
        LimoControl.record_cone_output(h, 5., wrong)
        LimoControl.record_cone_output(h, 5.05, wrong)
        self.assertEqual(h.cone_segment_elapsed, 0.)
        self.assertEqual(LimoControl.forced_cone_control(h)[2], 'cone_right')
        h.vehicle_motion_mode = 0
        self.assertEqual(LimoControl.forced_cone_control(h)[0], 0.)

    def test_forced_cone_drives_without_scan_or_lane_detection_but_requires_fresh_camera(self):
        h = self.cone_harness()
        h.cfg.update(command_control_mode=1, command_mode_settle=.2, lap_timeout=600.)
        h.vehicle_control_mode, h.command_ready_since = 1, 0.
        h.lidar_enabled, h.e_stop_flag = False, True
        h.vehicle_ok, h.odom_fault, h.started_at = True, False, None
        h.travel, h.phase_travel, h.mission_distance = 0., 0., None
        h.lane = {'valid': False}
        h.fresh = lambda key, now: key not in ('obstacles', 'e_stop')
        h.get_parameter = lambda name: SimpleNamespace(value=True)
        h.forced_cone_control = lambda: LimoControl.forced_cone_control(h)
        self.assertEqual(LimoControl.compute_control(h, 10., .05)[0], .3)
        h.fresh = lambda key, now: key != 'lane'
        self.assertEqual(LimoControl.compute_control(h, 10., .05)[0], 0.)

    def test_odometry_stop_accepts_unverified_camera_geometry_but_requires_odometry(self):
        for fresh in (True, False):
            warnings = []
            h = SimpleNamespace(mode='follow', cfg={'geometry_verified':False},
                                travel=2., fresh=lambda key, now: fresh,
                                get_logger=lambda: SimpleNamespace(warning=warnings.append))
            h._enter = lambda mode, now: setattr(h, 'mode', mode)
            LimoControl.command_callback(h, String(data=json.dumps(
                dict(mode='pedestrian',distance=.45))))
            if fresh:
                self.assertEqual(h.mode, 'pedestrian')
                self.assertAlmostEqual(h.stop_target, 2.45)
                self.assertFalse(warnings)
            else:
                self.assertEqual(h.mode, 'follow')
                self.assertIn('fresh odometry', h.last_command_error)

    def test_pedestrian_confirms_stop_then_waits_three_seconds(self):
        cfg = dict(command_control_mode=1, command_mode_settle=.2, lap_timeout=600.,
                   default_speed=.3, max_speed=.3, uncertain_speed=.12,
                   slow_speed=.2, auto_obstacle_mode=False, stopped_speed=.005,
                   pedestrian_wait=3., mission_timeout=15., stop_tolerance=.02,
                   stop_confirm_time=.2)
        h = SimpleNamespace(cfg=cfg, mode='pedestrian', phase='approach_stop',
                            phase_travel=0., travel=.45, vehicle_control_mode=1,
                            command_ready_since=0., e_stop_flag=True, lidar_enabled=False,
                            vehicle_ok=True, odom_fault=False, started_at=None,
                            mission_distance=.45, stop_target=.45, odom_speed=0.,
                            stationary_since=None, stopped_at=None, pose=(.45,0.,0.),
                            last_curvature=0., lane=dict(valid=True,confidence=1.),
                            slow_sign_until=None, slow_until=0., multi_frames=0,
                            get_parameter=lambda name: SimpleNamespace(value=True),
                            fresh=lambda key, now: True, steering_curvature=lambda *args: 0.)
        h._enter = lambda mode, now: LimoControl._enter(h,mode,now)
        h.compute_control = lambda now, dt: LimoControl.compute_control(h,now,dt)
        h.immediate_stop = True
        h.stop_target = None
        h.odom_speed = .2
        self.assertEqual(h.compute_control(9.,.05)[0], 0.)
        self.assertIsNone(h.stopped_at)
        h.travel = .52  # Braking travel must not cause a distance-target overshoot.
        h.odom_speed = 0.
        self.assertEqual(h.compute_control(10.,.05)[0], 0.)
        self.assertEqual(h.compute_control(10.21,.05)[0], 0.)
        self.assertEqual(h.phase,'waiting')
        self.assertEqual(h.compute_control(13.20,.05)[0], 0.)
        self.assertGreater(h.compute_control(13.22,.05)[0], 0.)
        self.assertEqual(h.mode,'follow')

    def test_requested_cruise_slow_and_low_confidence_speeds(self):
        cfg = dict(command_control_mode=1, command_mode_settle=.2, lap_timeout=600.,
                   default_speed=.8, max_speed=.8, uncertain_speed=.2,
                   slow_speed=.2, auto_obstacle_mode=False)
        h = SimpleNamespace(cfg=cfg, mode='follow', phase='running', phase_travel=0.,
                            travel=0., vehicle_control_mode=1, command_ready_since=0.,
                            e_stop_flag=True, lidar_enabled=False, vehicle_ok=True,
                            odom_fault=False, started_at=None, mission_distance=None,
                            lane=dict(valid=True, confidence=1.), stop_target=None,
                            slow_sign_until=None, slow_until=0., multi_frames=0,
                            get_parameter=lambda name: SimpleNamespace(value=True),
                            fresh=lambda key, now: True,
                            steering_curvature=lambda *args: 0.)
        for mode, confidence, expected in (('follow',1.,.8), ('slow',1.,.2),
                                            ('follow',.7,.2), ('slow',.7,.2)):
            h.mode, h.lane['confidence'] = mode, confidence
            self.assertAlmostEqual(LimoControl.compute_control(h, 10., .05)[0], expected)
        # High curvature still slows normal tracking but must not lower slow mode.
        h.steering_curvature = lambda *args: 2.
        h.mode, h.lane['confidence'] = 'follow', 1.
        self.assertAlmostEqual(LimoControl.compute_control(h, 10., .05)[0], .8/2.8)
        h.mode, h.lane['confidence'] = 'slow', .7
        self.assertAlmostEqual(LimoControl.compute_control(h, 10., .05)[0], .2)
        # Fixed slow setpoint must not bypass stale-camera or vehicle-fault stops.
        h.fresh = lambda key, now: key != 'lane'
        self.assertEqual(LimoControl.compute_control(h, 10., .05)[0], 0.)
        h.fresh = lambda key, now: True
        h.vehicle_ok = False
        self.assertEqual(LimoControl.compute_control(h, 10., .05)[0], 0.)

    def test_lidar_disabled_does_not_require_scan_or_lidar_stop_heartbeat(self):
        cfg = dict(command_control_mode=1, command_mode_settle=.2, lap_timeout=600.,
                   default_speed=.15, max_speed=.25, uncertain_speed=.06,
                   slow_speed=.08, auto_obstacle_mode=False)
        h = SimpleNamespace(cfg=cfg, mode='follow', phase='running', phase_travel=0.,
                            travel=0., vehicle_control_mode=1, command_ready_since=0.,
                            e_stop_flag=True, lidar_enabled=False, vehicle_ok=True,
                            odom_fault=False, started_at=None, mission_distance=None,
                            lane=dict(valid=True, confidence=1.), stop_target=None,
                            slow_sign_until=None, slow_until=0., multi_frames=0,
                            get_parameter=lambda name: SimpleNamespace(value=True),
                            fresh=lambda key, now: key not in ('obstacles','e_stop'),
                            steering_curvature=lambda *args: 0.)
        self.assertGreater(LimoControl.compute_control(h, 10., .05)[0], 0)
        h.lidar_enabled = True
        speed, _, reason = LimoControl.compute_control(h, 10., .05)
        self.assertEqual(speed, 0)
        self.assertIn('stale_or_missing', reason)

    def test_lidar_checks_bypassed_only_when_disabled(self):
        h = SimpleNamespace(lidar_enabled=False)
        self.assertTrue(LimoControl.curve_clear(h, 0., .2))
        h.lidar_enabled = True
        h.cfg = dict(rear_axle_offset=.1)
        h.footprint_clear = lambda *args: False
        self.assertFalse(LimoControl.curve_clear(h, 0., .2))

    def test_inactive_lidar_stop_callback_does_not_override_camera_drive(self):
        outputs = []
        h = SimpleNamespace(lidar_enabled=False, received={},
                            publish_motion=lambda *args: outputs.append(args))
        LimoControl.e_stop_callback(h, SimpleNamespace(data=True))
        self.assertFalse(outputs)
        h.lidar_enabled = True
        LimoControl.e_stop_callback(h, SimpleNamespace(data=True))
        self.assertEqual(outputs, [(0., 0.)])

    def test_motion_mode_does_not_block_when_check_is_disabled(self):
        for motion_mode in (0, 1, 2, 255):
            h = SimpleNamespace(cfg=dict(require_ackermann=False, command_control_mode=1),
                                command_ready_since=None, received={},
                                publish_motion=lambda *args: None)
            LimoControl.vehicle_callback(h, SimpleNamespace(
                error_code=0, motion_mode=motion_mode, control_mode=1))
            self.assertTrue(h.vehicle_ok)
            LimoControl.vehicle_callback(h, SimpleNamespace(
                error_code=1, motion_mode=motion_mode, control_mode=1))
            self.assertFalse(h.vehicle_ok)

    def test_branch_direction_requires_measured_yaw_and_exit(self):
        for mode, sign in (('branch_a', 1), ('branch_b', -1)):
            cfg = dict(command_control_mode=1, command_mode_settle=.2, lap_timeout=600.,
                       geometry_verified=True, branch_approach=.2, branch_angle=.7,
                       branch_yaw_tolerance=.08, branch_speed=.08, branch_curvature=1.5,
                       branch_exit_distance=.3, default_speed=.15, max_speed=.25,
                       uncertain_speed=.06, slow_speed=.08, auto_obstacle_mode=False)
            h = SimpleNamespace(cfg=cfg, mode=mode, phase='turning', phase_heading=0.,
                                phase_travel=0., travel=.4, pose=(.4,0,0),
                                vehicle_control_mode=1, command_ready_since=0.,
                                e_stop_flag=False, vehicle_ok=True, odom_fault=False,
                                started_at=0., mission_distance=None, stop_target=None,
                                lane=dict(valid=True, confidence=1.),
                                slow_sign_until=None, slow_until=0., multi_frames=0,
                                get_parameter=lambda name: SimpleNamespace(value=True),
                                fresh=lambda *args: True,
                                steering_curvature=lambda *args: 0.)
            def enter(new_mode, now):
                h.mode, h.phase = new_mode, 'running'
                h.phase_travel = h.travel
            h._enter = enter
            v,k,reason = LimoControl.compute_control(h, 10., .05)
            self.assertGreater(v, 0)
            self.assertEqual(k, sign*1.5)
            # Travel without turning must not finish the intersection.
            h.travel = 2.
            LimoControl.compute_control(h, 11., .05)
            self.assertEqual(h.phase, 'turning')
            h.pose = (2., 0., sign*.7)
            LimoControl.compute_control(h, 12., .05)
            self.assertEqual(h.phase, 'reacquire')
            h.travel = 2.4
            LimoControl.compute_control(h, 13., .05)
            self.assertEqual(h.mode, 'follow')

    def test_parking_blocked_path_produces_zero(self):
        h = SimpleNamespace(
            cfg=dict(rear_axle_offset=.1, parking_lookahead=.2, deceleration=.4),
            parking_origin=(0,0,0), parking_path=[[.2,0,0,.05],[.4,0,0,.05]],
            parking_index=0, parking_direction=1, pose=(0,0,0),
            max_curvature=1.8, curve_clear=lambda *args: False)
        v,k,reason = LimoControl.parking_control(h)
        self.assertEqual((v,k), (0.,0.))
        self.assertEqual(reason, 'parking_path_blocked_or_unobserved')

    def parking_simulation(self, side):
        cfg = dict(stop_confirm_time=.2, stopped_speed=.005, deceleration=.4,
                   rear_axle_offset=.1, robot_rear=.18, robot_front=.18,
                   robot_width=.22, parking_lookahead=.2)
        box = [-.35, .25, -1.35, -.45] if side == 'right' else [-.35, .25, .45, 1.35]
        h = SimpleNamespace(cfg=cfg, parking_origin=(0,0,0),
                            parking_path=t_parking_path(side,.6,.6,.3,.08,.05,.1,.2,.35),
                            parking_box=box, parking_index=0, parking_direction=1,
                            parking_shift_since=None, pose=(0,0,0), odom_speed=0.,
                            phase='running', max_curvature=math.tan(.35)/.2,
                            curve_clear=lambda *args: True)
        h.parking_control = lambda: LimoControl.parking_control(h)
        reversals = 0
        previous_direction = 1
        for i in range(10000):
            with patch('limo_control.time.monotonic', return_value=i*.04):
                v,k,reason = h.parking_control()
            if reason == 'parked_four_corners_inside':
                self.assertEqual(h.phase, 'complete')
                self.assertEqual(reversals, 1)
                return
            if h.parking_direction != previous_direction:
                self.assertEqual(h.odom_speed, 0)
                reversals += 1
                previous_direction = h.parking_direction
            x,y,yaw = h.pose
            dt = .04
            h.pose = (x+v*(math.cos(yaw)-.1*k*math.sin(yaw))*dt,
                      y+v*(math.sin(yaw)+.1*k*math.cos(yaw))*dt,
                      yaw+v*k*dt)
            h.odom_speed = v
        self.fail(f'{side} parking did not complete: {reason}, index={h.parking_index}, pose={h.pose}')

    def test_right_t_parking_tracks_generated_path(self):
        self.parking_simulation('right')

    def test_left_t_parking_tracks_generated_path(self):
        self.parking_simulation('left')


if __name__ == '__main__':
    unittest.main()
