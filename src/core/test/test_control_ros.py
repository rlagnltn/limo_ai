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
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ControllerTests(unittest.TestCase):
    def test_cone_scan_requires_three_entry_and_five_exit_frames(self):
        from cone_follow import ConeFollower, ModeMachine, Params, Plan
        from sensor_msgs.msg import LaserScan

        follower = ConeFollower(Params())
        h = SimpleNamespace(mode='cones', phase='cone_waiting_for_rows',
                            cone_follower=follower, cone_mode=ModeMachine(follower.p),
                            cone_scan_time=-math.inf, vehicle_motion_mode=1,
                            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9)))
        scan = LaserScan()
        scan.header.stamp.sec = 1
        scan.ranges = [1.]
        present = Plan(v=.15, k=1.9, both_near=True, cones_ahead=2, reason='both')
        absent = Plan(reason='no_cones')
        with patch.object(follower, 'update', side_effect=[present]*3+[absent]*5):
            for i in range(3):
                LimoControl.cone_scan_callback(h, scan)
                self.assertEqual(LimoControl.cone_control(h, h.cone_scan_time)[:2],
                                 (.15, 1.9) if i == 2 else (0., 0.))
            self.assertEqual(LimoControl.cone_control(h, h.cone_scan_time+.31)[:2], (0., 0.))
            for i in range(5):
                LimoControl.cone_scan_callback(h, scan)
                self.assertEqual(h.phase, 'cone_acquire' if i == 4 else 'cone_tracking')
            self.assertEqual(LimoControl.cone_control(h, h.cone_scan_time)[:2], (0., 0.))

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

    def test_pedestrian_confirms_stop_then_waits_four_seconds(self):
        cfg = dict(command_control_mode=1, command_mode_settle=.2, lap_timeout=600.,
                   default_speed=.3, max_speed=.3, uncertain_speed=.12,
                   slow_speed=.2, auto_obstacle_mode=False, stopped_speed=.005,
                   pedestrian_wait=4., mission_timeout=15., stop_tolerance=.02,
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
        self.assertEqual(h.compute_control(14.20,.05)[0], 0.)
        self.assertGreater(h.compute_control(14.22,.05)[0], 0.)
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
        h.cfg['lap_timeout'], h.started_at = 0., 0.
        self.assertAlmostEqual(LimoControl.compute_control(h, 601., .05)[0], .8/2.8)
        h.mode, h.lane['confidence'] = 'slow', .7
        self.assertAlmostEqual(LimoControl.compute_control(h, 10., .05)[0], .2)
        # Fixed slow setpoint must not bypass stale-camera or vehicle-fault stops.
        h.fresh = lambda key, now: key != 'lane'
        self.assertEqual(LimoControl.compute_control(h, 10., .05)[0], 0.)
        h.fresh = lambda key, now: True
        h.vehicle_ok = False
        self.assertEqual(LimoControl.compute_control(h, 10., .05)[0], 0.)

    def test_team_lane_candidate_respects_slow_cap_stop_and_freshness(self):
        command = Twist()
        command.linear.x, command.angular.z = .25, -.125
        control = SimpleNamespace(
            cfg=dict(command_control_mode=1, command_mode_settle=.2, lap_timeout=600.,
                     slow_speed=.2),
            mode='follow', phase='running', phase_travel=0., travel=0.,
            vehicle_control_mode=1, command_ready_since=0., lidar_enabled=False,
            vehicle_ok=True, odom_fault=False, started_at=None, mission_distance=None,
            lane=dict(valid=False), use_team_lane=True, team_lane_command=command,
            slow_sign_until=None, slow_until=0.,
            get_parameter=lambda name: SimpleNamespace(value=True),
            fresh=lambda key, now: True,
        )
        self.assertEqual(LimoControl.compute_control(control, 10., .05),
                         (.25, -.5, 'team_lane_follow'))
        control.mode = 'slow'
        self.assertEqual(LimoControl.compute_control(control, 10., .05),
                         (.2, -.5, 'team_lane_follow'))
        command.linear.x = -.05
        self.assertEqual(LimoControl.compute_control(control, 10., .05),
                         (-.05, 0., 'team_lane_reverse_search'))
        command.linear.x = 0.
        self.assertEqual(LimoControl.compute_control(control, 10., .05),
                         (0., 0., 'team_lane_stop'))
        control.fresh = lambda key, now: key != 'team_lane'
        self.assertEqual(LimoControl.compute_control(control, 10., .05),
                         (0., 0., 'team_lane_stale_or_missing'))

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

    def test_branch_fixed_steering_then_resumes_team_lane_without_legacy_lane(self):
        for mode, sign in (('branch_a', 1), ('branch_b', -1)):
            cfg = dict(command_control_mode=1, command_mode_settle=.2, lap_timeout=600.,
                       geometry_verified=False, default_speed=.15, max_speed=.8, wheelbase=.2,
                       uncertain_speed=.06, slow_speed=.08, auto_obstacle_mode=False)
            h = SimpleNamespace(cfg=cfg, mode=mode, phase='branch_fixed_steering',
                                branch_fixed_elapsed=0., branch_fixed_output_at=None,
                                phase_travel=0., travel=0., vehicle_control_mode=1,
                                command_ready_since=0., e_stop_flag=False, vehicle_ok=True,
                                odom_fault=False, started_at=0., mission_distance=None,
                                stop_target=None, lane={'valid': False}, slow_sign_until=None,
                                slow_until=0., multi_frames=0,
                                get_parameter=lambda name: SimpleNamespace(value=True),
                                fresh=lambda *args: True, steering_curvature=lambda *args: 0.)
            h.fixed_branch_curvature = lambda: LimoControl.fixed_branch_curvature(h)
            h._enter = lambda mode, now: setattr(h, 'mode', mode)
            speed, curvature, reason = LimoControl.compute_control(h, 10., .05)
            self.assertEqual((speed, reason), (.3, 'branch_fixed_steering'))
            self.assertAlmostEqual(math.degrees(math.atan(curvature*.2)), sign*15.)
            command = Twist()
            command.linear.x, command.angular.z = speed, speed*curvature
            LimoControl.record_fixed_branch_output(h, 0., command)
            LimoControl.record_fixed_branch_output(h, .05, Twist())
            LimoControl.record_fixed_branch_output(h, .1, command)
            self.assertEqual(h.branch_fixed_elapsed, 0.)
            for step in range(2, 31):
                LimoControl.record_fixed_branch_output(h, step/10, command)
            self.assertEqual(LimoControl.compute_control(h, 10., .05)[2],
                             'branch_fixed_steering')
            LimoControl.record_fixed_branch_output(h, 3.15, command)
            h.use_team_lane = True
            h.team_lane_command = Twist()
            h.team_lane_command.linear.x = .15
            self.assertEqual(LimoControl.compute_control(h, 10., .05),
                             (.15, 0., 'team_lane_follow'))
            self.assertEqual(h.mode, 'follow')
            h.team_lane_command.linear.x = 0.
            self.assertEqual(LimoControl.compute_control(h, 10.05, .05),
                             (0., 0., 'team_lane_stop'))

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
