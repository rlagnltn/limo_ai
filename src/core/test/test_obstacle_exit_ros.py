"""Regress early LiDAR release, off-centre exits and rotated road bounds."""
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).parents[2]/'line_slow'))
sys.path.insert(0,str(Path(__file__).parents[1]))
from limo_control import LimoControl
from detect_line import DetectLine
from core.missions.mission_04_obstacle import ObstacleMission, DEFAULTS
from core.missions.base import Observation
import test_core_lidar_gate_ros as core_tests
Core = core_tests.Core


class ObstacleExitTests(unittest.TestCase):
    def core(self, signs=(2,)):
        h,_,_=core_tests.CoreLidarGateTests().harness('intersection',True)
        h.lidar_latched=True
        h.estop=False;h.last_estop=time.monotonic()
        h.controller.fresh=lambda *args: True
        h.controller.obstacles=dict(valid=True,points=[])
        h.controller.curve_clear=lambda *args: True
        h.tracker.signs=lambda now: frozenset(signs)
        return h

    def test_lidar_releases_only_for_one_direction_and_clear_fresh_scan(self):
        for signs in ((),(2,),(3,),(2,3)):
            h=self.core(signs)
            self.assertEqual(Core.update_lidar_request(h),len(signs)!=1)
        for problem in ('obstacle','stale_scan','unknown_path','estop','pending_motor'):
            h=self.core()
            if problem=='obstacle':h.controller.obstacles['points']=[[.5,0.]]
            if problem=='stale_scan':h.controller.fresh=lambda *args: False
            if problem=='unknown_path':h.controller.curve_clear=lambda *args: False
            if problem=='estop':h.estop=True
            if problem=='pending_motor':h.lidar_pending=True
            self.assertTrue(Core.update_lidar_request(h),problem)

    def test_stage_change_without_direction_keeps_scan_and_sensor_checks(self):
        h=self.core(())
        self.assertTrue(Core.update_lidar_request(h))
        self.assertTrue(h.controller.lidar_enabled)
        h.course.active.name='traffic'
        self.assertTrue(Core.update_lidar_request(h))
        h.fault='stopped_on_fault'
        self.assertFalse(Core.update_lidar_request(h))

    def test_cone_lidar_lifecycle_is_unchanged(self):
        h=self.core(())
        h.course.active.name='cone'
        self.assertTrue(Core.update_lidar_request(h))
        h.course.active.name='traffic'
        self.assertFalse(Core.update_lidar_request(h))

    def test_obstacle_exit_requires_center_heading_sign_and_measured_clearance(self):
        lane=dict(valid=True,metric_valid=True,lane_count=1,
                  ground_centers=[[.8,0.]],ground_heading=0.,ground_lines=[[0.,.825],[0.,-.825]])
        for problem in ('none','no_sign','conflicting_signs','off_center','lookahead_only','heading','no_metric','blocked','unknown_scan'):
            mission=ObstacleMission(DEFAULTS)
            mission.phase='avoiding'
            sample=dict(lane=dict(lane),signs=frozenset({2}),lidar_clear=True,points=())
            if problem=='no_sign':sample['signs']=frozenset()
            if problem=='conflicting_signs':sample['signs']=frozenset({2,3})
            if problem=='off_center':sample['lane']['ground_lines']=[[0.,1.375],[0.,-.275]]
            if problem=='lookahead_only':
                sample['lane'].update(ground_heading=.119,ground_centers=[[.8,-.075]],
                                      ground_lines=[[math.tan(.119),.655/math.cos(.119)],
                                                    [math.tan(.119),-.995/math.cos(.119)]])
            if problem=='heading':sample['lane']['ground_heading']=.2
            if problem=='no_metric':sample['lane']['metric_valid']=False
            if problem=='blocked':sample['points']=((.5,0.),)
            if problem=='unknown_scan':sample['lidar_clear']=False
            mission.update(Observation(10.,1.2,**sample))
            result=mission.update(Observation(11.1,1.2,**sample))
            self.assertEqual(result.done,problem=='none',problem)

    def test_rotated_road_does_not_use_a_single_cross_section_as_a_wall(self):
        cfg=dict(robot_width=.22,safety_margin=.04,robot_front=.18,robot_rear=.18)
        h=NS(cfg=cfg,obstacles=dict(points=[],coverage=[1.]*36,range_limit=3.,sensor_origin=[0.,0.]))
        yaw=.2976211423338689;y=.43737244913915085
        slope=-math.tan(yaw)
        bounds=[[slope,(.825-y)/math.cos(yaw)],[slope,(-.825-y)/math.cos(yaw)]]
        self.assertTrue(LimoControl.footprint_clear(h,.28,-.011,-.102,road_bounds=bounds))
        self.assertFalse(LimoControl.footprint_clear(h,.28,-.011,-.102,road_bounds=[.160,-1.565]))

    def test_detector_publishes_metric_boundary_direction(self):
        h=NS(roi_=np.zeros((200,640,3),dtype=np.uint8),lane_confidence=1.,lane_heading_error=0.,
             boundary_samples=[(100,540,y) for y in range(20,181,20)],
             ground_point=lambda x,y:[(480-y)*.01,(320-x)*.003],
             longitudinal_white_mask=DetectLine.longitudinal_white_mask)
        state=DetectLine.road_state(h,np.zeros((200,640,3),dtype=np.uint8),320,320,100,540,100,0,280)
        self.assertTrue(state['metric_valid'])
        self.assertAlmostEqual(state['ground_heading'],0.)
        self.assertAlmostEqual(state['ground_lines'][0][1],.66)
        self.assertAlmostEqual(state['ground_lines'][1][1],-.66)


if __name__=='__main__':unittest.main()
