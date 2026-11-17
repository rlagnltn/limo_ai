"""Replay split/merge images through the v1 tracker and jihyuk detector."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[2] / 'line_slow'))
from detect_line import DetectLine
from lane_geometry import MergeTracker
from limo_control import LimoControl


class MergeTests(unittest.TestCase):
    @staticmethod
    def image(split=False, right=False, empty=False):
        image = np.zeros((480,640,3), dtype=np.uint8)
        if not empty:
            for x in ((50,320,590) if split else (50,590)):
                cv2.line(image,(x,280),(x,479),(0,200,200),8)
        return cv2.flip(image,1) if right else image

    def detector(self, mode=None):
        params = dict(merge_reference_distance=60., merge_confirm_sec=.5,
                      merge_width_tolerance_px=20., merge_boundary_margin_px=60)
        return SimpleNamespace(get_parameter=lambda key: SimpleNamespace(value=params[key]),
                               yellow_lane_low=np.array([0,90,70]),
                               yellow_lane_high=np.array([60,220,255]),
                               merge_tracker=MergeTracker(), merge_direction=None,
                               controller_received_at=time.monotonic(),
                               controller_status=dict(mode=mode,phase='merging'))

    def test_single_corridor_without_observed_split_never_completes(self):
        h = self.detector('branch_a')
        for now in (1.,2.,3.):
            h.controller_received_at = now
            with patch('detect_line.time.monotonic',return_value=now):
                result = DetectLine.merge_lane_state(h,self.image(),280,480)
            self.assertTrue(result['merge_valid'])
            self.assertFalse(result['merge_complete'])

    def test_left_and_right_split_then_stable_merge_and_boundary_loss(self):
        for mode, right, sign in (('branch_a',False,1),('branch_b',True,-1)):
            h = self.detector(mode)
            for now, split, complete in ((1.,True,False),(2.,False,False),(2.6,False,True)):
                h.controller_received_at = now
                with patch('detect_line.time.monotonic',return_value=now):
                    result = DetectLine.merge_lane_state(h,self.image(split,right),280,480)
                self.assertEqual(result['merge_mode'],mode)
                self.assertTrue(result['merge_valid'])
                self.assertGreater(sign*result['merge_offset'],0.)
                self.assertEqual(result['merge_complete'],complete)
            h.controller_received_at = 3.
            with patch('detect_line.time.monotonic',return_value=3.):
                result = DetectLine.merge_lane_state(h,self.image(empty=True),280,480)
            self.assertFalse(result['merge_valid'])
            self.assertFalse(result['merge_complete'])

    def test_normal_lane_and_stale_controller_do_not_activate_merge(self):
        for mode, stale in ((None,False),('branch_a',True)):
            h = self.detector(mode)
            if stale:
                h.controller_received_at -= 1.
            result = DetectLine.merge_lane_state(h,self.image(split=True),280,480)
            self.assertIsNone(result['merge_mode'])
            self.assertFalse(result['merge_valid'])
            self.assertNotIn('offset',result)
            self.assertNotIn('valid',result)

    def test_invalid_merge_packet_cannot_authorize_motion(self):
        import json
        for offset in (float('nan'),float('inf'),10000):
            h = SimpleNamespace(lane={},received={},
                                get_logger=lambda: None)
            packet = dict(version=1,width=640,offset=0,valid=True,lane_count=1,
                          merge_mode='branch_a',merge_valid=True,merge_complete=True,
                          merge_offset=offset)
            LimoControl.lane_callback(h,SimpleNamespace(data=json.dumps(packet)))
            self.assertEqual(h.lane,{})
            self.assertNotIn('lane',h.received)


if __name__ == '__main__':
    unittest.main()
