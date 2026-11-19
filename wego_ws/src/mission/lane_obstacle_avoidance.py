"""Camera lane following and LiDAR-triggered lane changes from track_drive.py.

Inputs and outputs assume the original Xycar calibration. The caller owns
mode selection, target-lane clearance checks and motor commands.
"""
from dataclasses import dataclass
from math import isfinite
from typing import Optional, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class LaneAvoidanceResult:
    steering: float
    lane: str
    target_lane: Optional[str]
    obstacle_detected: bool


class LaneObstacleAvoidance:
    def __init__(self, obstacle_distance: float = 10.0,
                 lane_hsv_low=(15, 70, 70), lane_hsv_high=(40, 255, 255)) -> None:
        self.obstacle_distance = obstacle_distance
        self.lane_hsv_low = tuple(lane_hsv_low)
        self.lane_hsv_high = tuple(lane_hsv_high)
        for bounds in (self.lane_hsv_low, self.lane_hsv_high):
            if len(bounds) != 3 or any(type(v) is not int for v in bounds):
                raise ValueError("HSV bounds must contain three integers")
        if any(not 0 <= lo <= hi <= maximum for lo, hi, maximum in
               zip(self.lane_hsv_low, self.lane_hsv_high, (179, 255, 255))):
            raise ValueError("Invalid HSV bounds")
        self.reset()

    def reset(self, lane: str = "left") -> None:
        if lane not in ("left", "right"):
            raise ValueError("lane must be left or right")
        self.lane = lane
        self.target_lane = None

    def color_mask(self, image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv, self.lane_hsv_low, self.lane_hsv_high)

    @staticmethod
    def warp_mask(lane_mask: np.ndarray) -> np.ndarray:
        """Apply the original perspective transform, also used by preview."""
        height, width = lane_mask.shape[:2]
        if height <= 260 or width <= 400:
            raise ValueError("Frame is too small for the original Xycar perspective transform")
        source = np.float32([(0, height), (200, 260), (width - 200, 260), (width, height)])
        destination = np.float32([(100, height), (100, 0), (width - 100, 0), (width - 100, height)])
        transform = cv2.getPerspectiveTransform(source, destination)
        return cv2.warpPerspective(lane_mask, transform, (width, height))

    def detect_lane_indices(self, image: np.ndarray) -> dict:
        """Detect yellow-lane positions in eight bands of the warped image."""
        height, width = image.shape[:2]
        warped = self.warp_mask(self.color_mask(image)) != 0
        center, band_height = width // 2, height // 8
        indices = {"left": [], "right": []}
        for band in range(8):
            end = height - band * band_height
            rows = warped[end - band_height:end]
            for lane, start, stop in (("left", 0, center), ("right", center, width)):
                columns = np.flatnonzero(rows[:, start:stop].sum(axis=0) >= 40)
                if columns.size:
                    indices[lane].append(start + (columns[0] + columns[-1]) // 2)
        return indices

    def update(self, image: np.ndarray, ranges: Sequence[float]) -> Optional[LaneAvoidanceResult]:
        """Compute steering from a BGR frame and an Xycar LiDAR scan."""
        if len(ranges) < 360:
            return None
        front = tuple(ranges[353:360]) + tuple(ranges[:8])
        valid = [distance for distance in front if isfinite(distance) and distance > 0]
        if not valid:
            return None
        obstacle = min(valid) < self.obstacle_distance
        lanes = self.detect_lane_indices(image)

        if self.target_lane is None and obstacle:
            self.target_lane = "right" if self.lane == "left" else "left"
            steering = 40.0 if self.target_lane == "right" else -40.0
            return LaneAvoidanceResult(steering, self.lane, self.target_lane, obstacle)

        if self.target_lane is not None:
            target = self.target_lane
            minimum_bands = 4 if target == "right" else 5
            if len(lanes[target]) < minimum_bands:
                steering = 33.0 if target == "right" else -20.0
                return LaneAvoidanceResult(steering, self.lane, target, obstacle)
            self.lane = target
            self.target_lane = None

        positions = lanes[self.lane]
        if not positions:
            return None
        reference = 160 if self.lane == "left" else image.shape[1] - 160
        steering = (float(np.mean(positions)) - reference) * (200 / 640)
        return LaneAvoidanceResult(steering, self.lane, None, obstacle)


if __name__ == '__main__':
    from lane_obstacle_avoidance_ros import main
    main()
