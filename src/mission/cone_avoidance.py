"""LiDAR gap selection extracted from track_drive.py (Xycar scan layout).

This module computes a result only. The caller owns mode switching and
motor commands. Steering values use the original Xycar convention.
"""

from dataclasses import dataclass
from math import isfinite
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class ConeAvoidanceResult:
    steering: float
    obstacle_points: int
    finished: bool


class ConeAvoidance:
    def __init__(self) -> None:
        self.started = False

    def reset(self) -> None:
        self.started = False

    @staticmethod
    def obstacle_indices(ranges: Sequence[float]) -> Optional[Tuple[int, ...]]:
        """Return hits in the original forward sector, or None for no usable scan."""
        if len(ranges) < 360:
            return None

        forward = tuple(ranges[270:360]) + tuple(ranges[:91])
        if not any(isfinite(distance) and distance > 0 for distance in forward):
            return None
        return tuple(
            index for index, distance in enumerate(forward)
            if 0 < distance < 3.0
        )

    def update(self, ranges: Sequence[float]) -> Optional[ConeAvoidanceResult]:
        """Find the widest gap. Call only while the cone mission is active."""
        obstacles = self.obstacle_indices(ranges)
        if obstacles is None:
            return None
        if not obstacles:
            return ConeAvoidanceResult(0.0, 0, self.started)

        self.started = True
        middle_width, middle_center = 0, 90
        for first, second in zip(obstacles, obstacles[1:]):
            width = second - first
            if width > 25 and width > middle_width:
                middle_width, middle_center = width, (first + second) // 2

        left_width = 180 - obstacles[-1]
        right_width = obstacles[0]
        widest = max(left_width, right_width, middle_width)
        if widest == left_width:
            center = 180 - left_width // 2
        elif widest == right_width:
            center = right_width // 2
        else:
            center = middle_center

        steering = float(max(-100, min(100, 3 * (90 - center))))
        return ConeAvoidanceResult(steering, len(obstacles), False)
