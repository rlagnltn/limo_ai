"""Prototype T-parking: forward alignment, reverse quarter-circle, reverse straight.

Path/box coordinates are relative to the vehicle when class 6 is confirmed.
This is a tunable trajectory, not a parking-space perception algorithm.
"""
import math


def t_parking_path(side, approach, radius, reverse, sample, speed, offset, wheelbase, max_angle):
    values = (approach, radius, reverse, sample, speed, offset, wheelbase, max_angle)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Parking parameters must be finite')
    if (side not in ('left', 'right') or min(approach, radius, reverse, sample, speed, wheelbase) <= 0
            or offset < 0 or not 0 < max_angle < math.pi/2):
        raise ValueError('Invalid T-parking geometry')
    if radius < wheelbase/math.tan(max_angle):
        raise ValueError('Parking radius is smaller than vehicle turning radius')
    steps = sum(math.ceil(length/sample) for length in (approach, radius*math.pi/2, reverse))
    if steps > 98:
        raise ValueError('Parking path exceeds 100 points; increase parking_sample')
    x, y, yaw = -offset, 0.0, 0.0  # rear axle relative to entry base_link
    path = []
    turn = -1.0 if side == 'right' else 1.0
    for length, curvature, direction in ((approach, 0.0, 1),
                                         (radius*math.pi/2, turn/radius, -1),
                                         (reverse, 0.0, -1)):
        count = math.ceil(length/sample)
        ds = direction*length/count
        for _ in range(count):
            next_yaw = yaw+curvature*ds
            if curvature:
                x += (math.sin(next_yaw)-math.sin(yaw))/curvature
                y += (math.cos(yaw)-math.cos(next_yaw))/curvature
            else:
                x += ds*math.cos(yaw)
                y += ds*math.sin(yaw)
            yaw = next_yaw
            path.append([x+offset*math.cos(yaw), y+offset*math.sin(yaw), yaw, direction*speed])
    return path
