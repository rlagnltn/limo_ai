"""Measure lane targets and the corridor used by IMU zigzag control."""
from dataclasses import dataclass
import cv2
import numpy as np
import time


def is_v_marking(mask):
    """Find two diagonal arms converging to a lower vertex in the road mask."""
    lines = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=25,
                            minLineLength=40, maxLineGap=12)
    if lines is None:
        return False
    left, right = [], []
    for x0, y0, x1, y1 in lines[:, 0]:
        x0, y0, x1, y1 = map(float, (x0, y0, x1, y1))
        if y0 > y1:
            x0, y0, x1, y1 = x1, y1, x0, y0
        dx, dy = x1 - x0, y1 - y0
        if abs(dx) < 35 or dy < 8 or not .15 <= dy / abs(dx) <= 1.5:
            continue
        slope = dx / dy
        segment = (x0, y0, x1, y1, slope, x0 - slope * y0)
        (left if dx > 0 else right).append(segment)
    for a in left:
        for b in right:
            overlap = min(a[3], b[3]) - max(a[1], b[1])
            if (overlap < max(6, .3 * min(a[3] - a[1], b[3] - b[1]))
                    or b[0] - a[0] < 80):
                continue
            vy = (b[5] - a[5]) / (a[4] - b[4])
            vx = a[4] * vy + a[5]
            if (0 <= vy <= mask.shape[0] + 30 and vy >= max(a[3], b[3]) - 12
                    and vy - min(a[3], b[3]) <= 45
                    and a[0] < vx < b[0]
                    and max(abs(vx - a[2]), abs(vx - b[2])) <= 35):
                return True
    return False


def is_curve_mask(mask, min_deviation_px):
    """Measure bending relative to a straight fit, independent of line tilt."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return False
    lane = labels == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    rows, positions = [], []
    for y in range(0, mask.shape[0], 5):
        xs = np.nonzero(lane[y:y + 5])[1]
        if xs.size:
            rows.append(y + 2)
            positions.append(float(np.median(xs)))
    if len(rows) < 6:
        return False
    straight = np.polyval(np.polyfit(rows, positions, 1), rows)
    deviation = np.percentile(np.abs(np.array(positions) - straight), 90)
    return bool(deviation >= min_deviation_px)


def is_zigzag_mask(mask, min_segment_px, min_reversals):
    """Detect repeated left/right turns in the largest masked component."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return False
    lane_label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    lane = labels == lane_label
    positions = []
    for y in range(0, mask.shape[0], 5):
        xs = np.nonzero(lane[y:y + 5])[1]
        if xs.size:
            positions.append(float(np.median(xs)))
    if len(positions) < 4:
        return False

    direction = 0
    extreme = positions[0]
    reversals = 0
    for x in positions[1:]:
        if direction == 0:
            if abs(x - extreme) >= min_segment_px:
                direction = 1 if x > extreme else -1
                extreme = x
        elif direction * (x - extreme) > 0:
            extreme = x
        elif direction * (x - extreme) <= -min_segment_px:
            reversals += 1
            direction = -direction
            extreme = x
            if reversals >= min_reversals:
                return True
    return False


def process_lane_mask(mask, min_segment_px, min_reversals):
    if is_zigzag_mask(mask, min_segment_px, min_reversals):
        return fit_straight_lane(mask)
    moments = cv2.moments(mask)
    if not moments['m00']:
        return mask, None, None
    center = (int(moments['m10'] / moments['m00']),
              int(moments['m01'] / moments['m00']))
    return mask, center, None


class MergeTracker:
    """Keep the chosen outer boundary until a split becomes one stable corridor."""

    def __init__(self, min_pixels=20, margin=60, confirm_sec=0.5, width_tolerance=20):
        self.min_pixels = min_pixels
        self.margin = margin
        self.confirm_sec = confirm_sec
        self.width_tolerance = width_tolerance
        self.previous = [None, None, None]
        self.split_seen = False
        self.stable_since = None
        self.stable_widths = None
        self.complete = False

    def process(self, mask, offset=0, now=None):
        now = time.monotonic() if now is None else now
        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        keep = stats[:, cv2.CC_STAT_AREA] >= self.min_pixels
        keep[0] = False
        clean = keep[labels]
        height = mask.shape[0] - offset
        selected, widths, multiple = [], [], []
        for index, fraction in enumerate((0.25, 0.5, 0.75)):
            row = offset + int((height - 1) * fraction)
            positions, paired_widths = [], []
            multi_rows = 0
            for y in range(max(offset, row - 5), min(mask.shape[0], row + 6)):
                xs = np.flatnonzero(clean[y])
                if not xs.size:
                    continue
                runs = np.split(xs, np.flatnonzero(np.diff(xs) > 1) + 1)
                candidates = np.array([float(np.median(run)) for run in runs])
                if self.previous[index] is None:
                    outer = candidates[candidates < mask.shape[1] / 2]
                    chosen = float(outer.min()) if outer.size else None
                else:
                    nearest = int(np.argmin(np.abs(candidates - self.previous[index])))
                    chosen = (float(candidates[nearest])
                              if abs(candidates[nearest] - self.previous[index]) <= self.margin
                              else None)
                if chosen is not None:
                    positions.append(chosen)
                multi_rows += len(candidates) >= 3
                if (len(candidates) == 2
                        and candidates[0] < mask.shape[1] / 2 <= candidates[1]
                        and chosen == candidates[0]):
                    paired_widths.append(candidates[1] - candidates[0])
            value = float(np.median(positions)) if len(positions) >= 4 else None
            selected.append(value)
            if value is not None:
                self.previous[index] = value
            widths.append(float(np.median(paired_widths))
                          if len(paired_widths) >= 4 and not multi_rows else None)
            multiple.append(multi_rows >= 4)

        self.split_seen = self.split_seen or sum(multiple) >= 2
        single = all(value is not None for value in selected + widths)
        if not self.complete:
            if self.split_seen and single:
                widths = np.array(widths)
                if (self.stable_widths is None
                        or np.max(np.abs(widths - self.stable_widths)) > self.width_tolerance):
                    self.stable_widths = widths
                    self.stable_since = now
                self.complete = now - self.stable_since >= self.confirm_sec
            else:
                self.stable_since = None
                self.stable_widths = None
        cy = (height - 1) // 2
        center = (int(round(selected[1])), cy) if selected[1] is not None else None
        line = None if center is None else ((center[0], 0), (center[0], height - 1))
        return mask[offset:].copy(), center, line


@dataclass(frozen=True)
class CorridorMeasurement:
    # Image tilt is an alignment cue, not a calibrated vehicle yaw angle.
    tilt_rad: float
    center_error_px: float
    left_clearance_px: float
    right_clearance_px: float
    bend_px: float


class LaneTracker:
    """Smooth tracking errors across left-lane and two-lane center tracking."""

    def __init__(self, min_segment_px=12, min_reversals=2,
                 exit_delay=0.5, smoothing_sec=0.25,
                 max_center_speed=120.0, min_lane_pixels=20, *,
                 center_band_px=40, boundary_margin_px=60,
                 center_deadband_px=2.0, center_hold_sec=0.3):
        self.min_segment_px = min_segment_px
        self.min_reversals = min_reversals
        self.exit_delay = exit_delay
        self.smoothing_sec = smoothing_sec
        self.max_center_speed = max_center_speed
        self.min_lane_pixels = min_lane_pixels
        self.center_band_px = center_band_px
        self.boundary_margin_px = boundary_margin_px
        self.center_deadband_px = center_deadband_px
        self.center_hold_sec = center_hold_sec
        self.boundaries = None
        self.last_pair_time = float('-inf')
        self.center_estimated = False
        self.zigzag = False
        self.tracking_center = False
        self.reference_x = 0
        self.last_seen = float('-inf')
        self.last_time = None
        self.last_x = None
        self.transition = False
        self.corridor = None
        self.exit_since = None

    def process(self, mask, lookahead_mask, offset=0, now=None, *,
                lane_mask=None, reference_x=0, center_reference_x=320):
        now = time.monotonic() if now is None else now
        was_zigzag = self.zigzag
        detected = is_zigzag_mask(lookahead_mask, self.min_segment_px, self.min_reversals)
        if not detected and lane_mask is not None:
            detected = is_zigzag_mask(lane_mask[:, int(center_reference_x):],
                                     self.min_segment_px, self.min_reversals)
        if detected:
            self.last_seen = now
            self.zigzag = True
            self.exit_since = None

        self.tracking_center = False
        self.center_estimated = False
        self.corridor = None
        center = None
        if self.zigzag and lane_mask is not None:
            if now - self.last_pair_time > self.center_hold_sec:
                self.boundaries = None
            left, right, cy = measure_lane_boundaries(
                lane_mask, center_reference_x, offset, self.min_lane_pixels,
                previous=self.boundaries, band_height=self.center_band_px,
                margin=self.boundary_margin_px)
            if left is not None and right is not None:
                self.boundaries = (left, right)
                self.last_pair_time = now
                self.corridor = measure_corridor(
                    lane_mask, center_reference_x, offset, self.min_lane_pixels,
                    band_height=self.center_band_px, boundaries=(left, right))
            elif (self.boundaries is not None
                  and now - self.last_pair_time <= self.center_hold_sec
                  and (left is not None or right is not None)):
                width = self.boundaries[1] - self.boundaries[0]
                if left is not None:
                    right = left + width
                else:
                    left = right - width
                self.boundaries = (left, right)
                self.center_estimated = True
            if left is not None and right is not None:
                center = ((left + right) / 2.0, cy)
            output, line = lane_mask[offset:].copy(), None
            self.tracking_center = center is not None
        else:
            self.boundaries = None
            self.last_pair_time = float('-inf')
            if lane_mask is not None:
                self.corridor = measure_corridor(
                    lane_mask, center_reference_x, offset, self.min_lane_pixels,
                    band_height=self.center_band_px)

        # Missing paint must not turn IMU mode off and resume ordinary driving.
        # Only observed boundary pairs can confirm the end of the zigzag section.
        if self.zigzag and not detected:
            if self.corridor is None:
                self.exit_since = None
            else:
                if self.exit_since is None:
                    self.exit_since = now
                if now - self.exit_since >= self.exit_delay:
                    self.zigzag = False
                    self.tracking_center = False
        if not self.tracking_center:
            # A missing boundary uses the existing left-lane measurement.
            moments = cv2.moments(mask)
            center = ((int(moments['m10'] / moments['m00']),
                       int(moments['m01'] / moments['m00']))
                      if moments['m00'] else None)
            output, line = mask, None

        self.reference_x = center_reference_x if self.tracking_center else reference_x
        if center is None:
            self.last_x = None
            self.transition = False
        else:
            # Filter the error, since the two modes have different references.
            target = float(center[0] - self.reference_x)
            if self.last_x is not None and (self.zigzag or was_zigzag or self.transition):
                dt = max(0.0, now - self.last_time)
                alpha = (1.0 - np.exp(-dt / self.smoothing_sec)
                         if self.smoothing_sec > 0 else 1.0)
                step = alpha * (target - self.last_x)
                if self.tracking_center and abs(target - self.last_x) <= self.center_deadband_px:
                    step = 0.0
                limit = self.max_center_speed * dt
                filtered = self.last_x + float(np.clip(step, -limit, limit))
                self.transition = not self.zigzag and abs(filtered - target) > 1.0
            else:
                filtered = target
            cx = int(round(filtered + self.reference_x))
            center = (cx, center[1])
            if self.tracking_center:
                cv2.circle(output, center, 4, 255, -1)
            self.last_x = filtered
        self.last_time = now
        return output, center, line


def measure_lane_boundaries(mask, split_x, roi_start=0, min_pixels=20, *,
                            previous=None, band_height=40, margin=60):
    """Measure boundary points near a fixed height without fitting a long line."""
    cy = (mask.shape[0] - roi_start - 1) // 2
    target_y = roi_start + cy
    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    keep = stats[:, cv2.CC_STAT_AREA] >= min_pixels
    keep[0] = False
    clean = keep[labels]
    left_rows, right_rows = [], []
    for y in range(max(roi_start, target_y - band_height // 2),
                   min(mask.shape[0], target_y + band_height // 2 + 1)):
        xs = np.flatnonzero(clean[y])
        if not xs.size:
            continue
        runs = np.split(xs, np.flatnonzero(np.diff(xs) > 1) + 1)
        candidates = np.array([float(np.median(run)) for run in runs])
        if previous is None:
            left = candidates[candidates < split_x]
            right = candidates[candidates >= split_x]
            left = float(left.max()) if left.size else None
            right = float(right.min()) if right.size else None
        else:
            # Associate fragments to past boundaries even across the image center.
            distances = np.abs(candidates[:, None] - np.array(previous))
            sides = np.argmin(distances, axis=1)
            selected = []
            for side in (0, 1):
                matches = np.flatnonzero((sides == side) & (distances[:, side] <= margin))
                index = matches[np.argmin(distances[matches, side])] if matches.size else None
                selected.append(float(candidates[index]) if index is not None else None)
            left, right = selected
        left_rows.append(np.nan if left is None else left)
        right_rows.append(np.nan if right is None else right)

    left_rows, right_rows = np.array(left_rows), np.array(right_rows)
    paired = np.isfinite(left_rows) & np.isfinite(right_rows)
    if np.count_nonzero(paired) >= 4:
        widths = right_rows[paired] - left_rows[paired]
        deviation = np.median(np.abs(widths - np.median(widths)))
        inliers = np.abs(widths - np.median(widths)) <= max(6.0, 3.0 * 1.4826 * deviation)
        if np.count_nonzero(inliers) >= 4:
            midpoints = (left_rows[paired][inliers] + right_rows[paired][inliers]) / 2.0
            midpoint = float(np.median(midpoints))
            half_width = float(np.median(widths[inliers])) / 2.0
            return midpoint - half_width, midpoint + half_width, cy
    left = float(np.nanmedian(left_rows)) if np.count_nonzero(np.isfinite(left_rows)) >= 4 else None
    right = float(np.nanmedian(right_rows)) if np.count_nonzero(np.isfinite(right_rows)) >= 4 else None
    # Do not pair boundaries supported only at different image heights.
    if left is not None and right is not None:
        return None, None, cy
    return left, right, cy


def measure_corridor(mask, split_x, roi_start=0, min_pixels=20, *,
                     band_height=40, boundaries=None):
    """Measure the corridor axis across height and actual near-boundary clearance."""
    if boundaries is None:
        left, right, _ = measure_lane_boundaries(
            mask, split_x, roi_start, min_pixels, band_height=band_height)
    else:
        left, right = boundaries
    if left is None or right is None:
        return None

    # Averages across height reduce the effect of individual zigzag vertices.
    # Every band needs both measured boundaries; inferred widths are not used.
    rows, midpoints = [], []
    for band in np.array_split(np.arange(mask.shape[0]), 5):
        if band.size < 4:
            return None
        start, end = int(band[0]), int(band[-1]) + 1
        far_left, far_right, _ = measure_lane_boundaries(
            mask[start:end], split_x, min_pixels=min_pixels,
            band_height=end - start)
        if far_left is None or far_right is None:
            return None
        rows.append((start + end - 1) / 2.)
        midpoints.append((far_left + far_right) / 2.)
    slope, intercept = np.polyfit(rows, midpoints, 1)
    bend = np.percentile(np.abs(np.array(midpoints) - np.polyval((slope, intercept), rows)), 90)
    return CorridorMeasurement(
        float(np.arctan(slope)), float(split_x - (left + right) / 2.),
        float(split_x - left), float(right - split_x), float(bend))


def fit_straight_lane(mask, roi_start=0):
    binary = mask > 0
    counts = binary.sum(axis=1)
    rows = np.flatnonzero(counts)
    straight_mask = np.zeros_like(mask[roi_start:])
    if not rows.size:
        return straight_mask, None, None

    # Give each image row equal weight, regardless of the marking's thickness.
    centers = (binary[rows] @ np.arange(mask.shape[1])) / counts[rows]
    origin = rows.mean()
    matrix = np.column_stack((rows - origin, np.ones(rows.size)))
    slope, intercept = np.linalg.lstsq(matrix, centers, rcond=None)[0]
    line = tuple((int(round(slope * (y - origin) + intercept)), y - roi_start)
                 for y in (roi_start, mask.shape[0] - 1))
    cv2.line(straight_mask, line[0], line[1], 255, 3)

    # A fixed evaluation height avoids shifting the target with visible vertices.
    cy = (straight_mask.shape[0] - 1) // 2
    cx = int(round(slope * (cy + roi_start - origin) + intercept))
    return straight_mask, (cx, cy), line
