"""v1 split-to-single-corridor tracking for intersection exits."""
import time
import cv2
import numpy as np


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
