"""Map training IDs without changing existing ROS messages."""
import math

CLASS_NAMES = ('red_light', 'green_light', 'left_sign', 'right_sign',
               'pedestrian_sign', 'slow_sign', 'parking_sign', 'traffic_cone')


def class_id(name):
    name = str(name).strip().lower().replace(' ', '_').replace('-', '_')
    aliases = {'red': 0, 'green': 1, 'pedestrian': 4, 'slow_down': 5}
    if name in aliases:
        return aliases[name]
    if name in CLASS_NAMES:
        return CLASS_NAMES.index(name)
    if name.isdigit() and 0 <= int(name) < 8:
        return int(name)
    return None


class SignTracker:
    def __init__(self, confidence, areas, confirm_frames=3, timeout=0.5):
        self.confidence, self.areas = confidence, areas
        self.confirm_frames, self.timeout = confirm_frames, timeout
        self.reset()

    def reset(self):
        self.counts = {}
        self.visible = frozenset()
        self.received = -math.inf
        self.large_cone_count = 0

    def update(self, boxes, now, areas=None):
        areas = self.areas if areas is None else areas
        if now-self.received > self.timeout:
            self.counts = {}
        seen = set()
        self.large_cone_count = 0
        for box in boxes:
            identifier = class_id(box.class_name)
            width, height = box.xmax-box.xmin, box.ymax-box.ymin
            if (identifier is not None and math.isfinite(box.probability)
                    and box.probability >= self.confidence and width > 0 and height > 0
                    and width*height >= areas[identifier]):
                seen.add(identifier)
                if identifier == 7 and width*height >= 8000:
                    self.large_cone_count += 1
        self.counts = {key: min(self.confirm_frames, self.counts.get(key, 0)+1) for key in seen}
        # A red observation inhibits departure immediately; green needs confirmation.
        self.visible = frozenset(k for k, count in self.counts.items()
                                 if count >= (1 if k == 0 else self.confirm_frames))
        self.received = now

    def signs(self, now):
        return self.visible if now-self.received <= self.timeout else frozenset()
