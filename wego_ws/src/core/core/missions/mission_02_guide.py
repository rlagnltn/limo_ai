from .base import Mission, Result


DEFAULTS = dict(guide_distance=1.6)


class GuideMission(Mission):
    name = 'guide'
    def update(self, obs):
        if self.phase == 'waiting':
            if obs.stop_line_seen_at is None or obs.stop_line_seen_at <= self.started:
                return Result(reason='waiting_for_stop_line')
            self.phase, self.origin, self.started = 'slowing', obs.travel, obs.now
            return Result({'mode': 'slow'}, reason='guide_stop_line_detected')
        if self.phase == 'slowing' and self.distance(obs) >= self.cfg['guide_distance']:
            return Result({'mode': 'follow'}, done=True, reason='guide_distance_completed')
        return Result(reason=self.phase)
