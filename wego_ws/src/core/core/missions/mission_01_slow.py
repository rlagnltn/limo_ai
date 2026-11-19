from .base import Mission, Result


DEFAULTS = dict(slow_duration=7.0)


class SlowMission(Mission):
    name = 'slow'
    def update(self, obs):
        if self.phase == 'waiting':
            if obs.stop_line_seen_at is None or obs.stop_line_seen_at <= self.started:
                return Result(reason='waiting_for_stop_line')
            self.phase, self.origin, self.started = 'slowing', obs.travel, obs.now
            return Result({'mode': 'slow'}, reason='slow_stop_line_detected')
        if self.phase == 'slowing' and obs.now-self.started >= self.cfg['slow_duration']:
            return Result({'mode': 'follow'}, done=True, reason='slow_duration_completed')
        return Result(reason=self.phase)
