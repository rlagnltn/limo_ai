from .base import Mission, Result


DEFAULTS = dict(slow_duration=7.0)


class SlowMission(Mission):
    name = 'slow'
    def update(self, obs):
        if self.phase == 'waiting' and 5 in obs.signs:
            self.phase, self.origin, self.started = 'slowing', obs.travel, obs.now
            return Result({'mode': 'slow'}, reason='slow_sign_5')
        if self.phase == 'slowing' and obs.now-self.started >= self.cfg['slow_duration']:
            return Result({'mode': 'follow'}, done=True, reason='slow_duration_completed')
        return Result(reason=self.phase)
