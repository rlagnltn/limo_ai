from .base import Mission, Result


DEFAULTS = dict(guide_distance=1.8)


class GuideMission(Mission):
    name = 'guide'
    def update(self, obs):
        if self.phase == 'waiting' and 5 in obs.signs:
            self.phase, self.origin, self.started = 'slowing', obs.travel, obs.now
            return Result({'mode': 'slow'}, reason='guide_sign_5')
        if self.phase == 'slowing' and self.distance(obs) >= self.cfg['guide_distance']:
            return Result({'mode': 'follow'}, done=True, reason='guide_distance_completed')
        return Result(reason=self.phase)
