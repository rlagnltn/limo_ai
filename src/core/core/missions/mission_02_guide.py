from .base import Mission, Result


DEFAULTS = dict(guide_distance=0.8)


class GuideMission(Mission):
    name = 'guide'
    def enter(self, obs):
        super().enter(obs)
        self.phase = 'slowing'
        return Result({'mode': 'slow'})

    def update(self, obs):
        if self.distance(obs) >= self.cfg['guide_distance']:
            return Result({'mode': 'follow'}, done=True, reason='guide_distance_completed')
        return Result(reason='following_guide_slowly')
