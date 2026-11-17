from .base import Mission, Result


DEFAULTS = dict(slow_distance=0.6, slow_min_seconds=2.0, slow_sign_absence=0.5)


class SlowMission(Mission):
    name = 'slow'
    def update(self, obs):
        if self.phase == 'waiting' and 5 in obs.signs:
            self.phase, self.origin, self.started = 'slowing', obs.travel, obs.now
            return Result({'mode': 'slow'}, reason='slow_sign_5')
        if self.phase == 'slowing':
            clear = self.stable_clear(5 not in obs.signs, obs.now, self.cfg['slow_sign_absence'])
            if (self.distance(obs) >= self.cfg['slow_distance'] and clear
                    and obs.now-self.started >= self.cfg['slow_min_seconds']):
                return Result({'mode': 'follow'}, done=True, reason='bump_passed_by_odometry')
        return Result(reason=self.phase)
