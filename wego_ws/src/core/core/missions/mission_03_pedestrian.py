from .base import Mission, Result


DEFAULTS = {}


class PedestrianMission(Mission):
    name = 'pedestrian'
    RESTART_SECONDS = 3.0

    def enter(self, obs):
        self.restart_at = None
        return super().enter(obs)

    def update(self, obs):
        if self.phase == 'waiting' and 4 in obs.signs:
            self.phase = 'stopping'
            return Result({'mode': 'pedestrian', 'immediate_stop': True},
                          reason='pedestrian_sign_4')
        if self.phase == 'stopping' and obs.mode == 'pedestrian' and obs.phase == 'waiting':
            self.phase = 'stopped'
        if self.phase == 'stopped' and obs.mode == 'follow':
            self.phase = 'restarting'
            self.restart_at = obs.now
        if self.phase == 'restarting':
            if obs.mode != 'follow':
                self.restart_at = None
            elif self.restart_at is None:
                self.restart_at = obs.now
            elif obs.now - self.restart_at >= self.RESTART_SECONDS:
                self.phase = 'complete'
                return Result(done=True, reason='pedestrian_restart_completed')
        return Result(reason=self.phase)
