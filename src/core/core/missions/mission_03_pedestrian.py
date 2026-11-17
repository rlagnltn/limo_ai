from .base import Mission, Result


DEFAULTS = dict(pedestrian_stop_distance=0.0, pedestrian_follow_seconds=20.0)


class PedestrianMission(Mission):
    name = 'pedestrian'
    def update(self, obs):
        if self.phase == 'waiting' and 4 in obs.signs:
            self.phase = 'stopping'
            return Result({'mode': 'pedestrian', 'immediate_stop': True},
                          reason='pedestrian_sign_4')
        if self.phase == 'stopping' and obs.mode == 'pedestrian' and obs.phase == 'waiting':
            self.phase = 'stopped'
        if self.phase == 'stopped' and obs.mode == 'follow':
            self.phase = 'following'
            self.follow_started = obs.now
            return Result(reason='pedestrian_follow_delay_started')
        if (self.phase == 'following' and obs.mode == 'follow'
                and obs.now-self.follow_started >= self.cfg['pedestrian_follow_seconds']):
            return Result(done=True, reason='pedestrian_stop_and_wait_completed')
        return Result(reason=self.phase)
