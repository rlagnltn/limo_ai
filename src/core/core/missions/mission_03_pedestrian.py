from .base import Mission, Result


DEFAULTS = dict(pedestrian_stop_distance=0.45)


class PedestrianMission(Mission):
    name = 'pedestrian'
    def update(self, obs):
        if self.phase == 'waiting' and 4 in obs.signs:
            self.phase = 'stopping'
            return Result(self.stop_command(obs, 'pedestrian', self.cfg['pedestrian_stop_distance']),
                          reason='pedestrian_sign_4')
        if self.phase == 'stopping' and obs.mode == 'pedestrian' and obs.phase == 'waiting':
            self.phase = 'stopped'
        if self.phase == 'stopped' and obs.mode == 'follow':
            return Result(done=True, reason='pedestrian_stop_and_wait_completed')
        return Result(reason=self.phase)
