from .base import Mission, Result


DEFAULTS = dict(cone_exit_confirm=1.0)


class ConeMission(Mission):
    name = 'cone'
    def enter(self, obs):
        super().enter(obs)
        self.phase = 'manoeuvring'
        self.last_observed = obs.now
        return Result({'mode': 'cones'}, reason='cone_stage_entered')

    def update(self, obs):
        if obs.now-self.last_observed > 0.2:
            self.clear_since = None
        self.last_observed = obs.now
        clear = (obs.mode == 'cones' and obs.phase == 'cone_acquire'
                 and abs(obs.speed) <= 0.005 and obs.lane.get('valid') is True)
        if self.stable_clear(clear, obs.now, self.cfg['cone_exit_confirm']):
            return Result({'mode': 'follow'}, done=True, reason='cone_exit_and_lane_confirmed')
        if obs.mode == 'cones' and obs.phase == 'cone_acquire':
            self.phase = 'acquiring_lane'
        return Result(reason=self.phase)
