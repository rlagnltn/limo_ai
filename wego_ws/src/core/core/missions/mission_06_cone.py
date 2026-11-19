from .base import Mission, Result


DEFAULTS = dict(cone_exit_confirm=1.0, cone_slow_sign_min_area=2800.0)


class ConeMission(Mission):
    name = 'cone'
    def enter(self, obs):
        super().enter(obs)
        self.phase = 'manoeuvring'
        self.last_observed = obs.now
        return Result({'mode': 'cones'}, reason='cone_stage_entered')

    def update(self, obs):
        if self.phase == 'following':
            if 5 in obs.signs:
                return Result({'mode': 'slow'}, done=True, reason='cone_slow_sign_confirmed')
            return Result(reason='waiting_for_slow_sign')
        if obs.now-self.last_observed > 0.2:
            self.clear_since = None
        self.last_observed = obs.now
        clear = (obs.mode == 'cones' and obs.phase == 'cone_acquire'
                 and abs(obs.speed) <= 0.005)
        if self.stable_clear(clear, obs.now, self.cfg['cone_exit_confirm']):
            self.phase = 'following'
            return Result({'mode': 'follow'}, reason='cone_exit_stop_completed')
        if obs.mode == 'cones' and obs.phase == 'cone_acquire':
            self.phase = 'exit_stop'
        return Result(reason=self.phase)


class ConeSlowMission(Mission):
    name = 'cone_slow'

    def enter(self, obs):
        super().enter(obs)
        self.phase = 'slowing'
        return Result({'mode': 'slow'}, reason='cone_slow_started')

    def update(self, obs):
        lights = obs.signs.intersection({0, 1})
        if lights:
            return Result(done=True, reason='cone_slow_traffic_light_confirmed',
                          handoff_signs=frozenset(lights))
        return Result(reason='waiting_for_traffic_light')
