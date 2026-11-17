from .base import Mission, Result


DEFAULTS = {}


class ObstacleMission(Mission):
    name = 'obstacle'
    def enter(self, obs):
        super().enter(obs)
        self.phase = 'avoiding'
        return Result({'mode': 'obstacles'}, reason='obstacle_stage_entered')

    def update(self, obs):
        if self.phase == 'avoiding':
            directions = obs.signs.intersection({2, 3})
            if len(directions) == 1:
                return Result({'mode': 'hold'}, done=True, reason='direction_sign_obstacle_exit',
                              handoff_signs=frozenset(directions))
            if len(directions) > 1:
                return Result({'mode': 'hold'}, reason='conflicting_direction_signs')
            if obs.mode == 'hold':
                return Result({'mode': 'obstacles'}, reason='waiting_direction_sign')
        return Result(reason=self.phase)
