from .base import Mission, Result


DEFAULTS = dict(branch_exit_confirm=0.5)


class IntersectionMission(Mission):
    name = 'intersection'
    def update(self, obs):
        if self.phase == 'waiting':
            left, right = 2 in obs.signs, 3 in obs.signs
            if left and right:
                return Result({'mode': 'hold'}, reason='conflicting_direction_signs')
            if left or right:
                self.phase = 'turning'
                return Result({'mode': 'branch_a' if left else 'branch_b'},
                              reason='left_sign_2' if left else 'right_sign_3')
            if obs.mode == 'hold':
                return Result({'mode': 'follow'}, reason='waiting_direction_sign')
        elif self.phase == 'turning':
            if self.stable_clear(obs.mode == 'follow' and obs.lane.get('valid') is True,
                                 obs.now, self.cfg['branch_exit_confirm']):
                return Result(done=True, reason='branch_turn_and_lane_reacquired')
        return Result(reason=self.phase)
