from .base import Mission, Result


DEFAULTS = dict(branch_cone_count=1)


class IntersectionMission(Mission):
    name = 'intersection'
    def enter(self, obs):
        super().enter(obs)
        return self.update(obs) if obs.signs.intersection({2, 3}) else Result({'mode': 'follow'})

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
        elif (self.phase == 'turning' and obs.mode == 'follow'
              and obs.large_cone_count >= self.cfg['branch_cone_count']):
            return Result(done=True, reason='branch_cones_confirmed')
        return Result(reason=self.phase)
