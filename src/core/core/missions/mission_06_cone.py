from .base import Mission, Result


DEFAULTS = dict(cone_min_distance=1.2, cone_max_distance=8.0, cone_exit_confirm=1.0)


class ConeMission(Mission):
    name = 'cone'
    def update(self, obs):
        if self.phase == 'waiting' and 7 in obs.signs:
            self.phase, self.origin = 'avoiding', obs.travel
            return Result({'mode': 'cones'}, reason='traffic_cone_7')
        if self.phase == 'avoiding':
            if self.distance(obs) > self.cfg['cone_max_distance']:
                return Result(fault='cone_exit_not_found')
            corridor = any(0.3 < x < 1.5 and 0.08 < abs(y) < 1.0 for x, y in obs.points)
            clear = (self.distance(obs) >= self.cfg['cone_min_distance'] and 7 not in obs.signs
                     and not corridor and obs.lane.get('valid') is True)
            if clear:
                # Reacquire the lane after the cone corridor disappears.
                if self.stable_clear(True, obs.now, self.cfg['cone_exit_confirm']):
                    return Result({'mode': 'follow'}, done=True, reason='cone_exit_and_lane_confirmed')
                return Result({'mode': 'hold'}, reason='confirming_cone_exit')
            self.clear_since = None
            if obs.mode == 'hold':
                return Result({'mode': 'cones'})
        return Result(reason=self.phase)
