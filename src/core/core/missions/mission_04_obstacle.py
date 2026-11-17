from .base import Mission, Result


DEFAULTS = dict(obstacle_entry_confirm=0.4, obstacle_min_distance=1.0,
                obstacle_max_distance=8.0, obstacle_exit_confirm=1.0)


class ObstacleMission(Mission):
    name = 'obstacle'
    def update(self, obs):
        if self.phase == 'waiting':
            if self.stable_clear(obs.lane.get('lane_count') == 3, obs.now,
                                 self.cfg['obstacle_entry_confirm']):
                self.phase, self.origin = 'avoiding', obs.travel
                self.clear_since = None
                return Result({'mode': 'obstacles'}, reason='three_lanes_confirmed')
        elif self.phase == 'avoiding':
            if self.distance(obs) > self.cfg['obstacle_max_distance']:
                return Result(fault='obstacle_exit_not_found')
            clear = (obs.lane.get('valid') is True and obs.lane.get('lane_count') == 1
                     and self.distance(obs) >= self.cfg['obstacle_min_distance']
                     and self.front_clear(obs, 0.8))
            if self.stable_clear(clear, obs.now, self.cfg['obstacle_exit_confirm']):
                return Result({'mode': 'follow'}, done=True, reason='single_lane_exit_confirmed')
        return Result(reason=self.phase)
