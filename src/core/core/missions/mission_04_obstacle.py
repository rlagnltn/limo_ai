from .base import Mission, Result
import math


DEFAULTS = dict(obstacle_min_distance=1.0,
                obstacle_max_distance=8.0, obstacle_exit_confirm=1.0,
                obstacle_exit_center_tolerance=0.12, obstacle_exit_heading_tolerance=0.26)


class ObstacleMission(Mission):
    name = 'obstacle'
    def enter(self, obs):
        super().enter(obs)
        self.phase = 'avoiding'
        return Result({'mode': 'obstacles'}, reason='obstacle_stage_activated')

    def update(self, obs):
        if self.phase == 'avoiding':
            if self.distance(obs) > self.cfg['obstacle_max_distance']:
                return Result(fault='obstacle_exit_not_found')
            lines = obs.lane.get('ground_lines', [])
            heading = obs.lane.get('ground_heading', 10.0)
            offset = ((lines[0][1]+lines[1][1])/2*math.cos(heading)
                      if len(lines) == 2 else math.inf)
            clear = (obs.lane.get('valid') is True and obs.lane.get('metric_valid') is True
                     and abs(offset) <= self.cfg['obstacle_exit_center_tolerance']
                     and abs(heading) <= self.cfg['obstacle_exit_heading_tolerance']
                     and (2 in obs.signs) != (3 in obs.signs) and obs.lidar_clear
                     and self.distance(obs) >= self.cfg['obstacle_min_distance'])
            if self.stable_clear(clear, obs.now, self.cfg['obstacle_exit_confirm']):
                return Result({'mode': 'follow'}, done=True, reason='direction_sign_and_clear_center_exit')
        return Result(reason=self.phase)
