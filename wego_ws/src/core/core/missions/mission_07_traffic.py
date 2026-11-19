from .base import Mission, Result


DEFAULTS = dict(traffic_stopped_speed=0.005)


class TrafficMission(Mission):
    name = 'traffic'

    def enter(self, obs):
        result = super().enter(obs)
        if obs.signs.intersection({0, 1}):
            return self.update(obs)
        return result

    def update(self, obs):
        if self.phase == 'waiting' and obs.signs.intersection({0, 1}):
            self.phase = 'stopped'
            return Result({'mode': 'hold'}, reason='traffic_pixel_threshold_stop')
        if self.phase in ('stopping', 'stopped'):
            # Pixel-qualified red always wins, including simultaneous red/green.
            # Verify measured vehicle speed before releasing the stop.
            if (0 not in obs.signs and 1 in obs.signs
                    and abs(obs.speed) <= self.cfg['traffic_stopped_speed']):
                return Result({'mode': 'follow'}, done=True, reason='green_pixel_threshold_departure')
            return Result({'mode': 'hold'}, reason='waiting_for_green_or_vehicle_stop')
        return Result(reason=self.phase)
