from .base import Mission, Result


DEFAULTS = {}


class ParkingMission(Mission):
    name = 'parking'

    def enter(self, obs):
        result = super().enter(obs)
        return self.update(obs) if 6 in obs.signs else result

    def update(self, obs):
        if self.phase == 'waiting' and 6 in obs.signs:
            self.phase = 'parking'
            return Result({'mode': 'hold'}, reason='parking_sign_6')
        if self.phase == 'parking':
            if obs.parking_state == 'FAILED':
                return Result(fault='forward_parking_failed: ' + obs.parking_reason)
            if obs.parking_state == 'DONE':
                return Result({'mode': 'hold'}, done=True, reason='t_parking_completed')
            return Result(reason='forward_parking_' + obs.parking_state.lower())
        return Result(reason=self.phase)
