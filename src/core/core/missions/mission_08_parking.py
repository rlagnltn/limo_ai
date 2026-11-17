from .base import Mission, Result
from ..parking_path import t_parking_path


DEFAULTS = dict(parking_side='right', parking_approach=0.6, parking_radius=0.6,
                parking_reverse=0.3, parking_sample=0.08,
                parking_box=[-0.35, 0.25, -1.35, -0.45])


class ParkingMission(Mission):
    name = 'parking'
    def update(self, obs):
        if self.phase == 'waiting' and 6 in obs.signs:
            try:
                path = t_parking_path(self.cfg['parking_side'], self.cfg['parking_approach'],
                                      self.cfg['parking_radius'], self.cfg['parking_reverse'],
                                      self.cfg['parking_sample'], self.cfg['parking_speed'],
                                      self.cfg['rear_axle_offset'], self.cfg['wheelbase'],
                                      self.cfg['max_steering_angle'])
            except ValueError as error:
                return Result(fault=str(error))
            self.phase = 'parking'
            return Result({'mode': 'parking', 'path': path, 'parking_box': self.cfg['parking_box']},
                          reason='parking_sign_6')
        if self.phase == 'parking' and obs.mode == 'parking' and obs.phase == 'complete':
            return Result({'mode': 'hold'}, done=True, reason='t_parking_completed')
        return Result(reason=self.phase)
