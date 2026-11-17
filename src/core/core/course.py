"""Order and lifecycle. No mission can start another mission or write motors."""
from .missions.base import Result, validate_config
from .missions import (mission_01_slow, mission_02_guide, mission_03_pedestrian,
                       mission_04_obstacle, mission_05_intersection, mission_06_cone,
                       mission_07_traffic, mission_08_parking)

MODULES = (mission_01_slow, mission_02_guide, mission_03_pedestrian, mission_04_obstacle,
           mission_05_intersection, mission_06_cone, mission_07_traffic, mission_08_parking)
CLASSES = (mission_01_slow.SlowMission, mission_02_guide.GuideMission,
           mission_03_pedestrian.PedestrianMission, mission_04_obstacle.ObstacleMission,
           mission_05_intersection.IntersectionMission, mission_06_cone.ConeMission,
           mission_07_traffic.TrafficMission, mission_08_parking.ParkingMission)
DEFAULTS = {key: value for module in MODULES for key, value in module.DEFAULTS.items()}
TITLES = ('방지턱 서행', '서행 유도선', '보행자 정지', '3차선 장애물 회피',
          '갈림길', '라바콘 회피', '횡단보도/신호등', 'T자 주차')


class Course:
    def __init__(self, config):
        validate_config(config)
        self.missions = [cls(config) for cls in CLASSES]
        self.index = 0
        self.entered = False
        self.finished = False
        self.fault = ''
        self.epoch = 0
        self.end_index = config.get('end_stage', 8)-1
        self.events = []
        self.results = [dict(stage=i+1, mission=m.name, title=TITLES[i],
                             state='pending', reason='', distance_m=0.0, elapsed_sec=0.0)
                        for i, m in enumerate(self.missions)]

    def record(self, state, reason, obs):
        record = dict(stage=self.index+1, mission=self.active.name,
                      title=TITLES[self.index], state=state, reason=reason,
                      phase=self.active.phase,
                      distance_m=round(self.active.distance(obs), 3) if self.entered else 0.0,
                      elapsed_sec=round(max(0.0, obs.now-self.active.started), 2) if self.entered else 0.0)
        self.results[self.index] = record
        self.events.append(dict(record, sequence=len(self.events)+1))

    def summary(self):
        return [dict(item) for item in self.results]

    def progress(self, obs):
        targets = dict(slow='slow_distance', guide='guide_distance',
                       obstacle='obstacle_min_distance', cone='cone_min_distance')
        key = targets.get(self.active.name)
        return dict(distance_m=round(self.active.distance(obs), 3) if self.entered else 0.0,
                    target_distance_m=self.active.cfg.get(key) if key else None,
                    elapsed_sec=round(max(0.0, obs.now-self.active.started), 2) if self.entered else 0.0)

    @property
    def active(self):
        return self.missions[self.index]

    def tick(self, obs):
        if self.fault or self.finished:
            return Result({'mode': 'hold'}, reason=self.fault or 'course_complete')
        if not self.entered:
            self.entered = True
            result = self.active.enter(obs)
            self.record('waiting' if self.active.phase == 'waiting' else 'active',
                        result.reason or self.active.phase, obs)
            return result
        phase = self.active.phase
        result = self.active.update(obs)
        if result.fault:
            self.fault = result.fault
            self.record('failed', result.fault, obs)
            return Result({'mode': 'hold'}, fault=result.fault)
        if result.done:
            self.record('completed', result.reason, obs)
            if self.index == self.end_index:
                self.finished = True
                return Result({'mode': 'hold'}, done=True, reason='course_complete')
            self.index += 1
            self.epoch += 1
            self.entered = False
        elif self.active.phase != phase:
            self.record('active', result.reason or self.active.phase, obs)
        return result
