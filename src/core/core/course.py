"""Order and lifecycle. No mission can start another mission or write motors."""
from .missions.base import Result, validate_config
from dataclasses import replace
from .missions import (mission_01_slow, mission_02_guide, mission_03_pedestrian,
                       mission_04_obstacle, mission_05_intersection, mission_06_cone,
                       mission_07_traffic, mission_08_parking)

MODULES = (mission_01_slow, mission_02_guide, mission_03_pedestrian, mission_04_obstacle,
           mission_05_intersection, mission_06_cone, mission_07_traffic, mission_08_parking)
CLASSES = (mission_01_slow.SlowMission, mission_02_guide.GuideMission,
           mission_03_pedestrian.PedestrianMission, mission_04_obstacle.ObstacleMission,
           mission_05_intersection.IntersectionMission, mission_06_cone.ConeMission,
           mission_06_cone.ConeSlowMission, mission_07_traffic.TrafficMission,
           mission_08_parking.ParkingMission)
DEFAULTS = {key: value for module in MODULES for key, value in module.DEFAULTS.items()}
STAGES = (1, 2, 3, 4, 5, 6, 6.5, 7, 8)
TITLES = ('방지턱 서행', '서행 유도선', '보행자 정지', '3차선 장애물 회피',
          '갈림길', '라바콘 회피', '라바콘 후 서행', '신호등', 'T자 주차')


class Course:
    def __init__(self, config):
        validate_config(config)
        self.missions = [cls(config) for cls in CLASSES]
        self.index = 0
        self.entered = False
        self.stop_line_armed_at = None
        self.finished = False
        self.fault = ''
        self.epoch = 0
        self.end_index = STAGES.index(config.get('end_stage', 8))
        self.events = []
        self.handoff_signs = frozenset()
        self.results = [dict(stage=STAGES[i], mission=m.name, title=TITLES[i],
                             state='pending', reason='', distance_m=0.0, elapsed_sec=0.0)
                        for i, m in enumerate(self.missions)]

    def record(self, state, reason, obs):
        record = dict(stage=STAGES[self.index], mission=self.active.name,
                      title=TITLES[self.index], state=state, reason=reason,
                      phase=self.active.phase,
                      distance_m=round(self.active.distance(obs), 3) if self.entered else 0.0,
                      elapsed_sec=round(max(0.0, obs.now-self.active.started), 2) if self.entered else 0.0)
        self.results[self.index] = record
        self.events.append(dict(record, sequence=len(self.events)+1))

    def summary(self):
        return [dict(item) for item in self.results]

    def progress(self, obs):
        targets = dict(guide='guide_distance')
        key = targets.get(self.active.name)
        return dict(distance_m=round(self.active.distance(obs), 3) if self.entered else 0.0,
                    target_distance_m=self.active.cfg.get(key) if key else None,
                    elapsed_sec=round(max(0.0, obs.now-self.active.started), 2) if self.entered else 0.0)

    @property
    def active(self):
        return self.missions[self.index]

    @property
    def waiting_for_stop_line(self):
        return self.stop_line_armed_at is not None or (
            self.entered and self.active.name == 'guide' and self.active.phase == 'waiting')

    def tick(self, obs):
        if self.fault or self.finished:
            return Result({'mode': 'hold'}, reason=self.fault or 'course_complete')
        if not self.entered:
            if self.stop_line_armed_at is not None:
                if (obs.stop_line_seen_at is None or
                        obs.stop_line_seen_at <= self.stop_line_armed_at):
                    return Result(reason='waiting_for_stop_line')
                self.stop_line_armed_at = None
            self.entered = True
            entry_obs = replace(obs, signs=self.handoff_signs) if self.handoff_signs else obs
            self.handoff_signs = frozenset()
            result = self.active.enter(entry_obs)
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
            self.handoff_signs = result.handoff_signs
            self.record('completed', result.reason, obs)
            if self.index == self.end_index:
                self.finished = True
                return Result({'mode': 'hold'}, done=True, reason='course_complete')
            pedestrian_completed = self.active.name == 'pedestrian'
            self.index += 1
            if pedestrian_completed and self.active.name == 'obstacle':
                self.stop_line_armed_at = obs.now
                self.active.phase = 'waiting_for_stop_line'
            self.epoch += 1
            self.entered = False
            if self.stop_line_armed_at is not None:
                self.record('waiting', 'waiting_for_stop_line', obs)
        elif self.active.phase != phase:
            self.record('active', result.reason or self.active.phase, obs)
        return result
