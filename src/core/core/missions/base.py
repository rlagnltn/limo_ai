from dataclasses import dataclass, field
import math


@dataclass
class Observation:
    now: float
    travel: float = 0.0
    signs: frozenset = frozenset()
    lane: dict = field(default_factory=dict)
    points: tuple = ()
    mode: str = 'follow'
    phase: str = 'running'
    speed: float = 0.0
    reason: str = ''
    lidar_clear: bool = False


@dataclass
class Result:
    command: dict = None
    done: bool = False
    reason: str = ''
    fault: str = ''


class Mission:
    name = ''
    def __init__(self, config):
        self.cfg = config
        self.phase = 'waiting'
        self.origin = 0.0
        self.started = 0.0
        self.clear_since = None

    def enter(self, obs):
        self.origin, self.started = obs.travel, obs.now
        return Result({'mode': 'follow'})

    def distance(self, obs):
        return max(0.0, obs.travel-self.origin)

    def stable_clear(self, clear, now, seconds):
        if not clear:
            self.clear_since = None
            return False
        if self.clear_since is None:
            self.clear_since = now
        return now-self.clear_since >= seconds

    def stop_command(self, obs, mode, fallback):
        if obs.lane.get('metric_valid') and obs.lane.get('stop_distance') is not None:
            return {'mode': mode}
        # Prototype fallback: measured travel from the confirmed sign, not image depth.
        return {'mode': mode, 'distance': fallback}


def validate_config(config):
    for key, value in config.items():
        if isinstance(value, float) and (not math.isfinite(value) or value < 0):
            raise ValueError(f'Invalid parameter {key}')
