"""Start/stop the scanner only while the selected course missions request it."""

LIDAR_MISSIONS = frozenset({'obstacle', 'cone'})


def requires_lidar(mission):
    if mission.name == 'obstacle' and mission.phase == 'waiting_for_stop_line':
        return False
    return mission.name in LIDAR_MISSIONS or (mission.name == 'parking' and mission.phase == 'parking')


class ScanLease:
    def __init__(self, timeout=0.5):
        self.timeout = timeout
        self.requested = False
        self.received = -float('inf')
        self.actual = None
        self.pending = None
        self.retry_at = 0.0

    def receive(self, enabled, now):
        self.requested = bool(enabled)
        self.received = now

    def desired(self, now):
        return self.requested and now-self.received <= self.timeout

    def next_command(self, now):
        desired = self.desired(now)
        if self.pending is not None or now < self.retry_at or self.actual is desired:
            return None
        return 'start' if desired else 'stop'

    def begin(self, command):
        self.pending = command == 'start'

    def complete(self):
        self.actual = self.pending
        self.pending = None

    def fail(self, now):
        self.actual = None
        self.pending = None
        self.retry_at = now+1.0
