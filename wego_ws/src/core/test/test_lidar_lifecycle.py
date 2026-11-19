from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parents[1]))
from core.lidar_lifecycle import LIDAR_MISSIONS, ScanLease, requires_lidar


class ScanLifecycleTests(unittest.TestCase):
    def test_obstacle_and_cone_use_lidar(self):
        self.assertEqual(LIDAR_MISSIONS, {'obstacle', 'cone'})
        for name in ('slow', 'guide', 'pedestrian', 'intersection', 'traffic', 'parking'):
            self.assertNotIn(name, LIDAR_MISSIONS)
        self.assertFalse(requires_lidar(SimpleNamespace(name='parking', phase='waiting')))
        self.assertTrue(requires_lidar(SimpleNamespace(name='parking', phase='parking')))

    def test_startup_stops_motor_and_does_not_repeat_commands(self):
        lease = ScanLease()
        self.assertEqual(lease.next_command(0), 'stop')
        lease.begin('stop')
        self.assertIsNone(lease.next_command(.1))
        lease.complete()
        self.assertIs(lease.actual, False)
        self.assertIsNone(lease.next_command(.2))

    def test_enter_and_leave_lidar_mission(self):
        lease = ScanLease()
        lease.begin('stop')
        lease.complete()
        lease.receive(True, 1)
        self.assertEqual(lease.next_command(1), 'start')
        lease.begin('start')
        lease.complete()
        self.assertIs(lease.actual, True)
        lease.receive(False, 1.2)
        self.assertEqual(lease.next_command(1.2), 'stop')
        lease.begin('stop')
        lease.complete()
        self.assertIs(lease.actual, False)

    def test_core_disappearing_stops_motor(self):
        lease = ScanLease(.5)
        lease.receive(True, 1)
        lease.begin('start')
        lease.complete()
        self.assertIsNone(lease.next_command(1.1))
        self.assertEqual(lease.next_command(1.6), 'stop')

    def test_exit_while_start_is_pending_reconciles_to_stop(self):
        lease = ScanLease()
        lease.receive(True, 1)
        lease.begin('start')
        lease.receive(False, 1.1)
        self.assertIsNone(lease.next_command(1.1))
        lease.complete()
        self.assertEqual(lease.next_command(1.2), 'stop')

    def test_service_failure_never_reports_motor_as_stopped(self):
        lease = ScanLease()
        lease.actual = True
        lease.begin('stop')
        lease.fail(1)
        self.assertIsNone(lease.actual)
        self.assertIsNone(lease.next_command(1.5))
        self.assertEqual(lease.next_command(2), 'stop')


if __name__ == '__main__':
    unittest.main()
