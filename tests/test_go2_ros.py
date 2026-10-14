"""ROS velocity input validity, range and wall-time expiry without ROS/Isaac Sim."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control.go2.ros_bridge import VelocityCommand


class VelocityTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.command = VelocityCommand(0.5, clock=lambda: self.now)

    def test_starts_standing_and_expires_at_wall_time_deadline(self):
        self.assertEqual(self.command.current(), (0., 0., 0.))
        self.assertTrue(self.command.stale)
        self.command.update(.4, -.3, .5)
        self.now += .49
        self.assertEqual(self.command.current(), (.4, -.3, .5))
        self.now += .01
        self.assertTrue(self.command.stale)
        self.assertEqual(self.command.current(), (0., 0., 0.))

    def test_refresh_and_zero_command_stop_previous_motion(self):
        self.command.update(-.4, .3, -.5)
        self.now += .4
        self.command.update(0., 0., 0.)
        self.now += .4
        self.assertFalse(self.command.stale)
        self.assertEqual(self.command.current(), (0., 0., 0.))
        self.assertEqual(self.command.received, 2)

    def test_nonfinite_command_clears_previous_motion_without_refreshing_it(self):
        for invalid in (float('nan'), float('inf'), -float('inf')):
            for axis in range(3):
                self.command.update(.4, .3, .5)
                values = [.4, .3, .5]
                values[axis] = invalid
                self.assertFalse(self.command.update(*values))
                self.assertTrue(self.command.stale)
                self.assertEqual(self.command.current(), (0., 0., 0.))
        self.assertEqual(self.command.rejected, 9)

    def test_commands_above_one_are_forwarded_without_clipping(self):
        self.command.update(3., -2., 1.5)
        self.assertEqual(self.command.current(), (3., -2., 1.5))
        self.assertEqual(self.command.clamped, 0)

    def test_invalid_timeout_rejected(self):
        for timeout in (0., -1., float('inf'), float('nan')):
            with self.assertRaises(ValueError):
                VelocityCommand(timeout)


if __name__ == '__main__':
    unittest.main()
