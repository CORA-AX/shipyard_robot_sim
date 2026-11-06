"""Hunter ROS commands, expiry and independent CLI/ROS input modes."""
from contextlib import redirect_stderr
import io
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from hunter.control.commands import VelocityCommand
from hunter.control.controller import Hunter2Controller, load_parameters
from hunter.sim.runtime import arguments, validate_arguments


class HunterRosTests(unittest.TestCase):
    def setUp(self):
        self.parameters = load_parameters(ROOT)
        self.controller = Hunter2Controller(self.parameters)

    def test_forward_reverse_speed_limit_and_zero_steering(self):
        for speed in (.5, -.5, 4., -4.):
            self.controller.reset()
            target = self.controller.update_command((speed, 0., 0.), 10.)
            expected = max(-1., min(1., speed))
            self.assertAlmostEqual(target.speed, expected)
            self.assertEqual(target.steering_positions, (0., 0.))
            self.assertEqual(target.rear_velocities, (expected / self.parameters['wheel_radius'],) * 2)

    def test_unsupported_or_invalid_twist_brakes_instead_of_driving_straight(self):
        for command in ((.5, .1, 0.), (0., 0., 1.),
                        (math.nan, 0., 0.), (.5, math.inf, 0.)):
            self.controller.reset()
            self.controller.update_command((.5, 0., 0.), 10.)
            target = self.controller.update_command(command, .02)
            self.assertAlmostEqual(target.speed, .5 - self.parameters['max_acceleration'] * .02)
            target = self.controller.update_command(command, 10.)
            self.assertEqual(target.speed, 0.)

    def test_silence_and_invalid_input_trigger_ramped_stop_then_recover(self):
        now = [10.]
        commands = VelocityCommand(.5, clock=lambda: now[0])
        self.assertEqual(commands.current(), (0., 0., 0.))
        commands.update(.5, 0., 0.)
        self.controller.update_command(commands.current(), 10.)
        now[0] += .49
        self.assertFalse(commands.stale)
        now[0] += .02
        self.assertTrue(commands.stale)
        target = self.controller.update_command(commands.current(), .02)
        self.assertAlmostEqual(target.speed, .5 - self.parameters['max_acceleration'] * .02)
        self.assertEqual(self.controller.update_command(commands.current(), 10.).speed, 0.)
        commands.update(-.5, 0., 0.)
        self.assertLess(self.controller.update_command(commands.current(), .02).speed, 0.)
        self.assertFalse(commands.update(math.nan, 0., 0.))
        self.assertEqual(commands.current(), (0., 0., 0.))

    def parse(self, values):
        parser = arguments(ROOT)
        args = parser.parse_args(values)
        validate_arguments(parser, args, self.parameters)
        return args

    def test_ros_waits_without_duration_and_supports_custom_namespace(self):
        args = self.parse(['--ros'])
        self.assertIsNone(args.duration)
        self.assertEqual(args.cmd_vel_topic, '/hunter2_0/cmd_vel')
        args = self.parse(['--ros', '--namespace', '/test/hunter', '--ros-domain-id', '93'])
        self.assertEqual(args.cmd_vel_topic, '/test/hunter/cmd_vel')
        self.assertEqual(args.ros_domain_id, 93)

    def test_cli_speed_and_duration_still_work(self):
        args = self.parse(['--speed', '1.0', '--duration', '5'])
        self.assertFalse(args.ros)
        self.assertGreaterEqual(args.brake_time, 1. / self.parameters['max_acceleration'] + .1)
        self.assertIsNone(self.parse([]).duration)

    def test_conflicting_modes_and_invalid_ros_settings_are_rejected(self):
        for values in (['--ros', '--speed', '.5'], ['--ros', '--duration', '2'],
                       ['--ros', '--brake-time', '1'], ['--ros', '--namespace', 'bad name'],
                       ['--ros', '--cmd-vel-topic', 'relative'],
                       ['--ros', '--command-timeout', 'nan'], ['--ros', '--command-timeout', '0'],
                       ['--ros', '--ros-domain-id', '233']):
            with self.subTest(values=values), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse(values)


if __name__ == '__main__':
    unittest.main()
