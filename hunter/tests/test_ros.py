"""Hunter ROS commands, expiry and independent CLI/ROS input modes."""
from contextlib import redirect_stderr
import io
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from hunter.control.commands import CommandBuffer, DriveCommand
from hunter.control.controller import Hunter2Controller, load_parameters
from hunter.sim.runtime import arguments, validate_arguments


class HunterRosTests(unittest.TestCase):
    def setUp(self):
        self.parameters = load_parameters(ROOT)
        self.controller = Hunter2Controller(self.parameters)

    def test_forward_reverse_speed_limit_and_zero_steering(self):
        for speed in (.5, -.5, 4., -4.):
            self.controller.reset()
            target = self.controller.update_drive(DriveCommand(speed), 10.)
            expected = max(-1., min(1., speed))
            self.assertAlmostEqual(target.speed, expected)
            self.assertEqual(target.steering_positions, (0., 0.))
            self.assertEqual(target.rear_velocities, (expected / self.parameters['wheel_radius'],) * 2)

    def test_invalid_and_unsupported_commands_brake_and_hold_steering(self):
        for command in (DriveCommand(math.nan), DriveCommand(.5, math.inf),
                        DriveCommand(.5, .2, acceleration=-1.), DriveCommand(.5, .2, jerk=1.)):
            self.controller.reset()
            turning = self.controller.update_drive(DriveCommand(.5, .2), 10.)
            target = self.controller.update_drive(command, .02)
            self.assertAlmostEqual(target.speed, .5 - self.parameters['max_acceleration'] * .02)
            self.assertEqual(target.steering_positions, turning.steering_positions)
            target = self.controller.update_drive(command, 10.)
            self.assertEqual(target.speed, 0.)

    def test_silence_and_invalid_input_trigger_ramped_stop_then_recover(self):
        now = [10.]
        commands = CommandBuffer(.5, clock=lambda: now[0])
        self.assertEqual(commands.current(), DriveCommand())
        commands.update(DriveCommand(.5, .2))
        self.controller.update_drive(commands.current(), 10.)
        now[0] += .49
        self.assertFalse(commands.stale)
        now[0] += .02
        self.assertTrue(commands.stale)
        target = self.controller.stop(.02)
        self.assertAlmostEqual(target.speed, .5 - self.parameters['max_acceleration'] * .02)
        self.assertEqual(self.controller.stop(10.).speed, 0.)
        commands.update(DriveCommand(-.5))
        self.assertLess(self.controller.update_drive(commands.current(), .02).speed, 0.)
        self.assertFalse(commands.update(DriveCommand(math.nan)))
        self.assertEqual(commands.current(), DriveCommand())

    def test_zero_speed_can_steer_without_wheel_propulsion_or_yaw(self):
        for angle in (.3, -.3):
            target = self.controller.update_drive(DriveCommand(0., angle), 10.)
            self.assertAlmostEqual(target.steering_angle, angle)
            self.assertEqual(target.rear_velocities, (0., 0.))
            self.assertEqual(target.yaw_rate, 0.)

    def test_requested_acceleration_and_steering_rates_are_bounded(self):
        target = self.controller.update_drive(DriveCommand(1., .3, .1, .2), .1)
        self.assertAlmostEqual(target.speed, .02)
        self.assertLessEqual(max(map(abs, target.steering_positions)), .01 + 1e-12)
        self.controller.reset()
        target = self.controller.update_drive(DriveCommand(1., .3, 100., 100.), .1)
        self.assertAlmostEqual(target.speed, self.parameters['max_acceleration']*.1)
        self.assertLessEqual(max(map(abs, target.steering_positions)), self.parameters['max_steering_rate']*.1 + 1e-12)

    def test_buffer_clear_and_invalid_limits_require_new_valid_command(self):
        commands = CommandBuffer()
        for command in (DriveCommand(1., .1, steering_angle_velocity=-1.),
                        DriveCommand(1., .1, acceleration=math.inf), DriveCommand(jerk=1.)):
            self.assertFalse(commands.update(command))
            self.assertTrue(commands.stale)
        commands.update(DriveCommand(.5, .2))
        commands.clear()
        self.assertTrue(commands.stale)
        self.assertEqual(commands.received, 1)

    def parse(self, values):
        parser = arguments(ROOT)
        args = parser.parse_args(values)
        validate_arguments(parser, args, self.parameters)
        return args

    def test_ros_waits_without_duration_and_supports_custom_namespace(self):
        args = self.parse(['--ros'])
        self.assertIsNone(args.duration)
        self.assertEqual(args.ackermann_topic, '/hunter2_0/ackermann_cmd')
        args = self.parse(['--ros', '--namespace', '/test/hunter', '--ros-domain-id', '93'])
        self.assertEqual(args.ackermann_topic, '/test/hunter/ackermann_cmd')
        self.assertEqual(args.ros_domain_id, 93)

    def test_cli_speed_and_duration_still_work(self):
        args = self.parse(['--speed', '1.0', '--duration', '5'])
        self.assertFalse(args.ros)
        self.assertGreaterEqual(args.brake_time, 1. / self.parameters['max_acceleration'] + .1)
        self.assertIsNone(self.parse([]).duration)

    def test_conflicting_modes_and_invalid_ros_settings_are_rejected(self):
        for values in (['--ros', '--speed', '.5'], ['--ros', '--duration', '2'],
                       ['--ros', '--brake-time', '1'], ['--ros', '--namespace', 'bad name'],
                       ['--ros', '--ackermann-topic', 'relative'],
                       ['--ros', '--command-timeout', 'nan'], ['--ros', '--command-timeout', '0'],
                       ['--ros', '--ros-domain-id', '233']):
            with self.subTest(values=values), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse(values)


if __name__ == '__main__':
    unittest.main()
