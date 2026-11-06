"""Physical sign conventions and startup/braking limits, without Isaac Sim."""
import copy
import math
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from hunter.control.controller import Hunter2Controller, load_parameters


class HunterControlTests(unittest.TestCase):
    def setUp(self):
        self.parameters = load_parameters(ROOT)
        self.controller = Hunter2Controller(self.parameters)

    def test_saved_usd_signs_forward_reverse_and_zero_steering(self):
        for speed in (.2, -.2, 0.):
            self.controller.reset()
            target = self.controller.update(speed, 2.)
            self.assertEqual(target.steering_positions, (0., 0.))
            for velocity in target.rear_velocities:
                self.assertAlmostEqual(velocity * self.parameters['wheel_radius'], speed)

    def test_acceleration_limit_applies_at_start_braking_and_reversal(self):
        previous = 0.
        for requested in [1.] * 100 + [-1.] * 200 + [0.] * 100:
            target = self.controller.update(requested, .02)
            self.assertLessEqual(abs(target.speed - previous), self.parameters['max_acceleration'] * .02 + 1e-12)
            self.assertLessEqual(abs(target.speed), self.parameters['max_speed'])
            previous = target.speed
        self.assertAlmostEqual(previous, 0.)

    def test_one_metre_per_second_is_reachable_within_joint_speed_limit(self):
        for _ in range(260):
            target = self.controller.update(1., .02)
        self.assertAlmostEqual(target.speed, 1.)
        for velocity in target.rear_velocities:
            self.assertAlmostEqual(velocity, 1. / self.parameters['wheel_radius'])
            self.assertLess(abs(velocity), self.parameters['rear_velocity_limit'])

    def test_invalid_commands_leave_controller_state_unchanged(self):
        self.controller.update(.2, .02)
        before = self.controller.speed
        for speed, dt in [(math.nan, .02), (math.inf, .02), (0., 0.), (0., -.1), (0., math.nan)]:
            with self.assertRaises(ValueError):
                self.controller.update(speed, dt)
            self.assertEqual(self.controller.speed, before)

    def test_configuration_rejects_invalid_radius_and_joint_mapping(self):
        for field, value in [('wheel_radius', 0), ('max_speed', math.inf),
                             ('max_acceleration', True), ('rear_velocity_limit', 0),
                             ('rear_velocity_limit', 7.), ('wheelbase', 0), ('front_track', math.nan),
                             ('max_steering_angle', math.pi / 2), ('max_steering_rate', -1),
                             ('passive_front_wheels', ['same', 'same'])]:
            config = copy.deepcopy(self.parameters)
            config[field] = value
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'hunter.yaml'
                path.write_text(yaml.safe_dump(config))
                with self.assertRaises(ValueError):
                    load_parameters(ROOT, path)

    def test_half_metre_per_second_target_is_reached_in_one_second(self):
        for _ in range(50):
            target = self.controller.update(.5, .02)
        self.assertAlmostEqual(target.speed, .5)

    def test_all_four_wheels_share_one_turning_centre_for_forward_and_reverse(self):
        length = self.parameters['wheelbase']
        half_front = self.parameters['front_track'] / 2.
        rear_track = self.parameters['rear_track']
        radius = self.parameters['wheel_radius']
        for speed, yaw in ((.5, .2), (.5, -.2), (-.5, .2), (-.5, -.2)):
            self.controller.reset()
            target = self.controller.update_command((speed, 0., yaw), 10.)
            self.assertAlmostEqual(target.yaw_rate, yaw)
            rear_left, rear_right = [v * radius for v in target.rear_velocities]
            self.assertAlmostEqual((rear_left + rear_right) / 2., speed)
            self.assertAlmostEqual((rear_right - rear_left) / rear_track, yaw)
            # A correctly steered front wheel has zero lateral velocity at
            # its pivot, using the rigid body's velocity field v + omega x r.
            for y, angle in zip((half_front, -half_front), target.steering_positions):
                vx, vy = speed - yaw * y, yaw * length
                self.assertAlmostEqual(-math.sin(angle) * vx + math.cos(angle) * vy, 0.)

    def test_steering_and_speed_limits_hold_during_alternating_turns_and_reversals(self):
        previous_speed, previous_angles = 0., (0., 0.)
        for command in ([(1., 0., 20.)] * 120 + [(-1., 0., 20.)] * 220
                        + [(1., 0., -20.)] * 220 + [(0., 0., 0.)] * 120):
            target = self.controller.update_command(command, .02)
            self.assertLessEqual(abs(target.speed - previous_speed), self.parameters['max_acceleration'] * .02 + 1e-12)
            self.assertLessEqual(abs(target.steering_angle), self.parameters['max_steering_angle'] + 1e-12)
            for before, angle in zip(previous_angles, target.steering_positions):
                self.assertLessEqual(abs(angle - before), self.parameters['max_steering_rate'] * .02 + 1e-12)
                self.assertLess(abs(angle), .58)
            for velocity in target.rear_velocities:
                self.assertLessEqual(abs(velocity), self.parameters['rear_velocity_limit'])
            previous_speed, previous_angles = target.speed, target.steering_positions
        self.assertEqual(target.speed, 0.)
        self.assertEqual(target.yaw_rate, 0.)

    def test_stop_holds_steering_until_stopped_and_new_straight_command_centres_it(self):
        turning = self.controller.update_command((.5, 0., .2), 10.)
        stopped = self.controller.update_command((0., 0., 0.), 10.)
        self.assertEqual(stopped.steering_positions, turning.steering_positions)
        self.assertEqual(stopped.rear_velocities, (0., 0.))
        straight = self.controller.update_command((.5, 0., 0.), 10.)
        self.assertEqual(straight.steering_positions, (0., 0.))

    def test_joint_signs_apply_to_both_steering_and_rear_drive(self):
        reference = self.controller.update_command((.5, 0., .2), 10.)
        parameters = copy.deepcopy(self.parameters)
        parameters['steering'][0]['sign'] = -1
        parameters['rear_wheels'][1]['sign'] = -1
        target = Hunter2Controller(parameters).update_command((.5, 0., .2), 10.)
        self.assertEqual(target.steering_positions, (-reference.steering_positions[0], reference.steering_positions[1]))
        self.assertEqual(target.rear_velocities, (reference.rear_velocities[0], -reference.rear_velocities[1]))


if __name__ == '__main__':
    unittest.main()
