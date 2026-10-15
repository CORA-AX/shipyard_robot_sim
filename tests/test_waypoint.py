"""Waypoint convergence, coordinate transforms and command validity without ROS/GPU."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/robot_control'))
from robot_control.waypoint import Parameters, State, WaypointPD, load_settings, rotate, yaw


class WaypointTests(unittest.TestCase):
    def setUp(self):
        self.settings, self.profiles, self.robots = load_settings(
            ROOT / 'ros2_ws/src/robot_control/config/waypoint.yaml')

    def test_both_profiles_reach_a_goal_with_actuator_lag_and_keep_commands_bounded(self):
        for name, profile in self.profiles.items():
            with self.subTest(robot=name):
                controller = WaypointPD(profile)
                x, y, angle, vx, vy, wz = 0., 0., math.pi / 2, 0., 0., 0.
                controller.update(State(0., x, y, angle, vx, vy, wz))
                controller.set_goal(.7, -.3)
                previous_command = (0., 0., 0.)
                for step in range(1, 1501):
                    controller.update(State(step * .02, x, y, angle, vx, vy, wz))
                    command = controller.step()
                    self.assertLessEqual(math.hypot(*command[:2]), profile.max_speed + 1e-9)
                    if controller.status == 'TRACKING':
                        self.assertLessEqual(math.dist(command[:2], previous_command[:2]),
                                             profile.max_acceleration * .02 + 1e-8)
                    previous_command = command
                    c, s = math.cos(angle), math.sin(angle)
                    vx += .12 * (c * command[0] - s * command[1] - vx)
                    vy += .12 * (s * command[0] + c * command[1] - vy)
                    wz += .12 * (command[2] - wz)
                    x, y, angle = x + vx * .02, y + vy * .02, angle + wz * .02
                    if controller.status == 'REACHED':
                        break
                self.assertEqual(controller.status, 'REACHED')
                self.assertLessEqual(math.dist((x, y), (.7, -.3)), profile.position_tolerance)
                self.assertEqual(controller.command, (0., 0., 0.))

    def test_world_x_goal_at_ninety_degree_yaw_commands_negative_body_y(self):
        controller = WaypointPD(self.profiles['go2'])
        controller.update(State(0., 0., 0., math.pi / 2, 0., 0., 0.))
        controller.set_goal(1., 0.)
        controller.update(State(.02, 0., 0., math.pi / 2, 0., 0., 0.))
        vx, vy, _ = controller.step()
        self.assertAlmostEqual(vx, 0.)
        self.assertLess(vy, 0.)

    def test_teleport_and_clock_rewind_cancel_goals_and_duplicate_stamps_do_not_advance_control(self):
        controller = WaypointPD(self.profiles['go2'])
        controller.update(State(1., 0., 0., 0., 0., 0., 0.))
        controller.set_goal(2., 0.)
        controller.update(State(1.02, 0., 0., 0., 0., 0., 0.))
        command = controller.step()
        self.assertFalse(controller.update(State(1.02, 0., 0., 0., 0., 0., 0.)))
        self.assertEqual(controller.step(), command)
        controller.update(State(1.04, 1., 0., 0., 0., 0., 0.))
        self.assertEqual(controller.status, 'POSE_RESET')
        self.assertIsNone(controller.goal)
        controller.set_goal(2., 0.)
        controller.update(State(0., 1., 0., 0., 0., 0., 0.))
        self.assertEqual(controller.status, 'CLOCK_RESET')
        self.assertEqual(controller.step(), (0., 0., 0.))

    def test_yaw_wrap_chooses_short_rotation_and_invalid_values_are_rejected(self):
        controller = WaypointPD(self.profiles['go2'])
        controller.update(State(0., 0., 0., math.pi - .1, 0., 0., 0.))
        controller.set_goal(0., 0., -math.pi + .1)
        controller.update(State(.02, 0., 0., math.pi - .1, 0., 0., 0.))
        self.assertGreater(controller.step()[2], 0.)
        for value in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                controller.set_goal(value, 0.)
            with self.assertRaises(ValueError):
                replace(self.profiles['go2'], max_speed=value)
        rotated = rotate((0., 0., math.sin(math.pi / 4), math.cos(math.pi / 4)), (1., 0., 0.))
        self.assertAlmostEqual(rotated[0], 0.)
        self.assertAlmostEqual(rotated[1], 1.)


if __name__ == '__main__':
    unittest.main()
