"""Verify body-axis conventions, invalid input handling and imported wheel order."""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/robot_control'))
from robot_control.lekiwi.controller import LekiwiController, load_parameters


class LekiwiControlTests(unittest.TestCase):
    def setUp(self):
        self.parameters = load_parameters(ROOT)
        self.controller = LekiwiController(self.parameters)

    def test_forward_and_sideways_have_correct_wheel_sign_and_magnitude(self):
        radius = self.parameters['wheel_radius']
        forward = self.controller.wheel_velocities((1., 0., 0.))
        sideways = self.controller.wheel_velocities((0., 1., 0.))
        for actual, expected in zip(forward, (.5, -1., .5)):
            self.assertAlmostEqual(actual * radius, expected, places=6)
        for actual, expected in zip(sideways, (-math.sqrt(3) / 2, 0., math.sqrt(3) / 2)):
            self.assertAlmostEqual(actual * radius, expected, places=6)

    def test_rotation_drives_all_wheels_with_same_sign_and_stop_zeros_all(self):
        self.assertTrue(all(value < 0 for value in self.controller.wheel_velocities((0., 0., 1.))))
        self.assertEqual(self.controller.wheel_velocities((0., 0., 0.)), [0., 0., 0.])
        with self.assertRaises(ValueError):
            self.controller.wheel_velocities((float('nan'), 0., 0.))


if __name__ == '__main__':
    unittest.main()
