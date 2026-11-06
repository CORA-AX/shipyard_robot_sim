"""Reference-point kinematics, including turning, 3-D attitude and CoM aliases."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from hunter.ros.odometry import FrameState, robot_frames


def rotate(q, v):
    # Independent rotation-matrix expression for the finite-difference oracle.
    w, x, y, z = q
    return ((1 - 2*y*y - 2*z*z)*v[0] + (2*x*y - 2*w*z)*v[1] + (2*x*z + 2*w*y)*v[2],
            (2*x*y + 2*w*z)*v[0] + (1 - 2*x*x - 2*z*z)*v[1] + (2*y*z - 2*w*x)*v[2],
            (2*x*z - 2*w*y)*v[0] + (2*y*z + 2*w*x)*v[1] + (1 - 2*x*x - 2*y*y)*v[2])


class OdometryTests(unittest.TestCase):
    def assert_vector(self, actual, expected, places=7):
        for value, reference in zip(actual, expected):
            self.assertAlmostEqual(value, reference, places=places)

    def test_straight_motion_retains_speed_at_rear_axle(self):
        for speed in (-.5, 0., .5):
            base = FrameState((1, 2, .35), (1, 0, 0, 0), (speed, 0, 0), (0, 0, 0))
            rear = base.at_offset((-.28, 0, -.195))
            self.assert_vector(rear.position, (.72, 2, .155))
            self.assert_vector(rear.linear_velocity, (speed, 0, 0))

    def test_turning_rear_axle_has_no_kinematic_lateral_velocity(self):
        offset = (-.28009, -.00001, -.19558)
        for speed, yaw_rate in ((.5, .3), (.5, -.3), (-.5, .3), (-.5, -.3)):
            base_velocity = (speed + yaw_rate*offset[1], -yaw_rate*offset[0], 0.)
            base = FrameState((1, 2, .35), (1, 0, 0, 0), base_velocity, (0, 0, yaw_rate))
            rear = base.at_offset(offset)
            self.assert_vector(rear.linear_velocity, (speed, 0, 0))
            self.assert_vector(rear.angular_velocity, (0, 0, yaw_rate))
            self.assertGreater(abs(base.linear_velocity[1]), .08)

    def test_pose_offset_rotates_into_world_axes(self):
        q = (math.sqrt(.5), 0, 0, math.sqrt(.5))
        base = FrameState((3, 4, .35), q, (.5, 0, 0), (0, 0, 0))
        rear = base.at_offset((-.28, 0, -.195))
        self.assert_vector(rear.position, (3, 3.72, .155))
        # Twists stay expressed in the child frame, not the world axes.
        self.assert_vector(rear.linear_velocity, (.5, 0, 0))

    def test_point_velocity_matches_position_finite_difference_in_3d(self):
        q = (.8, .2, -.3, .4)
        base = FrameState((1, 2, 3), q, (.5, -.1, .2), (.3, -.2, .4))
        offset = (-.28, .01, -.195)
        rear = base.at_offset(offset)
        dt = 1e-6
        angle = math.sqrt(sum(w*w for w in base.angular_velocity))*dt
        factor = math.sin(angle/2) / (angle/dt)
        dq = (math.cos(angle/2), *(w*factor for w in base.angular_velocity))
        w, x, y, z = base.quaternion
        a, b, c, d = dq
        next_q = (w*a-x*b-y*c-z*d, w*b+x*a+y*d-z*c,
                  w*c-x*d+y*a+z*b, w*d+x*c-y*b+z*a)
        v_world = rotate(base.quaternion, base.linear_velocity)
        next_offset = rotate(next_q, offset)
        next_position = tuple(p + v*dt + r for p, v, r in zip(base.position, v_world, next_offset))
        measured = tuple((b-a)/dt for a, b in zip(rear.position, next_position))
        self.assert_vector(measured, rotate(base.quaternion, rear.linear_velocity), places=6)

    def test_explicit_link_fields_ignore_legacy_com_velocity_alias(self):
        class Row(list):
            def tolist(self):
                return list(self)
        data = SimpleNamespace(**{name: [Row(value)] for name, value in {
            'root_link_pos_w': (1, 2, .35), 'root_link_quat_w': (1, 0, 0, 0),
            'root_link_lin_vel_b': (.5, .084, 0), 'root_link_ang_vel_b': (0, 0, .3),
            'root_lin_vel_b': (999, 999, 999), 'root_pos_w': (999, 999, 999),
        }.items()})
        base, rear = robot_frames(data, (-.28, 0, -.195))
        self.assert_vector(base.position, (1, 2, .35))
        self.assert_vector(rear.linear_velocity, (.5, 0, 0))

    def test_invalid_state_and_offset_are_rejected(self):
        for q in ((0, 0, 0, 0), (math.nan, 0, 0, 0)):
            with self.assertRaises(ValueError):
                FrameState((0, 0, 0), q, (0, 0, 0), (0, 0, 0))
        base = FrameState((0, 0, 0), (1, 0, 0, 0), (0, 0, 0), (0, 0, 0))
        for offset in ((0, 0), (0, 0, math.inf)):
            with self.assertRaises(ValueError):
                base.at_offset(offset)


if __name__ == '__main__':
    unittest.main()
