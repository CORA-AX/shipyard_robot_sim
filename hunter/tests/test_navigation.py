"""Geometric and closed-loop kinematic tests; never starts Isaac Sim/Kit."""
from contextlib import redirect_stderr
import io
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from hunter.control.controller import Hunter2Controller, load_parameters
from hunter.navigation.custom.follower import Mission, load_navigation
from hunter.navigation.state import Observation
from hunter.navigation.custom.path import plan
from hunter.navigation.state import Pose, Segment, advance, wrap
from hunter.ros.custom_waypoint import arguments
from hunter.ros.odometry import read_odometry


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.robot = load_parameters(ROOT)
        self.config = load_navigation(ROOT, self.robot)

    def test_random_paths_terminate_at_exact_goal_and_respect_radius(self):
        rng = random.Random(218)
        for _ in range(100):
            start = Pose(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi))
            goal = Pose(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi))
            path = plan(start, goal, 1.8)
            final = path.at(path.length)
            self.assertAlmostEqual(final.x, goal.x)
            self.assertAlmostEqual(final.y, goal.y)
            self.assertAlmostEqual(wrap(final.yaw-goal.yaw), 0.)
            self.assertGreaterEqual(path.length+1e-8, math.hypot(goal.x-start.x, goal.y-start.y))
            for segment in path.segments:
                self.assertGreaterEqual(segment.length, 0.)
                self.assertLessEqual(abs(segment.curvature), 1./1.8+1e-12)

    def test_straight_and_identical_goals_need_no_extra_loop(self):
        for distance in (0., .1, 3.):
            path = plan(Pose(0, 0, 0), Pose(distance, 0, 0), 1.8)
            self.assertAlmostEqual(path.length, distance)

    def test_local_projection_does_not_jump_to_end_of_loop(self):
        start = Pose(0, 0, 0)
        path = plan(start, Pose(0, 0, math.pi), 1.8)
        progress, error = path.project(start, 0.)
        self.assertEqual(progress, 0.)
        self.assertEqual(error, 0.)
        self.assertGreater(path.length, 5.)

    def test_nearby_aligned_goal_does_not_loop_for_long_approach(self):
        path = plan(Pose(-.28, -.00001, 0), Pose(0, 0, 0), 1.8, approach=.8)
        self.assertLess(path.length, .3)

    def test_initially_within_tolerance_only_confirms_stop(self):
        start = Observation(Pose(.02, .01, .03), 0, 0, 10.)
        mission = Mission(start, Pose(0, 0, 0), self.robot, self.config)
        self.assertEqual(mission.path.length, 0.)
        self.assertEqual(mission.update(start).speed, 0.)
        mission.update(Observation(start.pose, 0, 0, 10.3))
        self.assertEqual(mission.state, 'succeeded')

    def test_closed_loop_reaches_forward_turning_and_behind_goals(self):
        for goal in (Pose(3, 0, 0), Pose(4, 3, math.pi/2), Pose(4, -3, -math.pi/2),
                     Pose(-2, 0, math.pi), Pose(0, 0, math.pi), Pose(.5, .2, 0)):
            with self.subTest(goal=goal):
                state = Observation(Pose(0, 0, 0), 0., 0., 10.)
                mission = Mission(state, goal, self.robot, self.config)
                controller = Hunter2Controller(self.robot)
                for step in range(6000):
                    command = mission.update(state)
                    target = controller.update_drive(command, .02)
                    self.assertGreaterEqual(command.speed, 0.)
                    self.assertLessEqual(abs(target.steering_angle), self.robot['max_steering_angle']+1e-9)
                    pose = advance(state.pose, Segment(target.speed*.02,
                                   math.tan(target.steering_angle)/self.robot['wheelbase']))
                    state = Observation(pose, target.speed, target.yaw_rate, 10.+(step+1)*.02)
                    if mission.done:
                        break
                self.assertEqual(mission.state, 'succeeded', mission.result())
                result = mission.result()
                self.assertLessEqual(result['position_error_m'], self.config['position_tolerance'])
                self.assertLessEqual(result['yaw_error_rad'], self.config['yaw_tolerance'])
                self.assertLessEqual(abs(mission.previous.speed), self.config['stopped_speed'])
                self.assertGreater(result['time_cost_s'], 0.)

    def test_success_requires_pose_heading_actual_stop_and_hold_time(self):
        mission = Mission(Observation(Pose(0, 0, 0), 0, 0, 10), Pose(1, 0, 0), self.robot, self.config)
        mission.update(Observation(Pose(.95, 0, 0), .1, 0, 11))
        self.assertFalse(mission.done)
        mission.update(Observation(Pose(.95, 0, 0), 0, 0, 11.2))
        self.assertFalse(mission.done)
        # Repeated wall-time callbacks with the same simulation stamp do not add cost.
        for _ in range(20):
            mission.update(Observation(Pose(.95, 0, 0), 0, 0, 11.2))
        self.assertFalse(mission.done)
        mission.update(Observation(Pose(.95, 0, 0), 0, 0, 11.5))
        self.assertEqual(mission.state, 'succeeded')
        self.assertAlmostEqual(mission.result()['time_cost_s'], 1.5)
        wrong = Mission(Observation(Pose(0, 0, 0), 0, 0, 10), Pose(1, 0, 0), self.robot, self.config)
        wrong.update(Observation(Pose(1, 0, .5), 0, 0, 11))
        self.assertEqual(wrong.state, 'failed')
        self.assertIsNone(wrong.result()['time_cost_s'])

    def test_reset_tracking_failure_and_cancel_never_score_as_success(self):
        start = Observation(Pose(0, 0, 0), 0, 0, 10)
        for mode in ('reset', 'tracking', 'cancel'):
            mission = Mission(start, Pose(3, 0, 0), self.robot, self.config)
            mission.update(start)
            if mode == 'reset':
                command = mission.update(Observation(Pose(0, 0, 0), 0, 0, 0))
            elif mode == 'tracking':
                command = mission.update(Observation(Pose(0, 2, 0), .3, 0, 11))
            else:
                command = mission.stop('user_cancelled', cancelled=True)
            self.assertTrue(mission.done)
            self.assertEqual(command.speed, 0.)
            self.assertIsNone(mission.result()['time_cost_s'])

    def test_odometry_requires_rear_axle_frames_and_uses_stamp_and_heading(self):
        msg = SimpleNamespace(header=SimpleNamespace(frame_id='hunter2_0/odom', stamp=SimpleNamespace(sec=12, nanosec=250000000)),
            child_frame_id='hunter2_0/base_link', pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=1., y=2.), orientation=SimpleNamespace(w=math.sqrt(.5), x=0., y=0., z=math.sqrt(.5)))),
            twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=.4), angular=SimpleNamespace(z=.2))))
        state = read_odometry(msg, 'hunter2_0')
        self.assertAlmostEqual(state.pose.yaw, math.pi/2)
        self.assertEqual(state.seconds, 12.25)
        msg.child_frame_id = 'hunter2_0/chassis_link'
        with self.assertRaises(ValueError):
            read_odometry(msg, 'hunter2_0')

    def test_cli_defaults_and_invalid_goal_speed_and_timing(self):
        args, _, config = arguments(ROOT, ['--x', '3', '--y', '0'])
        self.assertIsNone(args.max_sim_time)
        self.assertEqual(config['cruise_speed'], .4)
        for extra in (['--x', 'nan', '--y', '0'], ['--x', '0', '--y', '0', '--speed', '-1'],
                      ['--x', '0', '--y', '0', '--speed', '2'], ['--x', '0', '--y', '0', '--max-sim-time', '0']):
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                arguments(ROOT, extra)


if __name__ == '__main__':
    unittest.main()
