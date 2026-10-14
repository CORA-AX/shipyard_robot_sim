"""Command coverage and reward incentives without launching Isaac Sim."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rl.go2.tasks.flat_tracking.mdp import (
    feet_air_time_tracking, sample_command_modes, track_ang_vel_z_error, track_lin_vel_xy_error,
    feet_air_time_excess, feet_slide, trot_gait, trot_timing_score,
)


class TrackingTests(unittest.TestCase):
    def test_disjoint_command_modes_cover_turns_and_preserve_moving_commands(self):
        torch.manual_seed(42)
        original = torch.rand(20000, 3) * 2 - 1
        commands, standing, turning = sample_command_modes(original, 0.02, 0.20, 0.30)
        self.assertFalse((standing & turning).any())
        self.assertTrue(torch.equal(commands[standing], torch.zeros_like(commands[standing])))
        self.assertTrue(torch.equal(commands[turning, :2], torch.zeros_like(commands[turning, :2])))
        self.assertTrue((commands[turning, 2].abs() >= 0.30).all())
        self.assertTrue((commands[turning, 2].abs() <= 1.0).all())
        self.assertTrue((commands[turning, 2] > 0).any() and (commands[turning, 2] < 0).any())
        ordinary = ~(standing | turning)
        self.assertTrue(torch.equal(commands[ordinary], original[ordinary]))
        self.assertAlmostEqual(float(standing.float().mean()), 0.02, delta=0.01)
        self.assertAlmostEqual(float(turning.float().mean()), 0.20, delta=0.02)

    def test_all_standing_and_no_special_modes(self):
        original = torch.tensor([[0.4, -0.2, 0.5], [-0.4, 0.2, -0.5]])
        commands, standing, turning = sample_command_modes(original, 1.0, 0.0, 0.3)
        self.assertEqual(commands.count_nonzero(), 0)
        self.assertTrue(standing.all())
        self.assertFalse(turning.any())
        commands, _, _ = sample_command_modes(original, 0.0, 0.0, 0.3)
        self.assertTrue(torch.equal(commands, original))

    def test_invalid_command_mixture_rejected(self):
        with self.assertRaises(ValueError):
            sample_command_modes(torch.zeros(1, 3), 0.9, 0.2, 0.3)

    def test_low_speed_forward_and_forward_turn_modes_preserve_general_coverage(self):
        torch.manual_seed(42)
        original = torch.rand(30000, 3) * 2 - 1
        commands, standing, turning = sample_command_modes(
            original, 0.02, 0.30, 0.30, forward_fraction=0.20, forward_turn_fraction=0.20)
        forward = ~(standing | turning) & (commands[:, 1] == 0) & (commands[:, 2] == 0)
        forward_turn = ~(standing | turning) & (commands[:, 1] == 0) & (commands[:, 2] != 0)
        general = ~(standing | turning | forward | forward_turn)
        for mask, expected in [(standing, .02), (turning, .30), (forward, .20),
                               (forward_turn, .20), (general, .28)]:
            self.assertAlmostEqual(float(mask.float().mean()), expected, delta=.015)
        self.assertTrue(torch.equal(commands[general], original[general]))
        advancing = forward | forward_turn
        self.assertTrue(((commands[advancing, 0] >= .2) & (commands[advancing, 0] <= .6)).all())
        self.assertTrue((commands[forward_turn, 2].abs() >= .3).all())
        self.assertTrue((commands[forward_turn, 2] > 0).any() and (commands[forward_turn, 2] < 0).any())
        self.assertTrue((commands[general, 0] < 0).any() and (commands[general, 1] < 0).any())

    def test_invalid_extended_modes_rejected(self):
        with self.assertRaises(ValueError):
            sample_command_modes(torch.zeros(1, 3), .02, .30, .30, forward_fraction=.7)
        with self.assertRaises(ValueError):
            sample_command_modes(torch.zeros(1, 3), .02, .30, .30, forward_speed_range=(.6, .2))

    def velocity_env(self, commands, velocities, yaw):
        robot = SimpleNamespace(data=SimpleNamespace(
            root_lin_vel_b=torch.tensor(velocities),
            root_ang_vel_b=torch.tensor([[0., 0., value] for value in yaw])))
        return SimpleNamespace(scene={'robot': robot}, command_manager=SimpleNamespace(
            get_command=lambda name: torch.tensor(commands)))

    def test_speed_error_prefers_matching_over_under_and_over_speed(self):
        env = self.velocity_env([[.4, 0., 0.]] * 3,
                                [[.4, 0., 0.], [.3, 0., 0.], [.5, 0., 0.]], [0.] * 3)
        errors = track_lin_vel_xy_error(env, 'base_velocity', SimpleNamespace(name='robot'))
        self.assertEqual(errors[0], 0)
        torch.testing.assert_close(errors[1], errors[2])
        self.assertGreater(errors[1], errors[0])

    def test_yaw_error_emphasizes_both_pure_turns_and_penalizes_standing_drift(self):
        env = self.velocity_env([[0., 0., .5], [0., 0., -.5], [.4, 0., .5], [0., 0., 0.]],
                                [[0., 0., 0.]] * 4, [0., 0., 0., .1])
        errors = track_ang_vel_z_error(env, 'base_velocity', SimpleNamespace(name='robot'))
        torch.testing.assert_close(errors, torch.tensor([1.5, 1.5, .5, .1]))

    def test_matching_yaw_has_no_error_in_any_mode(self):
        env = self.velocity_env([[0., 0., .5], [0., 0., -.5], [.4, 0., .5], [0., 0., 0.]],
                                [[0., 0., 0.]] * 4, [.5, -.5, .5, 0.])
        errors = track_ang_vel_z_error(env, 'base_velocity', SimpleNamespace(name='robot'))
        self.assertEqual(errors.count_nonzero(), 0)

    def reward(self, commands, air_time, contacts):
        sensor = SimpleNamespace(
            data=SimpleNamespace(last_air_time=torch.tensor(air_time)),
            compute_first_contact=lambda dt: torch.tensor(contacts))
        env = SimpleNamespace(
            step_dt=0.02, scene=SimpleNamespace(sensors={'feet': sensor}),
            command_manager=SimpleNamespace(get_command=lambda name: torch.tensor(commands)))
        return feet_air_time_tracking(env, 'base_velocity', SimpleNamespace(name='feet', body_ids=slice(None)))

    def test_rotation_and_translation_get_touchdown_reward_but_standing_does_not(self):
        rewards = self.reward(
            [[0.0, 0.0, 0.5], [0.0, 0.0, -0.5], [0.4, 0.0, 0.0], [0.0, 0.0, 0.0]],
            [[0.25] * 4] * 4, [[True] * 4] * 4)
        self.assertTrue((rewards[:3] > 0).all())
        self.assertEqual(rewards[0], rewards[1])
        self.assertEqual(rewards[0], rewards[2])
        self.assertEqual(rewards[3], 0)

    def test_no_touchdown_or_contact_chatter_has_no_bonus(self):
        rewards = self.reward([[0.0, 0.0, 0.5]] * 2,
                              [[0.3] * 4, [0.02] * 4],
                              [[False] * 4, [True] * 4])
        self.assertEqual(rewards.count_nonzero(), 0)

    def test_long_flight_bonus_is_bounded_and_only_selected_feet_land(self):
        rewards = self.reward([[0.0, 0.0, 0.5]] * 2,
                              [[0.4] * 4, [100.0] * 4],
                              [[True, False, True, False]] * 2)
        torch.testing.assert_close(rewards, torch.tensor([0.6, 0.6]))

    def test_trot_preferred_to_pace_bound_and_uncoordinated_steps(self):
        air = torch.tensor([[.2, 0., 0., .2], [0., .2, .2, 0.],
                            [.2, 0., .2, 0.], [.2, .2, 0., 0.], [.2, 0., 0., 0.]])
        contact = torch.where(air > 0, 0., .2)
        score = trot_timing_score(air, contact)
        torch.testing.assert_close(score[:2], torch.ones(2))
        self.assertTrue((score[2:] < score[0]).all())
        self.assertTrue(((score >= 0) & (score <= 1)).all())

    def test_all_stance_flight_reset_and_static_diagonal_cannot_earn_full_bonus(self):
        air = torch.tensor([[0.] * 4, [.2] * 4, [0.] * 4, [2., 0., 0., 2.]])
        contact = torch.tensor([[.2] * 4, [0.] * 4, [0.] * 4, [0., 2., 2., 0.]])
        score = trot_timing_score(air, contact)
        self.assertEqual(score[:3].count_nonzero(), 0)
        self.assertLess(score[3], 1e-5)
        short = trot_timing_score(torch.tensor([[.005, 0., 0., .005]]),
                                  torch.tensor([[0., .005, .005, 0.]]))
        self.assertLess(short.item(), .1)

    def test_trot_command_gating_ramp_and_explicit_sensor_order(self):
        # Source body order RR, body, FR, FL, RL deliberately differs from reward order.
        air = torch.tensor([[.2, 0., 0., .2, 0.]] * 5)
        contact = torch.tensor([[0., .2, .2, 0., .2]] * 5)
        sensor = SimpleNamespace(cfg=SimpleNamespace(track_air_time=True), data=SimpleNamespace(
            current_air_time=air, current_contact_time=contact))
        env = SimpleNamespace(common_step_counter=0, scene=SimpleNamespace(sensors={'feet': sensor}),
            command_manager=SimpleNamespace(get_command=lambda name: torch.tensor(
                [[.4, 0., 0.], [-.4, 0., 0.], [0., 0., .5], [0., 0., -.5], [0., 0., 0.]])))
        cfg = SimpleNamespace(name='feet', body_ids=[3, 2, 4, 0])
        def ramp_reward():
            return trot_gait(env, 'base_velocity', cfg, ramp_steps=12000, initial_scale=.2)

        start = ramp_reward()
        torch.testing.assert_close(start, torch.tensor([.2, .2, .2, .2, 0.]))
        env.common_step_counter = 6000
        torch.testing.assert_close(ramp_reward(), start * 3)
        env.common_step_counter = 24000
        torch.testing.assert_close(ramp_reward(), start * 5)
        env.common_step_counter = 0
        torch.testing.assert_close(trot_gait(env, 'base_velocity', cfg, ramp_steps=0), start * 5)
        torch.testing.assert_close(trot_gait(env, 'base_velocity', cfg), start * 5)

    def excess_penalty(self, commands, air_time, body_ids=slice(None), **params):
        sensor = SimpleNamespace(cfg=SimpleNamespace(track_air_time=True),
                                 data=SimpleNamespace(current_air_time=torch.tensor(air_time)))
        env = SimpleNamespace(scene=SimpleNamespace(sensors={'feet': sensor}),
                              command_manager=SimpleNamespace(get_command=lambda name: torch.tensor(commands)))
        return feet_air_time_excess(env, 'base_velocity', SimpleNamespace(name='feet', body_ids=body_ids), **params)

    def test_normal_swings_and_planted_feet_have_no_excess_cost(self):
        cost = self.excess_penalty([[.4, 0., 0.]] * 3,
                                   [[0.] * 4, [.2, 0., 0., .2], [.4, 0., 0., .4]])
        self.assertEqual(cost.count_nonzero(), 0)

    def test_single_lifted_foot_cost_increases_without_waiting_for_touchdown(self):
        cost = self.excess_penalty([[.4, 0., 0.]] * 4,
                                   [[0., 0., .45, 0.], [0., 0., .5, 0.],
                                    [0., 0., .6, 0.], [0., 0., 32., 0.]])
        torch.testing.assert_close(cost, torch.tensor([.25, .5, 1., 1.]))

    def test_each_airborne_foot_contributes_and_both_turn_directions_are_enabled(self):
        cost = self.excess_penalty([[0., 0., .5], [0., 0., -.5], [-.4, 0., 0.], [0., .4, 0.]],
                                   [[.7] * 4, [.7] * 4, [0., 0., .7, 0.], [0., 0., .7, 0.]])
        torch.testing.assert_close(cost, torch.tensor([4., 4., 1., 1.]))

    def test_excess_cost_gates_standing_ignores_other_bodies_and_clears_at_contact(self):
        cost = self.excess_penalty([[0., 0., 0.], [.4, 0., 0.], [.4, 0., 0.]],
                                   [[0., 999., 0., .7, 0.], [0., 999., 0., .7, 0.],
                                    [0., 999., 0., 0., 0.]], body_ids=[0, 2, 3, 4])
        torch.testing.assert_close(cost, torch.tensor([0., 1., 0.]))

    def test_invalid_excess_penalty_parameters_and_selection_are_rejected(self):
        for params in ({'max_air_time': 0}, {'saturation_time': 0}):
            with self.assertRaises(ValueError):
                self.excess_penalty([[.4, 0., 0.]], [[0.] * 4], **params)
        with self.assertRaises(ValueError):
            self.excess_penalty([[.4, 0., 0.]], [[0.] * 3])

    def test_invalid_trot_parameters_rejected(self):
        with self.assertRaises(ValueError):
            trot_timing_score(torch.zeros(2, 3), torch.zeros(2, 3))
        with self.assertRaises(ValueError):
            trot_timing_score(torch.zeros(2, 4), torch.zeros(2, 4), timing_std=0)

    def test_slip_selects_matching_bodies_and_ignores_swing_and_vertical_speed(self):
        class Scene(dict):
            pass
        forces = torch.zeros(1, 3, 5, 3)
        forces[0, 0, 3, 2] = 10.  # FL: recent contact, speed 5
        forces[0, 2, 0, 2] = 10.  # RR: contact, vertical speed only
        forces[0, :, 1, 2] = 100.  # base must not be counted
        velocity = torch.tensor([[[0., 0., 99.], [20., 0., 0.], [3., 4., 0.],
                                 [50., 0., 0.], [0., 0., 0.]]])
        scene = Scene(robot=SimpleNamespace(data=SimpleNamespace(body_lin_vel_w=velocity)))
        scene.sensors = {'feet': SimpleNamespace(data=SimpleNamespace(net_forces_w_history=forces))}
        value = feet_slide(SimpleNamespace(scene=scene),
                           SimpleNamespace(name='feet', body_ids=[3, 2, 4, 0]),
                           SimpleNamespace(name='robot', body_ids=[2, 3, 4, 0]))
        torch.testing.assert_close(value, torch.tensor([5.]))


if __name__ == '__main__':
    unittest.main()
