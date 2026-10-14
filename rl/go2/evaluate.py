#!/usr/bin/env python3
"""Evaluate one fixed Go2 command; save tracking metrics and foot contact traces."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from isaaclab.app import AppLauncher

COMMANDS = {
    'forward_02': (0.2, 0., 0.), 'forward_04': (0.4, 0., 0.), 'forward_08': (0.8, 0., 0.),
    'yaw_05': (0., 0., 0.5), 'yaw_neg_05': (0., 0., -0.5), 'forward_turn': (0.4, 0., 0.5),
    'stand': (0., 0., 0.), 'backward_04': (-0.4, 0., 0.), 'lateral_04': (0., 0.4, 0.),
}
parser = argparse.ArgumentParser(description=__doc__)
checkpoint_arg = parser.add_argument('--checkpoint', type=Path)
output_arg = parser.add_argument('--output', type=Path, help='JSON path; NPZ and contact PNG share its stem.')
case_arg = parser.add_argument('--case', choices=COMMANDS)
parser.add_argument('--num_envs', type=int, default=16)
parser.add_argument('--steps', type=int, default=1600)
parser.add_argument('--warmup-steps', type=int, default=100)
parser.add_argument('--seed', type=int, default=42)
# AppLauncher temporarily disables help and parses known arguments. Defer the
# required flags until after registration so --help works without a checkpoint.
AppLauncher.add_app_launcher_args(parser)
for argument in (checkpoint_arg, output_arg, case_arg):
    argument.required = True
args = parser.parse_args()
if args.num_envs < 1 or not 1 <= args.warmup_steps < args.steps:
    parser.error('Require num_envs >= 1 and 1 <= warmup-steps < steps.')
if args.output.suffix != '.json':
    parser.error('--output must end in .json.')
if not args.checkpoint.is_file():
    parser.error(f'Checkpoint does not exist: {args.checkpoint}')
app = AppLauncher(args).app

import numpy as np
import torch
import isaaclab_tasks  # noqa: F401
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry
from rl.go2.gait_metrics import FOOT_NAMES, summarize_gait
from control.go2.policy import FlatPolicy


def stats(values):
    flat = np.asarray(values).reshape(-1)
    flat = flat[np.isfinite(flat)]
    if not flat.size:
        return None
    return {'mean': float(flat.mean()), 'std': float(flat.std()), 'min': float(flat.min()),
            'max': float(flat.max()), 'p05': float(np.percentile(flat, 5)), 'p95': float(np.percentile(flat, 95))}


env = None
try:
    # Same physical protocol as the previous fixed-command comparisons. Reward
    # definitions do not affect deterministic inference; keep the official task.
    task = 'Isaac-Velocity-Flat-Unitree-Go2-Play-v0'
    cfg = load_cfg_from_registry(task, 'env_cfg_entry_point')
    cfg.scene.num_envs, cfg.seed = args.num_envs, args.seed
    cfg.sim.device = args.device or 'cuda:0'
    cfg.observations.policy.enable_corruption = False
    for event in ('add_base_mass', 'base_com', 'base_external_force_torque', 'push_robot'):
        setattr(cfg.events, event, None)
    cfg.events.reset_base.params['pose_range'] = {key: (0., 0.) for key in ('x', 'y', 'yaw')}
    cfg.events.reset_base.params['velocity_range'] = {
        key: (0., 0.) for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')}
    cfg.events.reset_robot_joints.params['position_range'] = (1., 1.)
    cfg.events.reset_robot_joints.params['velocity_range'] = (0., 0.)
    cfg.terminations.time_out = None
    cfg.episode_length_s = args.steps * cfg.decimation * cfg.sim.dt + 10.
    command = COMMANDS[args.case]
    cfg.commands.base_velocity.heading_command = False
    cfg.commands.base_velocity.rel_heading_envs = 0.
    cfg.commands.base_velocity.rel_standing_envs = 0.
    cfg.commands.base_velocity.ranges.heading = None
    cfg.commands.base_velocity.debug_vis = False
    cfg.commands.base_velocity.resampling_time_range = (1000., 1000.)
    for key, value in zip(('lin_vel_x', 'lin_vel_y', 'ang_vel_z'), command):
        setattr(cfg.commands.base_velocity.ranges, key, (value, value))
    env = ManagerBasedRLEnv(cfg=cfg)
    robot = env.scene['robot']
    sensor = env.scene.sensors['contact_forces']
    sensor_ids, sensor_names = sensor.find_bodies(list(FOOT_NAMES), preserve_order=True)
    robot_ids, robot_names = robot.find_bodies(list(FOOT_NAMES), preserve_order=True)
    if tuple(sensor_names) != FOOT_NAMES or tuple(robot_names) != FOOT_NAMES:
        raise RuntimeError('Evaluation requires all four feet in FL, FR, RL, RR order.')
    policy = FlatPolicy(ROOT, device=env.device, checkpoint=args.checkpoint)
    env.reset(seed=args.seed)
    term = env.command_manager.get_term('base_velocity')
    term.vel_command_b[:] = torch.tensor(command, device=env.device)
    obs = env.observation_manager.compute()['policy']
    keys = ('height_m', 'vx_m_s', 'vy_m_s', 'wz_rad_s', 'xy_error_m_s', 'yaw_error_rad_s',
            'gravity_z', 'foot_contact', 'foot_force_n', 'foot_height_m', 'foot_speed_m_s',
            'foot_air_time_s', 'foot_contact_time_s')
    records = {key: [] for key in keys}
    counts = torch.zeros(args.num_envs, dtype=torch.long, device=env.device)
    ages = torch.zeros_like(counts)
    with torch.inference_mode():
        for step in range(args.steps):
            if not app.is_running():
                raise RuntimeError(f'Simulator stopped at step {step}.')
            if not torch.allclose(obs[:, 9:12], term.command):
                raise RuntimeError('Policy command observation mismatch.')
            observations, _, terminated, truncated, _ = env.step(policy.actor(obs))
            obs = observations['policy']
            if not torch.isfinite(obs).all() or not torch.isfinite(robot.data.root_state_w).all():
                raise RuntimeError('Non-finite simulation state.')
            counts += terminated.long()
            ages += 1
            ages[terminated | truncated] = 0
            if step >= args.warmup_steps:
                valid = ages >= args.warmup_steps
                if not torch.allclose(term.command, torch.tensor(command, device=env.device).expand_as(term.command)):
                    raise RuntimeError('Fixed evaluation command changed.')
                vel = robot.data.root_lin_vel_b
                yaw = robot.data.root_ang_vel_b[:, 2]
                force = sensor.data.net_forces_w[:, sensor_ids].norm(dim=-1)
                values = {
                    'height_m': robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2],
                    'vx_m_s': vel[:, 0], 'vy_m_s': vel[:, 1], 'wz_rad_s': yaw,
                    'xy_error_m_s': (vel[:, :2] - term.command[:, :2]).norm(dim=-1),
                    'yaw_error_rad_s': (yaw - term.command[:, 2]).abs(),
                    'gravity_z': robot.data.projected_gravity_b[:, 2],
                    'foot_contact': (sensor.data.current_contact_time[:, sensor_ids] > 0).float(),
                    'foot_force_n': force,
                    'foot_height_m': robot.data.body_pos_w[:, robot_ids, 2] - env.scene.env_origins[:, 2:3],
                    'foot_speed_m_s': robot.data.body_lin_vel_w[:, robot_ids, :2].norm(dim=-1),
                    'foot_air_time_s': sensor.data.current_air_time[:, sensor_ids],
                    'foot_contact_time_s': sensor.data.current_contact_time[:, sensor_ids],
                }
                for key, value in values.items():
                    mask = valid if value.ndim == 1 else valid[:, None]
                    records[key].append(torch.where(mask, value, torch.nan).cpu().numpy())
            if (step + 1) % 500 == 0:
                print(f'EVAL_PROGRESS {args.case} {step + 1}/{args.steps}', flush=True)
    arrays = {key: np.asarray(value) for key, value in records.items()}
    gait = summarize_gait(arrays['foot_contact'], arrays['foot_height_m'], arrays['foot_speed_m_s'], env.step_dt)
    result = {
        'complete': True, 'task': task, 'case': args.case, 'command': command, 'seed': args.seed,
        'checkpoint': str(policy.checkpoint_path), 'checkpoint_sha256': policy.checkpoint_sha256,
        'checkpoint_iteration': policy.iteration, 'num_envs': args.num_envs,
        'simulated_seconds_per_env': args.steps * env.step_dt,
        'measurement_seconds': (args.steps - args.warmup_steps) * env.step_dt,
        'policy_dt_s': env.step_dt,
        'valid_samples': int(np.isfinite(arrays['height_m']).sum()),
        'base_contact_resets': int(counts.sum()), 'envs_with_base_contact_reset': int((counts > 0).sum()),
        'protocol': ('Deterministic actor; fixed commands/default pose; no observation noise, mass/COM '
                     'randomization or pushes; official contact material retained. '
                     f'Exclude first {args.warmup_steps * env.step_dt:g}s and that duration after each reset; '
                     'count resets over entire run. Feet sampled at policy frequency; sub-step contacts may be missed. '
                     'Environments share initial conditions and are not independent seeds.'),
        'tracking': {key: stats(value) for key, value in arrays.items() if not key.startswith('foot_')},
        'gait': gait,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output.with_suffix('.npz'), **arrays,
                        time_s=(np.arange(len(arrays['height_m'])) + args.warmup_steps + 1) * env.step_dt,
                        foot_names=np.asarray(FOOT_NAMES))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(12, 2.8))
    ax.imshow(np.ma.masked_invalid(arrays['foot_contact'][:, 0, :].T), aspect='auto',
              interpolation='nearest', cmap='Blues', vmin=0, vmax=1,
              extent=(args.warmup_steps * env.step_dt, args.steps * env.step_dt, 3.5, -0.5))
    ax.set_yticks(range(4), FOOT_NAMES)
    ax.set_xlabel('Simulation time (s); blue = contact, white = swing; gray = excluded')
    ax.set_facecolor('#cccccc')
    ax.set_title(f'{args.case} / iteration {policy.iteration} / environment 0')
    fig.tight_layout()
    fig.savefig(args.output.with_suffix('.png'), dpi=150)
    plt.close(fig)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('EVAL_COMPLETE ' + str(args.output), flush=True)
finally:
    if env is not None:
        env.close()
    app.close()
