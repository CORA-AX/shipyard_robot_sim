"""Summarize any completed or partial Go2 TensorBoard training run."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch
import yaml
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def analyze(run, output, window=200):
    run, output = Path(run).resolve(), Path(output).resolve()
    agent = yaml.load((run / 'params/agent.yaml').read_text(), Loader=yaml.BaseLoader)
    env = yaml.load((run / 'params/env.yaml').read_text(), Loader=yaml.BaseLoader)
    accumulator = EventAccumulator(str(run), size_guidance={'scalars': 0}).Reload()
    history, scalars = {}, {}
    for tag in accumulator.Tags()['scalars']:
        if tag.endswith('/time'):
            continue
        events = accumulator.Scalars(tag)
        if not events:
            continue
        values = np.asarray([event.value for event in events])
        if not np.isfinite(values).all():
            raise ValueError(f'Non-finite training scalar: {tag}')
        history[tag] = [{'iteration': event.step, 'value': event.value, 'wall_time': event.wall_time}
                        for event in events]
        scalars[tag] = {'count': len(events), 'first_iteration': events[0].step,
                        'last_iteration': events[-1].step, 'minimum': float(values.min()),
                        'maximum': float(values.max()), 'last_value': float(values[-1]),
                        'first_window_mean': float(values[:window].mean()),
                        'last_window_mean': float(values[-window:].mean())}
    if 'Perf/total_fps' not in history:
        raise ValueError('Run has no logged training iterations yet.')
    indices = [value['iteration'] for value in history['Perf/total_fps']]
    checkpoints = [path for path in run.glob('model_*.pt') if path.stem.removeprefix('model_').isdigit()]
    if not checkpoints:
        raise ValueError('Run has no saved checkpoints yet.')
    checkpoint_path = max(checkpoints, key=lambda path: int(path.stem.removeprefix('model_')))
    checkpoint = torch.load(checkpoint_path, map_location='cpu', weights_only=True)
    if not all(torch.isfinite(value).all() for value in checkpoint['model_state_dict'].values()):
        raise ValueError('Checkpoint contains non-finite model parameters.')
    # FPS counts global transitions, while timing is measured on rank 0. Align
    # by logged iteration rather than assuming every scalar starts at zero.
    collection = {v['iteration']: v['value'] for v in history['Perf/collection time']}
    learning = {v['iteration']: v['value'] for v in history['Perf/learning_time']}
    timings = [collection[index] + learning[index] for index in indices]
    local_envs, rollout = int(env['scene']['num_envs']), int(agent['num_steps_per_env'])
    estimated_world = max(1, round(float(np.median([
        entry['value'] * timing / (local_envs * rollout)
        for entry, timing in zip(history['Perf/total_fps'], timings)]))))
    summary = {
        'run': str(run), 'checkpoint': str(checkpoint_path),
        'checkpoint_iteration': checkpoint['iter'],
        'checkpoint_sha256': hashlib.sha256(checkpoint_path.read_bytes()).hexdigest(),
        'checkpoint_tensors_all_finite': True, 'all_scalar_values_finite': True,
        'resume': agent.get('resume') == 'true', 'logged_iterations': len(indices),
        'first_iteration': indices[0], 'last_iteration': indices[-1],
        'iterations_contiguous': indices == list(range(indices[0], indices[-1] + 1)),
        'configured_max_iterations': int(agent['max_iterations']),
        'num_envs_per_rank': local_envs, 'world_size_inferred_from_fps': estimated_world,
        'rollout_steps': rollout,
        'estimated_transitions': len(indices) * local_envs * rollout * estimated_world,
        'training_loop_seconds': float(sum(timings)), 'summary_window_samples': window,
        'note': ('Training reward is not a gait-quality ranking. Command/termination metrics are rank-0 '
                 'training logs, not fixed-command physical evaluations. World size is inferred from FPS. '
                 'A partial run may have a checkpoint older than its final logged iteration.'),
        'scalars': scalars,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / 'training_overview.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    (output / 'scalar_history.json').write_text(json.dumps(history, indent=2, allow_nan=False) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    panels = [('Train/mean_reward', 'Episode reward'), ('Train/mean_episode_length', 'Episode duration (s)'),
              ('Episode_Reward/trot_gait', 'Trot reward'),
              ('Episode_Reward/feet_air_time_excess', 'Long airborne-foot penalty'),
              ('Episode_Reward/feet_slide', 'Foot slip penalty'),
              ('Episode_Reward/base_height', 'Body height reward'),
              ('Metrics/base_velocity/error_vel_xy', 'Logged planar error'),
              ('Metrics/base_velocity/error_vel_yaw', 'Logged yaw error')]
    fig, axes = plt.subplots(4, 2, figsize=(12, 12), constrained_layout=True)
    for ax, (tag, title) in zip(axes.flat, panels):
        ax.set(title=title, xlabel='PPO iteration')
        if tag not in history:
            ax.text(.5, .5, 'Not logged in this run', ha='center', transform=ax.transAxes)
            continue
        x = np.array([entry['iteration'] for entry in history[tag]])
        y = np.array([entry['value'] for entry in history[tag]])
        if tag == 'Train/mean_episode_length':
            y *= float(env['sim']['dt']) * int(env['decimation'])
        width = min(window, len(y))
        ax.plot(x, y, alpha=.25, linewidth=.6)
        ax.plot(x[width - 1:], np.convolve(y, np.ones(width) / width, mode='valid'), linewidth=1.3)
        ax.grid(alpha=.2)
    fig.suptitle(run.name)
    fig.savefig(output / 'training_overview.png', dpi=150)
    plt.close(fig)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='Run directory containing params and event files.')
    parser.add_argument('--output', type=Path, help='Defaults to <run>/analysis; uses new overview filenames.')
    parser.add_argument('--window', type=int, default=200, help='Smoothing/summary window in logged samples.')
    args = parser.parse_args()
    if args.window < 1:
        parser.error('--window must be positive.')
    summary = analyze(args.run, args.output or args.run / 'analysis', args.window)
    print(json.dumps({key: value for key, value in summary.items() if key != 'scalars'}, indent=2))


if __name__ == '__main__':
    main()
