#!/usr/bin/env python3
"""Compare the shared-scene adapter to Isaac Lab's existing Go2 observation/action managers."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / '.runtime/multi_ros/go2_contract.json'
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts')]
from control.workspace import add_ros_packages
add_ros_packages()


def worker():
    from visualization.runtime import prepare_environment, kit_arguments
    prepare_environment(ROOT, 'go2_ros')
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser()
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.kit_args += ' ' + kit_arguments(ROOT, 'go2_ros') + ' --/isaac/startup/ros_bridge_extension='
    app = AppLauncher(args).app
    env = None
    report = {'passed': False}
    try:
        import torch
        from isaaclab.envs import ManagerBasedRLEnv
        from scenarios.registry import load_go2_scenario
        from robot_control.go2.policy import FlatPolicy
        from robot_control.go2.controller import Go2Controller, ArticulationGo2Controller
        cfg = load_go2_scenario('flat')
        cfg.sim.device = args.device
        cfg.commands.base_velocity.debug_vis = False
        cfg.commands.base_velocity.resampling_time_range = (1e9, 1e9)
        cfg.terminations.time_out = None
        env = ManagerBasedRLEnv(cfg)
        env.reset()
        policy = FlatPolicy(ROOT)
        reference, shared = Go2Controller(env, policy), ArticulationGo2Controller(env.scene['robot'], policy)
        max_obs_error = max_target_error = 0.
        commands = ((0., 0., 0.), (.4, 0., 0.), (0., .2, 0.), (0., 0., .5))
        with torch.inference_mode():
            for step in range(80):
                command = commands[(step // 20) % len(commands)]
                expected, actual = reference.observation(command), shared.observation(command)
                max_obs_error = max(max_obs_error, float((expected - actual).abs().max()))
                assert torch.equal(expected, actual), (step, expected, actual)
                actions = reference.actions(expected)
                shared.apply(command)
                assert torch.equal(actions, shared.last_action)
                _, _, terminated, truncated, _ = env.step(actions)
                target = env.action_manager.get_term('joint_pos').processed_actions
                actual_target = shared.robot.data.default_joint_pos + .25 * shared.last_action
                max_target_error = max(max_target_error, float((target - actual_target).abs().max()))
                assert torch.equal(target, actual_target)
                if (terminated | truncated).any():
                    shared.reset()
        report.update(passed=True, steps=80, observation_max_error=max_obs_error,
                      target_max_error=max_target_error, checkpoint_sha256=policy.checkpoint_sha256)
        print('GO2_SHARED_CONTRACT_PASS ' + json.dumps(report), flush=True)
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        import traceback
        traceback.print_exc()
    finally:
        if env is not None:
            env.close()
        REPORT.parent.mkdir(parents=True, exist_ok=True)
        REPORT.write_text(json.dumps(report, indent=2) + '\n')
        app.close()


if __name__ == '__main__':
    if '--worker' in sys.argv:
        sys.argv.remove('--worker')
        worker()
    else:
        REPORT.unlink(missing_ok=True)
        status = subprocess.run([sys.executable, '-u', str(Path(__file__).resolve()), '--worker',
                                 '--headless', *sys.argv[1:]]).returncode
        passed = REPORT.is_file() and json.loads(REPORT.read_text()).get('passed')
        raise SystemExit(0 if status == 0 and passed else 1)
