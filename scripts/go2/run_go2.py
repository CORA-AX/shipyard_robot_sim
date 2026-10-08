#!/usr/bin/env python3
"""Run project-owned Go2 scenarios with Isaac Lab as the simulation framework."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import traceback

ROOT = Path(__file__).resolve().parents[2]
EXPECTED_ENV = ROOT / '.conda/isaacsim-5.0'
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT))
from visualization.runtime import FramePacer, kit_arguments, prepare_environment
from visualization.display import add_display_arguments, dispatch_display
from assets.go2.scenarios import SCENARIOS, load_scenario


def main():
    if Path(sys.prefix).resolve() != EXPECTED_ENV.resolve():
        raise SystemExit(f'Activate {EXPECTED_ENV} before running this script.')
    dispatch_display(Path(__file__), root=ROOT, robot='go2')
    prepare_environment(ROOT, 'go2')

    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    add_display_arguments(parser, default=None)
    parser.add_argument('--mode', choices=('view', 'policy', 'check'), default='check')
    parser.add_argument('--scenario', choices=SCENARIOS, default='flat')
    parser.add_argument('--steps', type=int, help='Control steps; check defaults to 500, other modes run until stopped.')
    parser.add_argument('--vx', type=float, help='Override scenario forward command; check defaults to 0.4 m/s.')
    parser.add_argument('--real-time', action='store_true', help='Pace steps for viewing the motion check.')
    parser.add_argument('--view', action='store_true', help='Show the Go2 scene continuously with zero actions.')
    parser.add_argument('--report', type=Path)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.view:
        args.mode = 'view'
    if args.mode == 'view':
        args.real_time = True
    args.steps = args.steps if args.steps is not None else (500 if args.mode == 'check' else 0)
    if (args.steps < 0 or (args.mode == 'check' and args.steps < 100)
            or (args.vx is not None and not 0.2 <= args.vx <= 0.8)):
        parser.error('Check requires at least 100 steps; use a forward speed between 0.2 and 0.8 m/s.')
    if args.report is None:
        args.report = (ROOT / 'docs/go2_install_evidence/runtime_check.json' if args.mode == 'check'
                       else ROOT / f'.runtime/go2/{args.scenario}_{args.mode}_report.json')
    manifest = json.loads((ROOT / 'assets/go2/config/sources.lock.json').read_text())
    source = ROOT / manifest['isaaclab']['local_path']
    commit = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != manifest['isaaclab']['commit']:
        raise SystemExit('Isaac Lab checkout does not match the pinned commit.')
    subprocess.run(['git', '-C', str(source), 'diff', '--quiet', 'HEAD', '--', 'source', 'apps'], check=True)
    import isaaclab
    if Path(isaaclab.__file__).resolve() != source / 'source/isaaclab/isaaclab/__init__.py':
        raise SystemExit('Isaac Lab is imported from a different checkout.')
    checkpoint = ROOT / manifest['policy']['path']
    if args.mode != 'view' and not checkpoint.is_file():
        raise SystemExit(f'Missing {checkpoint}; follow docs/go2_setup.md.')
    if args.mode != 'view' and hashlib.sha256(checkpoint.read_bytes()).hexdigest() != manifest['policy']['sha256']:
        raise SystemExit('Go2 checkpoint checksum mismatch.')
    args.kit_args = f'{args.kit_args} {kit_arguments(ROOT, "go2")}'
    pacer = FramePacer(args.real_time)
    launcher = AppLauncher(args)
    app = launcher.app
    env = None
    report = {'passed': False, 'scenario': args.scenario, 'mode': args.mode,
              'environment': str(Path(sys.prefix).resolve()), 'python_executable': sys.executable,
              'isaaclab_commit': manifest['isaaclab']['commit'], 'checkpoint_sha256': manifest['policy']['sha256']}
    try:
        import torch
        from importlib.metadata import version
        from rsl_rl.runners import OnPolicyRunner
        from isaaclab.envs import ManagerBasedRLEnv
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx
        from assets.go2.config.policy_cfg import Go2PolicyCfg

        if not torch.cuda.is_available():
            raise RuntimeError('CUDA GPU access is required.')
        report['versions'] = {name: version(name) for name in ('isaacsim', 'isaaclab', 'torch', 'rsl-rl-lib')}
        if report['versions']['isaacsim'] != '5.0.0.0' or report['versions']['rsl-rl-lib'] != '2.3.3':
            raise RuntimeError('This runner requires Isaac Sim 5.0.0.0 and RSL-RL 2.3.3.')
        report['gpu'] = torch.cuda.get_device_name(torch.device(args.device))
        cfg = load_scenario(args.scenario)
        cfg.sim.device = args.device
        report['scenario_config'] = f'{type(cfg).__module__}.{type(cfg).__name__}'
        if args.steps:
            cfg.episode_length_s = args.steps * cfg.sim.dt * cfg.decimation + 10.0
        # A fixed forward command makes a real motion check possible.
        cmd = cfg.commands.base_velocity
        if args.mode == 'check':
            args.vx = args.vx if args.vx is not None else 0.4
            cmd.heading_command = False
            cmd.rel_standing_envs = 0.0
            cmd.ranges.lin_vel_y = (0.0, 0.0)
            cmd.ranges.ang_vel_z = (0.0, 0.0)
        if args.vx is not None:
            cmd.ranges.lin_vel_x = (args.vx, args.vx)
        report['command_ranges'] = {
            'lin_vel_x': cmd.ranges.lin_vel_x, 'lin_vel_y': cmd.ranges.lin_vel_y,
            'ang_vel_z': cmd.ranges.ang_vel_z}
        env = ManagerBasedRLEnv(cfg=cfg)
        report['scene_entities'] = list(env.scene.keys())
        if args.scenario == 'shipyard':
            from pxr import Usd, UsdPhysics
            stage = env.sim.stage
            report['cargo_colliders'] = {
                name: any(child.HasAPI(UsdPhysics.CollisionAPI) for child in
                          Usd.PrimRange(stage.GetPrimAtPath(f'/World/envs/env_0/{prim}')))
                for name, prim in (('cargo_left', 'CargoLeft'), ('cargo_right', 'CargoRight'), ('cargo_rear', 'CargoRear'))}
            if not all(report['cargo_colliders'].values()):
                raise RuntimeError('Shipyard cargo collision geometry was not created.')
        if args.mode == 'view':
            env.reset()
            print('SHIPYARD_GO2_VIEW_READY: Go2 scene running without a policy.', flush=True)
            with torch.inference_mode():
                completed = 0
                while app.is_running() and (not args.steps or completed < args.steps):
                    pacer.start()
                    env.step(torch.zeros(env.action_space.shape, device=args.device))
                    completed += 1
                    pacer.wait(env.unwrapped.step_dt)
            report.update(steps=completed, passed=True)
            return
        agent_cfg = Go2PolicyCfg()
        env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=args.device)
        runner.load(str(checkpoint), load_optimizer=False)
        policy = runner.get_inference_policy(device=args.device)
        obs, _ = env.get_observations()
        report.update(observation_size=obs.shape[-1], action_size=env.num_actions)
        if obs.shape[-1] != 48 or env.num_actions != 12:
            raise RuntimeError('The flat checkpoint requires 48 observations and 12 actions.')
        if args.mode == 'policy':
            export_dir = checkpoint.parent / 'exported'
            export_policy_as_jit(runner.alg.policy, runner.obs_normalizer, path=str(export_dir), filename='policy.pt')
            export_policy_as_onnx(runner.alg.policy, normalizer=runner.obs_normalizer,
                                  path=str(export_dir), filename='policy.onnx')
            print(f'SHIPYARD_GO2_POLICY_READY: {args.scenario}; exported to {export_dir}', flush=True)
        robot = env.unwrapped.scene['robot']
        start = robot.data.root_pos_w[0].clone()
        min_height = float(start[2])
        reset_count = 0
        completed = 0
        speed_sum = 0.0
        samples = deque(maxlen=20 if args.mode == 'policy' else None)
        with torch.inference_mode():
            while not args.steps or completed < args.steps:
                pacer.start()
                if not app.is_running():
                    if args.mode == 'check' or args.steps:
                        raise RuntimeError('Simulation closed before the requested steps completed.')
                    break
                actions = policy(obs)
                if not torch.isfinite(actions).all():
                    raise RuntimeError('Policy produced non-finite actions.')
                obs, _, dones, _ = env.step(actions)
                if not torch.isfinite(robot.data.root_state_w).all():
                    raise RuntimeError('Robot state became non-finite.')
                reset_count += int(dones.sum().item())
                min_height = min(min_height, float(robot.data.root_pos_w[0, 2]))
                speed_sum += float(robot.data.root_lin_vel_b[0, 0])
                completed += 1
                if completed % 50 == 0:
                    samples.append({'step': completed, 'position': robot.data.root_pos_w[0].tolist(),
                                    'forward_m_s': float(robot.data.root_lin_vel_b[0, 0])})
                pacer.wait(env.unwrapped.step_dt)
        end = robot.data.root_pos_w[0]
        dx = float(end[0] - start[0])
        report.update(steps=completed, simulated_seconds=completed * env.unwrapped.step_dt,
                      commanded_forward_m_s=args.vx, mean_forward_m_s=speed_sum / max(completed, 1),
                      start_position=start.tolist(), end_position=end.tolist(),
                      forward_displacement_m=dx, minimum_base_height_m=min_height,
                      resets=reset_count, joint_names=robot.joint_names,
                      samples=list(samples),
                      passed=(reset_count == 0 and min_height > 0.15 and dx > 0.1)
                      if args.mode == 'check' else True)
        if not report['passed']:
            raise RuntimeError('Go2 motion check failed; inspect the JSON report.')
        print('SHIPYARD_GO2_PASS ' + json.dumps(report), flush=True)
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        traceback.print_exc()
        raise
    finally:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
        if env is not None:
            env.close()
        app.close()


if __name__ == '__main__':
    main()
