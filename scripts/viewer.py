#!/usr/bin/env python3
"""View a scene or spawn Go2 for an external controller, locally or via WebRTC."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from scenarios.registry import SCENARIOS, load_scene
from visualization.runtime import FramePacer, kit_arguments, prepare_environment
from visualization.launcher import add_display_arguments, launch_display


def add_robot_arguments(parser):
    parser.add_argument('--robot', choices=('none', 'go2'), default='none')
    parser.add_argument('--robot-id', default='go2')
    parser.add_argument('--control-port', type=int, default=8765)
    parser.add_argument('--control-timeout', type=float, default=2.0,
                        help='Seconds without a valid action before detaching the controller.')


def validate_robot_arguments(parser, args):
    if (not 1 <= args.control_port <= 65535 or args.control_timeout <= 0
            or not args.robot_id or len(args.robot_id) > 64):
        parser.error('Use a valid port, positive control timeout and robot ID of 1 to 64 characters.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=SCENARIOS, default='shipyard')
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--steps', type=int, default=0, help='Scene frames or applied robot control steps; 0 runs until stopped.')
    parser.add_argument('--report', type=Path)
    add_robot_arguments(parser)
    add_display_arguments(parser)
    args = parser.parse_args()
    if Path(sys.prefix).resolve() != (ROOT / '.conda/isaacsim-5.0').resolve():
        parser.error(f'Activate {ROOT / ".conda/isaacsim-5.0"} first.')
    if args.steps < 0:
        parser.error('--steps must be nonnegative.')
    validate_robot_arguments(parser, args)
    command = [str(Path(__file__).resolve()), '--worker',
               '--scenario', args.scenario, '--device', args.device, '--steps', str(args.steps),
               '--robot', args.robot, '--robot-id', args.robot_id,
               '--control-port', str(args.control_port), '--control-timeout', str(args.control_timeout)]
    if args.report:
        command += ['--report', str(args.report.resolve())]
    try:
        status = launch_display(command, root=ROOT, robot='scenario_viewer',
                                webrtc=args.webrtc, server_ip=args.server_ip)
    except ValueError as exc:
        parser.error(str(exc))
    raise SystemExit(status)




def run_viewer():
    if Path(sys.prefix).resolve() != (ROOT / '.conda/isaacsim-5.0').resolve():
        raise SystemExit('Activate the project .conda/isaacsim-5.0 environment first.')
    prepare_environment(ROOT, 'scenario_viewer')
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=SCENARIOS, default='shipyard')
    parser.add_argument('--steps', type=int, default=0)
    parser.add_argument('--report', type=Path)
    add_robot_arguments(parser)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.steps < 0:
        parser.error('--steps must be nonnegative.')
    validate_robot_arguments(parser, args)
    args.kit_args += ' ' + kit_arguments(ROOT, 'scenario_viewer')
    device = args.device
    headless = bool(args.headless or args.livestream > 0)
    app = AppLauncher(args).app
    args.device = device
    report = {'passed': False, 'scenario': args.scenario, 'frames': 0,
              'headless': headless}
    try:
        if args.robot == 'go2':
            from control.go2.simulation import run_controlled
            run_controlled(app, args, report)
            if not report['passed']:
                raise RuntimeError('Viewer closed before the requested control steps completed.')
            return
        import isaaclab.sim as sim
        import omni.usd
        import omni.timeline
        import torch
        from isaaclab.assets import ArticulationCfg, AssetBaseCfg
        from isaacsim.core.utils.viewports import set_camera_view
        from pxr import UsdGeom

        if device.startswith('cuda'):
            torch.cuda.set_device(device)
        cfg, eye, target = load_scene(args.scenario)
        stage = omni.usd.get_context().get_stage()
        omni.timeline.get_timeline_interface().set_auto_update(False)
        UsdGeom.Xform.Define(stage, '/World/envs/env_0')
        entities = []
        terrain = getattr(cfg, 'terrain', None)
        if terrain is not None:
            if terrain.terrain_type != 'plane':
                raise ValueError('Static viewer currently supports plane terrain; use a background USD asset for other terrain.')
            ground = sim.CuboidCfg(
                size=(100.0, 100.0, 0.02), visual_material=terrain.visual_material)
            ground.func(terrain.prim_path, ground, translation=(0.0, 0.0, -0.01))
            entities.append('terrain')
        for name, asset in vars(cfg).items():
            if isinstance(asset, ArticulationCfg):
                raise RuntimeError('The scenario viewer must not load robot articulations.')
            if not isinstance(asset, AssetBaseCfg):
                continue
            if asset.spawn is None:
                raise ValueError(f'{name}: static scene assets require a spawn configuration.')
            prim_path = asset.prim_path.replace('{ENV_REGEX_NS}', '/World/envs/env_0').replace('{ENV_NS}', '/World/envs/env_0')
            asset.spawn.func(prim_path, asset.spawn, translation=asset.init_state.pos,
                             orientation=asset.init_state.rot)
            entities.append(name)
        set_camera_view(eye=list(eye), target=list(target))
        report['scene_entities'] = entities
        print(f'SCENARIO_VIEW_READY: {args.scenario}; no robots loaded.', flush=True)
        pacer = FramePacer(True)
        while app.is_running() and (not args.steps or report['frames'] < args.steps):
            pacer.start()
            app.update()
            if device.startswith('cuda'):
                torch.cuda.set_device(device)
            report['frames'] += 1
            pacer.wait(0.02)
        report['passed'] = not args.steps or report['frames'] == args.steps
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        import traceback
        traceback.print_exc()
    finally:
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        app.close()
    if not report['passed']:
        raise SystemExit('Viewer closed before completing the requested frames.')


if __name__ == '__main__':
    if '--worker' in sys.argv:
        sys.argv.remove('--worker')
        run_viewer()
    else:
        main()
