#!/usr/bin/env python3
"""Launch robots with isolated Isaac Python and local/WebRTC display."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import sys

def project_root():
    if os.environ.get('WAREHOUSE_SIM_ROOT'):
        return Path(os.environ['WAREHOUSE_SIM_ROOT']).resolve()
    for start in (Path(__file__).resolve(), Path.cwd()):
        for path in (start, *start.parents):
            if (path / 'assets/go2/config/sources.lock.json').is_file():
                return path
    raise RuntimeError('Cannot locate simulation assets; use robot_bringup with project_root.')


ROOT = project_root()
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from control.workspace import add_ros_packages
add_ros_packages()
from robot_sim.environment import ros_environment
from control.go2.keyboard import add_keyboard_arguments, validate_keyboard_arguments
from scenarios.registry import DEFAULT_SCENARIO, GO2_SCENARIOS
from visualization.launcher import add_display_arguments, launch_display, run_process
from visualization.runtime import kit_arguments, prepare_environment


def add_arguments(parser):
    add_keyboard_arguments(parser)
    parser.add_argument('--robot-type', choices=('go2', 'lekiwi', 'multi'), default='go2')
    parser.add_argument('--robots-file', type=Path)
    parser.add_argument('--scenario', choices=GO2_SCENARIOS, default=DEFAULT_SCENARIO)
    parser.add_argument('--robot-id', default='go2')
    parser.add_argument('--namespace', default='unitree_go2')
    parser.add_argument('--initial-position', type=float, nargs=3)
    parser.add_argument('--initial-yaw', type=float, default=0.)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--cmd-vel-topic', default='/unitree_go2/cmd_vel')
    parser.add_argument('--command-timeout', type=float, default=0.5,
                        help='Wall seconds without valid cmd_vel before commanding zero velocity.')
    parser.add_argument('--ros-domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--policy-device', default='cpu')
    parser.add_argument('--checkpoint', type=Path,
                        help='Defaults to the selected control_policy in the asset manifest.')
    parser.add_argument('--steps', type=int, default=0, help='50Hz control steps; 0 runs until stopped.')
    parser.add_argument('--report', type=Path)


def validate(parser, args):
    validate_keyboard_arguments(parser, args)
    if Path(sys.prefix).resolve() != (ROOT / '.conda/isaacsim-5.0').resolve():
        parser.error(f'Activate {ROOT / ".conda/isaacsim-5.0"} first.')
    if (args.steps < 0 or not math.isfinite(args.command_timeout) or args.command_timeout <= 0
            or not 0 <= args.ros_domain_id <= 232):
        parser.error('Use nonnegative steps, a finite positive timeout and ROS domain 0..232.')
    if args.robot_type == 'multi':
        from robot_sim.config import load_world_config
        if not args.robots_file:
            parser.error('Multi-robot runtime requires --robots-file.')
        world = load_world_config(args.robots_file)
        args.scenario, args.seed = world['scenario'], world['seed']
        if args.keyboard:
            parser.error('Use a separate ROS teleop node for each robot.')
        if args.checkpoint and not any(r['robot_type'] == 'go2' for r in world['robots']):
            parser.error('--checkpoint requires at least one Go2.')
    if args.robot_type == 'lekiwi' and args.checkpoint:
        parser.error('--checkpoint is only supported for Go2.')
    if args.robot_type == 'lekiwi' and args.keyboard:
        parser.error('Use ros2 run robot_control teleop --robot-id lekiwi_0 --keyboard.')
    if not args.cmd_vel_topic.startswith('/'):
        parser.error('--cmd-vel-topic must be an absolute ROS topic name.')
    args.namespace = args.namespace.strip('/')
    if (not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', args.robot_id)
            or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', args.namespace)
            or args.seed < 0 or not math.isfinite(args.initial_yaw)
            or (args.initial_position and not all(math.isfinite(v) for v in args.initial_position))):
        parser.error('Use valid robot id/namespace, finite spawn pose and a nonnegative seed.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_arguments(parser)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--headless', action='store_true', help='Run without a local window.')
    add_display_arguments(parser)
    args = parser.parse_args()
    validate(parser, args)
    if args.webrtc and args.headless:
        parser.error('--webrtc already runs headless; use one display option.')
    environment = ros_environment()
    environment['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    # A fresh interpreter loads the ROS shared libraries before Kit starts.
    for key in set(os.environ) - set(environment):
        os.environ.pop(key)
    os.environ.update(environment)
    command = [str(Path(__file__).resolve()), '--worker', '--robot-type', args.robot_type,
               '--scenario', args.scenario,
               '--device', args.device, '--policy-device', args.policy_device,
               '--cmd-vel-topic', args.cmd_vel_topic, '--command-timeout', str(args.command_timeout),
               '--ros-domain-id', str(args.ros_domain_id), '--steps', str(args.steps)]
    command += ['--robot-id', args.robot_id, '--namespace', args.namespace,
                '--seed', str(args.seed), '--initial-yaw', str(args.initial_yaw)]
    if args.initial_position:
        command += ['--initial-position', *map(str, args.initial_position)]
    if args.robots_file:
        command += ['--robots-file', str(args.robots_file.resolve())]
    if args.checkpoint:
        command += ['--checkpoint', str(args.checkpoint.resolve())]
    report_path = (args.report.resolve() if args.report else
                   ROOT / '.runtime' / (args.robot_type + '_ros') / f'run_{os.getpid()}.json')
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.unlink(missing_ok=True)
    command += ['--report', str(report_path)]
    if args.keyboard:
        command += ['--keyboard', '--keyboard-speed', str(args.keyboard_speed),
                    '--keyboard-turn-speed', str(args.keyboard_turn_speed),
                    '--keyboard-timeout', str(args.keyboard_timeout)]
    if args.headless:
        environment = prepare_environment(ROOT, args.robot_type + '_ros', environment)
        status = run_process([sys.executable, *command, '--headless'], cwd=ROOT, environment=environment)
    else:
        status = launch_display(command, root=ROOT, robot=args.robot_type + '_ros',
                                webrtc=args.webrtc, server_ip=args.server_ip)
    # Kit can exit Python during app.close(), skipping code after that call.
    # Inspect the fresh worker result even if its process exit status was zero.
    if status == 0:
        result = json.loads(report_path.read_text()) if report_path.is_file() else {}
        if not result.get('passed') and not result.get('stopped_by_user'):
            print(f'Robot simulation failed; inspect {report_path}', file=sys.stderr)
            status = 1
    raise SystemExit(status)


def worker():
    prepare_environment(ROOT, 'robot_ros')
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    add_arguments(parser)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    validate(parser, args)
    args.kit_args += ' ' + kit_arguments(ROOT, args.robot_type + '_ros') + ' --/isaac/startup/ros_bridge_extension='
    device = args.device
    app = AppLauncher(args).app
    args.device = device
    report = {'passed': False, 'scenario': args.scenario, 'cmd_vel_topic': args.cmd_vel_topic,
              'ros_domain_id': args.ros_domain_id, 'command_timeout': args.command_timeout}
    try:
        # Command/clock transport uses the bundled rclpy directly. No camera or
        # OmniGraph publisher is needed, so keep the rendering bridge disabled.
        if args.robot_type == 'go2':
            from robot_sim.runtime import run_ros
        elif args.robot_type == 'lekiwi':
            from robot_sim.lekiwi_runtime import run_ros
        else:
            from robot_sim.multi_runtime import run_ros
        run_ros(app, args, ROOT, report)
    except KeyboardInterrupt:
        report['stopped_by_user'] = True
    except Exception as exc:
        report['passed'] = False
        report['error'] = f'{type(exc).__name__}: {exc}'
        import traceback
        traceback.print_exc()
    finally:
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        app.close()
    if not report['passed'] and not report.get('stopped_by_user'):
        raise SystemExit('Robot ROS run did not complete successfully.')


if __name__ == '__main__':
    if '--worker' in sys.argv:
        sys.argv.remove('--worker')
        worker()
    else:
        main()
