#!/usr/bin/env python3
"""Run the selected Go2 policy and Isaac Sim in one ROS-controlled process."""
import argparse
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from control.go2.ros_bridge import ros_environment
from control.go2.keyboard import add_keyboard_arguments, validate_keyboard_arguments
from scenarios.registry import DEFAULT_SCENARIO, GO2_SCENARIOS
from visualization.launcher import add_display_arguments, launch_display, run_process
from visualization.runtime import kit_arguments, prepare_environment


def add_arguments(parser):
    add_keyboard_arguments(parser)
    parser.add_argument('--scenario', choices=GO2_SCENARIOS, default=DEFAULT_SCENARIO)
    parser.add_argument('--cmd-vel-topic', default='/unitree_go2/cmd_vel')
    parser.add_argument('--command-timeout', type=float, default=0.5,
                        help='Wall seconds without valid cmd_vel before commanding zero velocity.')
    parser.add_argument('--ros-domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--policy-device', default='cpu')
    parser.add_argument('--checkpoint', type=Path,
                        help='Defaults to the selected control_policy in the asset manifest.')
    parser.add_argument('--steps', type=int, default=0, help='50Hz policy steps; 0 runs until stopped.')
    parser.add_argument('--report', type=Path)


def validate(parser, args):
    validate_keyboard_arguments(parser, args)
    if Path(sys.prefix).resolve() != (ROOT / '.conda/isaacsim-5.0').resolve():
        parser.error(f'Activate {ROOT / ".conda/isaacsim-5.0"} first.')
    if (args.steps < 0 or not math.isfinite(args.command_timeout) or args.command_timeout <= 0
            or not 0 <= args.ros_domain_id <= 232):
        parser.error('Use nonnegative steps, a finite positive timeout and ROS domain 0..232.')
    if not args.cmd_vel_topic.startswith('/'):
        parser.error('--cmd-vel-topic must be an absolute ROS topic name.')


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
    command = [str(Path(__file__).resolve()), '--worker', '--scenario', args.scenario,
               '--device', args.device, '--policy-device', args.policy_device,
               '--cmd-vel-topic', args.cmd_vel_topic, '--command-timeout', str(args.command_timeout),
               '--ros-domain-id', str(args.ros_domain_id), '--steps', str(args.steps)]
    if args.checkpoint:
        command += ['--checkpoint', str(args.checkpoint.resolve())]
    if args.report:
        command += ['--report', str(args.report.resolve())]
    if args.keyboard:
        command += ['--keyboard', '--keyboard-speed', str(args.keyboard_speed),
                    '--keyboard-turn-speed', str(args.keyboard_turn_speed),
                    '--keyboard-timeout', str(args.keyboard_timeout)]
    if args.headless:
        environment = prepare_environment(ROOT, 'go2_ros', environment)
        status = run_process([sys.executable, *command, '--headless'], cwd=ROOT, environment=environment)
    else:
        status = launch_display(command, root=ROOT, robot='go2_ros',
                                webrtc=args.webrtc, server_ip=args.server_ip)
    raise SystemExit(status)


def worker():
    prepare_environment(ROOT, 'go2_ros')
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    add_arguments(parser)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    validate(parser, args)
    args.kit_args += ' ' + kit_arguments(ROOT, 'go2_ros') + ' --/isaac/startup/ros_bridge_extension='
    device = args.device
    app = AppLauncher(args).app
    args.device = device
    report = {'passed': False, 'scenario': args.scenario, 'cmd_vel_topic': args.cmd_vel_topic,
              'ros_domain_id': args.ros_domain_id, 'command_timeout': args.command_timeout}
    try:
        # Command/clock transport uses the bundled rclpy directly. No camera or
        # OmniGraph publisher is needed, so keep the rendering bridge disabled.
        from control.go2.ros_runtime import run_ros
        run_ros(app, args, ROOT, report)
    except KeyboardInterrupt:
        report['stopped_by_user'] = True
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        import traceback
        traceback.print_exc()
    finally:
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        app.close()
    if not report['passed'] and not report.get('stopped_by_user'):
        raise SystemExit('Go2 ROS run did not complete successfully.')


if __name__ == '__main__':
    if '--worker' in sys.argv:
        sys.argv.remove('--worker')
        worker()
    else:
        main()
