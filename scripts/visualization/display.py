"""Select a local window or headless WebRTC without importing Isaac Sim."""
import argparse
import os
from pathlib import Path
import sys

from .runtime import prepare_environment
from .webrtc import STREAM_ENTRY, WebRTCConfig, check_port_available, run_process, run_webrtc


def add_display_arguments(parser, *, default=False):
    parser.add_argument('--webrtc', action=argparse.BooleanOptionalAction, default=default,
                        help='Use headless WebRTC; --no-webrtc opens a local window.')
    parser.add_argument('--server-ip', default=os.environ.get('PUBLIC_IP'),
                        help='WebRTC server address reachable by the client (or PUBLIC_IP).')


def launch_display(command, *, root, robot, webrtc, server_ip):
    if webrtc:
        config = WebRTCConfig(server_ip)
        check_port_available()
        print(config.connection_hint(), flush=True)
        command = [*command, *config.app_arguments(), '--headless', '--kit_args', config.kit_arguments()]
        return run_webrtc(command, root=root, robot=robot, config=config)
    environment = prepare_environment(root, robot, os.environ.copy())
    environment.update(HEADLESS='0', LIVESTREAM='0')
    if not environment.get('DISPLAY') and Path('/tmp/.X11-unix/X0').exists():
        environment['DISPLAY'] = ':0'
        authority = Path.home() / '.Xauthority'
        if authority.is_file():
            environment.setdefault('XAUTHORITY', str(authority))
    print('Opening a local Isaac Sim window on the server desktop.', flush=True)
    return run_process([sys.executable, str(STREAM_ENTRY), *command, '--livestream', '0'],
                       cwd=root, environment=environment)


def dispatch_display(script, *, root, robot):
    """Route explicit display flags before a simulation script imports Kit."""
    parser = argparse.ArgumentParser(add_help=False)
    add_display_arguments(parser, default=None)
    args, remaining = parser.parse_known_args()
    if args.webrtc is None:
        return
    if any(flag in remaining for flag in ('--headless', '--livestream', '--experience', '--kit_args')):
        parser.error('Use --webrtc/--no-webrtc alone to select the display backend.')
    try:
        status = launch_display([str(script), *remaining], root=root, robot=robot,
                                webrtc=args.webrtc, server_ip=args.server_ip)
    except ValueError as exc:
        parser.error(str(exc))
    raise SystemExit(status)
