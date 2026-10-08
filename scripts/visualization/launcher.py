"""Local/WebRTC launch, app bootstrap and managed simulator shutdown."""
import argparse
import runpy
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys

if __package__:
    from .runtime import prepare_environment
else:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from visualization.runtime import prepare_environment

STREAM_ENTRY = Path(__file__).resolve()
SIGNALING_PORT = 49100
VIDEO_PORT = 47998
ASSET_ROOT = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0'


@dataclass(frozen=True)
class WebRTCConfig:
    server_ip: str

    def __post_init__(self):
        if not self.server_ip or any(c.isspace() for c in self.server_ip):
            raise ValueError('Set PUBLIC_IP or pass --server-ip with the reachable server address.')

    def app_arguments(self):
        return ['--livestream', '1', '--experience', 'isaacsim.exp.full.streaming.kit']

    def kit_arguments(self):
        settings = (
            '--/isaac/startup/ros_bridge_extension='
            ' --/rtx/materialDb/syncLoads=false'
            ' --/rtx/hydra/materialSyncLoads=false'
        )
        for name in ('default', 'cloud', 'nvidia'):
            settings += f' --/persistent/isaac/asset_root/{name}={ASSET_ROOT}'
        return settings

    def connection_hint(self):
        return (f'connect the WebRTC client to {self.server_ip} '
                f'(TCP {SIGNALING_PORT}, UDP {VIDEO_PORT}).')


def check_port_available(port=SIGNALING_PORT):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(('0.0.0.0', port))
        except OSError as exc:
            raise ValueError(
                f'WebRTC port {port} is unavailable: {exc}. Stop the existing Isaac Sim first.'
            ) from exc


def run_process(command, *, cwd: Path, environment, shutdown_timeout=10):
    """Forward termination to the simulator and reap it before the launcher exits."""
    child = subprocess.Popen(command, cwd=cwd, env=environment, start_new_session=True)
    previous_handlers = {}
    stop_requested = None

    def send_signal(signum):
        try:
            os.killpg(child.pid, signum)
        except ProcessLookupError:
            pass

    def stop(signum, _frame):
        # Popen.wait() holds an internal lock. Calling poll()/wait() from a
        # signal handler can deadlock that same thread; defer cleanup to the loop.
        nonlocal stop_requested
        if stop_requested is None:
            stop_requested = signum

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, stop)
    try:
        while True:
            if stop_requested is not None:
                send_signal(stop_requested)
                try:
                    child.wait(timeout=shutdown_timeout)
                except subprocess.TimeoutExpired:
                    send_signal(signal.SIGKILL)
                    child.wait()
                return 128 + stop_requested
            try:
                return child.wait(timeout=0.1)
            except subprocess.TimeoutExpired:
                continue
    finally:
        if child.poll() is None:
            send_signal(signal.SIGKILL)
            child.wait()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def run_webrtc(script_arguments, *, root: Path, robot: str, config: WebRTCConfig):
    environment = prepare_environment(root, robot, os.environ.copy())
    environment['PUBLIC_IP'] = config.server_ip
    command = [sys.executable, str(STREAM_ENTRY), "--entry", *script_arguments]
    return run_process(command, cwd=root, environment=environment)


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
    return run_process([sys.executable, str(STREAM_ENTRY), "--entry", *command, '--livestream', '0'],
                       cwd=root, environment=environment)


def configure_streaming_app():
    from isaacsim import SimulationApp

    # CLI settings are reset by SimulationApp.reset_render_settings().
    # Avoid creating another empty stage before the display is ready.
    SimulationApp.DEFAULT_LAUNCHER_CONFIG.update(sync_loads=False, hide_ui=False, create_new_stage=False)
    from isaaclab.app import AppLauncher

    original_init = AppLauncher.__init__

    def launch(app_launcher, *args, **kwargs):
        original_init(app_launcher, *args, **kwargs)
        import carb

        carb.settings.get_settings().set_bool('/omni.kit.plugin/syncUsdLoads', True)

    AppLauncher.__init__ = launch


def main():
    if len(sys.argv) < 3 or sys.argv[1] != '--entry':
        raise SystemExit('Use scripts/viewer.py.')
    script = Path(sys.argv[2]).resolve()
    sys.argv = [str(script), *sys.argv[3:]]
    sys.path.insert(0, str(script.parent))
    configure_streaming_app()
    runpy.run_path(str(script), run_name='__main__')


if __name__ == '__main__':
    main()
