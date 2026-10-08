"""WebRTC settings and managed simulator processes. No robot or policy dependencies."""
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys

from .runtime import prepare_environment

STREAM_ENTRY = Path(__file__).with_name('stream_entry.py')
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
    command = [sys.executable, str(STREAM_ENTRY), *script_arguments]
    return run_process(command, cwd=root, environment=environment)
