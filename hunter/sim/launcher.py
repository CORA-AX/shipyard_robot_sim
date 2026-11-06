"""Hunter-local Kit paths and managed simulator process shutdown."""
import os
from pathlib import Path
import signal
import subprocess


def prepare_environment(root: Path, robot: str, environment=None):
    environment = os.environ if environment is None else environment
    environment.update(PYTHONNOUSERSITE='1', PYTHONUNBUFFERED='1')
    for name, relative in (
        ('XDG_CACHE_HOME', f'.cache/{robot}/xdg'),
        ('XDG_CONFIG_HOME', f'.runtime/{robot}/config'),
        ('XDG_DATA_HOME', f'.runtime/{robot}/data'),
        ('CUDA_CACHE_PATH', f'.cache/{robot}/cuda'),
    ):
        directory = root / relative
        directory.mkdir(parents=True, exist_ok=True)
        environment[name] = str(directory)
    return environment


def kit_arguments(root: Path, robot: str) -> str:
    return (
        f'--portable --portable-root {root}/.runtime/{robot}/kit'
        ' --/renderer/multiGpu/enabled=false --/renderer/multiGpu/autoEnable=false'
        ' --/app/extensions/fsWatcherEnabled=false'
    )


def run_process(command, *, cwd: Path, environment, shutdown_timeout=10):
    """Forward termination to the simulator and reap it before returning."""
    child = subprocess.Popen(command, cwd=cwd, env=environment, start_new_session=True)
    previous_handlers = {}
    stop_requested = None

    def send_signal(signum):
        try:
            os.killpg(child.pid, signum)
        except ProcessLookupError:
            pass

    def stop(signum, _frame):
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
