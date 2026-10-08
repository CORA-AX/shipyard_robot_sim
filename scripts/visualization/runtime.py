"""Project-local Kit settings and display timing. No simulator imports at module load."""
import os
from pathlib import Path
import time


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


class FramePacer:
    """Wait for display time without changing the simulator's physics time step."""

    def __init__(self, enabled: bool):
        self.enabled = enabled
        self.started = 0.0

    def start(self):
        self.started = time.perf_counter()

    def wait(self, step_dt: float):
        if self.enabled:
            time.sleep(max(0.0, step_dt - (time.perf_counter() - self.started)))
